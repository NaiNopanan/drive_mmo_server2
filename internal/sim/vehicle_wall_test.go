package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// defaultWalls returns a 100x100 arena centered at origin, no bounce
func defaultWalls() WallBounds {
	return WallBounds{
		MinX:        fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ:        fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
		Restitution: fixed.Zero,
	}
}

// bouncyWalls returns a 100x100 arena with 70% restitution
func bouncyWalls() WallBounds {
	return WallBounds{
		MinX:        fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ:        fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
		Restitution: fixed.FromFraction(7, 10),
	}
}

// settleVehicleOnGround runs physics on flat ground to bring a vehicle to rest.
func settleVehicleOnGround(v *Vehicle, dt fixed.Fixed, ticks int) {
	g := FlatGround{Y: fixed.Zero}
	for i := 0; i < ticks; i++ {
		v.Step(dt, g)
	}
}

// ─── Wall Contact Detection ───────────────────────────────────────────────────

func TestWall_NoContactWhenInsideBounds(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	wb := defaultWalls()
	contact := CheckWallContact(&v, wb)
	if contact.Hit {
		t.Fatalf("expected no wall contact for vehicle at center, got depth=%v", contact.Depth)
	}
}

func TestWall_ContactDetected_MaxX(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.FromInt(50), fixed.Zero, fixed.Zero))
	wb := defaultWalls()
	contact := CheckWallContact(&v, wb)
	if !contact.Hit {
		t.Fatalf("expected wall contact at X=50")
	}
	if contact.Normal.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected normal pointing in -X direction, got=%v", contact.Normal)
	}
}

// ─── Wall Collision Response ──────────────────────────────────────────────────

func TestWall_StopsAtMaxX(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	wb := defaultWalls()
	dt := fixed.FromFraction(1, 60)

	settleVehicleOnGround(&v, dt, 120)
	for i := 0; i < 300; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)
		CollideVehicleWithWalls(&v, wb)
	}

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
		t.Fatalf("vehicle passed through +X wall: pos.X=%v", v.Position.X)
	}
}

func TestWall_StopsAtMinX(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	wb := defaultWalls()
	dt := fixed.FromFraction(1, 60)

	settleVehicleOnGround(&v, dt, 120)
	v.Velocity.X = fixed.FromInt(-30)
	for i := 0; i < 60; i++ {
		v.Step(dt, g)
		CollideVehicleWithWalls(&v, wb)
	}

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	if v.Position.X.Sub(halfW).Cmp(wb.MinX) < 0 {
		t.Fatalf("vehicle passed through -X wall: pos.X=%v", v.Position.X)
	}
}

func TestWall_StopsAtMaxZ(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	wb := defaultWalls()
	dt := fixed.FromFraction(1, 60)

	settleVehicleOnGround(&v, dt, 120)
	v.Velocity.Z = fixed.FromInt(30)
	for i := 0; i < 60; i++ {
		v.Step(dt, g)
		CollideVehicleWithWalls(&v, wb)
	}

	halfL := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	if v.Position.Z.Add(halfL).Cmp(wb.MaxZ) > 0 {
		t.Fatalf("vehicle passed through +Z wall: pos.Z=%v", v.Position.Z)
	}
}

func TestWall_VelocityZeroedOnImpact(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.FromInt(50), fixed.Zero, fixed.Zero))
	wb := defaultWalls()
	v.Velocity.X = fixed.FromInt(20)
	CollideVehicleWithWalls(&v, wb)
	if v.Velocity.X.Cmp(fixed.Zero) > 0 {
		t.Fatalf("expected X velocity zeroed after hitting wall, got=%v", v.Velocity.X)
	}
}

func TestWall_BounceReverses(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.FromInt(50), fixed.Zero, fixed.Zero))
	wb := bouncyWalls()
	v.Velocity.X = fixed.FromInt(10)
	CollideVehicleWithWalls(&v, wb)

	expected := fixed.FromInt(-7)
	tol := fixed.FromFraction(1, 10)
	diff := v.Velocity.X.Sub(expected).Abs()
	if diff.Cmp(tol) > 0 {
		t.Fatalf("expected bounced velocity ~%v, got %v (diff=%v)", expected, v.Velocity.X, diff)
	}
}

func TestWall_NoTunnel_TeleportIntoWall(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.FromInt(55), fixed.Zero, fixed.Zero))
	wb := defaultWalls()
	v.Velocity.X = fixed.FromInt(5)
	CollideVehicleWithWalls(&v, wb)

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
		t.Fatalf("vehicle still past wall after resolution: pos.X=%v", v.Position.X)
	}
}

func TestWall_Deterministic(t *testing.T) {
	g := FlatGround{Y: fixed.Zero}
	wb := bouncyWalls()
	dt := fixed.FromFraction(1, 60)

	va := NewVehicle(1, geom.V3(fixed.FromInt(48), fixed.FromInt(3), fixed.Zero))
	vb := NewVehicle(1, geom.V3(fixed.FromInt(48), fixed.FromInt(3), fixed.Zero))

	for i := 0; i < 120; i++ {
		va.Input = VehicleInput{Throttle: fixed.One}
		vb.Input = VehicleInput{Throttle: fixed.One}
		va.Step(dt, g)
		vb.Step(dt, g)
		CollideVehicleWithWalls(&va, wb)
		CollideVehicleWithWalls(&vb, wb)
	}

	ha := HashVehicle(va)
	hb := HashVehicle(vb)
	if ha != hb {
		t.Fatalf("wall collision is not deterministic: hash mismatch %d != %d", ha, hb)
	}
}

// ─── High-Speed Wall Collision Tests ─────────────────────────────────────────

// TestWall_HighSpeed_NoTunnel verifies that a vehicle at 200 m/s does NOT
// tunnel through the wall in a single CollideVehicleWithWalls call.
func TestWall_HighSpeed_NoTunnel(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.Zero, fixed.Zero))
	wb := defaultWalls()

	v.Velocity.X = fixed.FromInt(200)
	CollideVehicleWithWalls(&v, wb)

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
		t.Fatalf("tunneled through wall at 200 m/s: pos.X=%v maxX=%v", v.Position.X, wb.MaxX)
	}
}

// TestWall_HighSpeed_VelocityReversed verifies that after a high-speed bounce
// the velocity direction is reversed (moving away from wall).
func TestWall_HighSpeed_VelocityReversed(t *testing.T) {
	// Start the vehicle at the wall edge so the collision fires immediately
	v := NewVehicle(1, geom.V3(fixed.FromInt(50), fixed.Zero, fixed.Zero))
	wb := bouncyWalls()

	v.Velocity.X = fixed.FromInt(200)
	CollideVehicleWithWalls(&v, wb)

	if v.Velocity.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected reversed velocity after high-speed wall impact, got=%v", v.Velocity.X)
	}
}

// TestWall_HighSpeed_ExceedsArenaWidth tests a velocity so extreme (30,000 m/s)
// that the vehicle would cross the entire 100-unit arena in a single tick.
func TestWall_HighSpeed_ExceedsArenaWidth(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.Zero, fixed.Zero))
	wb := defaultWalls()

	v.Velocity.X = fixed.FromInt(30000)
	CollideVehicleWithWalls(&v, wb)

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
		t.Fatalf("escaped arena at 30,000 m/s: pos.X=%v", v.Position.X)
	}
	if v.Position.X.Sub(halfW).Cmp(wb.MinX) < 0 {
		t.Fatalf("went past -X wall at 30,000 m/s: pos.X=%v", v.Position.X)
	}
}

// TestWall_HighSpeed_ZAxis_NoTunnel tests extreme speed (500 m/s) in the Z axis.
func TestWall_HighSpeed_ZAxis_NoTunnel(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.Zero, fixed.Zero))
	wb := defaultWalls()

	v.Velocity.Z = fixed.FromInt(500)
	CollideVehicleWithWalls(&v, wb)

	halfL := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	if v.Position.Z.Add(halfL).Cmp(wb.MaxZ) > 0 {
		t.Fatalf("tunneled through +Z wall at 500 m/s: pos.Z=%v maxZ=%v", v.Position.Z, wb.MaxZ)
	}
}

// TestWall_HighSpeed_Continuous simulates 5 seconds of physics at 100 m/s
// with bouncy walls. Checks every single tick that vehicle never escapes.
func TestWall_HighSpeed_Continuous(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	wb := WallBounds{
		MinX:        fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ:        fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
		Restitution: fixed.FromFraction(8, 10),
	}
	dt := fixed.FromFraction(1, 60)

	settleVehicleOnGround(&v, dt, 120)
	v.Velocity.X = fixed.FromInt(100)
	v.Velocity.Z = fixed.FromInt(100)

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfL := v.Tuning.WheelBase.Div(fixed.FromInt(2))

	for tick := 0; tick < 300; tick++ {
		v.Step(dt, g)
		CollideVehicleWithWalls(&v, wb)

		if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
			t.Fatalf("tick %d: tunneled through +X wall: pos.X=%v", tick, v.Position.X)
		}
		if v.Position.X.Sub(halfW).Cmp(wb.MinX) < 0 {
			t.Fatalf("tick %d: tunneled through -X wall: pos.X=%v", tick, v.Position.X)
		}
		if v.Position.Z.Add(halfL).Cmp(wb.MaxZ) > 0 {
			t.Fatalf("tick %d: tunneled through +Z wall: pos.Z=%v", tick, v.Position.Z)
		}
		if v.Position.Z.Sub(halfL).Cmp(wb.MinZ) < 0 {
			t.Fatalf("tick %d: tunneled through -Z wall: pos.Z=%v", tick, v.Position.Z)
		}
	}
}

// TestWall_HighSpeed_LargeDt simulates poor conditions (10fps) combined with
// high velocity (50 m/s). Each tick moves 5m, much larger than 1-unit grid cells.
func TestWall_HighSpeed_LargeDt(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	wb := defaultWalls()
	dt := fixed.FromFraction(1, 10) // 10 fps

	settleVehicleOnGround(&v, dt, 30)
	v.Velocity.X = fixed.FromInt(50)

	halfW := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	for tick := 0; tick < 60; tick++ {
		v.Step(dt, g)
		CollideVehicleWithWalls(&v, wb)

		if v.Position.X.Add(halfW).Cmp(wb.MaxX) > 0 {
			t.Fatalf("tick %d (10fps, 50m/s): tunneled through wall: pos.X=%v", tick, v.Position.X)
		}
	}
}
