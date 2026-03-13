package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func assertVehicleAnchorsNotBelowFlatGround(t *testing.T, v *Vehicle, groundY fixed.Fixed) {
	t.Helper()

	minAnchorY := groundY.Add(v.Tuning.WheelRadius).Add(v.minSuspensionLength())

	for i := range v.Wheels {
		anchor := v.Position.Add(v.LocalToWorld(v.WheelDefs[i].LocalAnchor))
		if anchor.Y.Add(fixed.FromFraction(1, 1000)).Cmp(minAnchorY) < 0 {
			t.Fatalf("wheel %d anchor below allowed min: got=%v min=%v", i, anchor.Y, minAnchorY)
		}
	}
}

func assertVehicleAnchorsNotBelowSlopeGround(t *testing.T, v *Vehicle, g SlopeGround) {
	t.Helper()

	for i := range v.Wheels {
		anchor := v.Position.Add(v.LocalToWorld(v.WheelDefs[i].LocalAnchor))
		groundY := g.BaseY.Add(g.Slope.Mul(anchor.Z))
		minAnchorY := groundY.Add(v.Tuning.WheelRadius).Add(v.minSuspensionLength())

		// Relax by small epsilon
		if anchor.Y.Add(fixed.FromFraction(1, 1000)).Cmp(minAnchorY) < 0 {
			t.Fatalf("wheel %d anchor below slope min: got=%v min=%v", i, anchor.Y, minAnchorY)
		}
	}
}

func TestFindGroundTOIFlat_SelectsEarliestWheel(t *testing.T) {
	v := NewVehicle(1, geom.Zero())

	pose0 := VehiclePose{
		Position: geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero),
		Yaw:      fixed.Zero,
	}
	pose1 := VehiclePose{
		Position: geom.V3(fixed.Zero, fixed.FromInt(-1), fixed.Zero),
		Yaw:      fixed.Zero,
	}

	toi := v.FindGroundTOI(pose0, pose1, FlatGround{Y: fixed.Zero})
	if !toi.Hit {
		t.Fatalf("expected flat TOI hit")
	}

	if toi.Time.Cmp(fixed.Zero) < 0 || toi.Time.Cmp(fixed.One) > 0 {
		t.Fatalf("toi time out of range: %v", toi.Time)
	}
}

func TestFindGroundTOISlope_Hits(t *testing.T) {
	v := NewVehicle(1, geom.Zero())

	pose0 := VehiclePose{
		Position: geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero),
		Yaw:      fixed.Zero,
	}
	pose1 := VehiclePose{
		Position: geom.V3(fixed.Zero, fixed.FromInt(-1), fixed.FromInt(1)),
		Yaw:      fixed.Zero,
	}

	toi := v.FindGroundTOI(pose0, pose1, SlopeGround{
		BaseY: fixed.Zero,
		Slope: fixed.FromFraction(1, 10), // 0.1
	})
	if !toi.Hit {
		t.Fatalf("expected slope TOI hit")
	}
}

func TestCCD_HighSpawn_Flat_NoTunnel(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(20), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 600; i++ {
		v.Step(dt, g)
	}

	if !v.OnGround {
		t.Fatalf("expected vehicle grounded after fall")
	}
	if v.GroundedWheels == 0 {
		t.Fatalf("expected grounded wheels > 0")
	}

	assertVehicleAnchorsNotBelowFlatGround(t, &v, g.Y)

	if v.Velocity.Y.Abs().Cmp(fixed.One) > 0 {
		t.Fatalf("expected vertical velocity settled, got=%v", v.Velocity.Y)
	}
}

func TestCCD_LargeDt_Flat_NoTunnel(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(20), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 15)

	for i := 0; i < 240; i++ {
		v.Step(dt, g)
	}

	if !v.OnGround {
		t.Fatalf("expected grounded even with large dt")
	}

	assertVehicleAnchorsNotBelowFlatGround(t, &v, g.Y)
}

func TestCCD_HighSpawn_Slope_NoTunnel(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(20), fixed.Zero))
	g := SlopeGround{
		BaseY: fixed.Zero,
		Slope: fixed.FromFraction(1, 10),
	}
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 600; i++ {
		v.Step(dt, g)
	}

	if !v.OnGround {
		t.Fatalf("expected grounded on slope")
	}
	if v.GroundedWheels == 0 {
		t.Fatalf("expected grounded wheels on slope")
	}

	assertVehicleAnchorsNotBelowSlopeGround(t, &v, g)

	if v.Velocity.Y.Abs().Cmp(fixed.One) > 0 {
		t.Fatalf("expected vertical velocity settled on slope, got=%v", v.Velocity.Y)
	}
}

func TestCCD_ImmediatePenetration_ResolvedAtTimeZero(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromFraction(-25, 100), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	v.Step(dt, g)

	assertVehicleAnchorsNotBelowFlatGround(t, &v, g.Y)
}

func TestCCD_DriveOffEdge_Flat_Falls(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
	}
	dt := fixed.FromFraction(1, 60)

	// Step a few times to settle
	for i := 0; i < 60; i++ {
		v.Step(dt, g)
	}
	if !v.OnGround {
		t.Fatalf("expected vehicle grounded at center")
	}

	// Move far off edge (Z > 100)
	v.Position.Z = fixed.FromInt(100)
	v.Step(dt, g)

	if v.OnGround {
		t.Fatalf("expected vehicle NOT grounded after moving off edge to Z=100")
	}
}

// ===== Vehicle Falling Test Cases =====

// TestCCD_Fall_VehicleDescendsWhenOffEdge verifies that once off the edge,
// the vehicle's Y position decreases each tick (gravity pulls it down).
func TestCCD_Fall_VehicleDescendsWhenOffEdge(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
	}
	dt := fixed.FromFraction(1, 60)

	// Settle on ground first
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Teleport off edge
	v.Position.Z = fixed.FromInt(100)
	v.Velocity.Y = fixed.Zero // clear any vertical velocity

	startY := v.Position.Y

	// Step several times - Y should decrease (falling)
	for i := 0; i < 30; i++ {
		v.Step(dt, g)
	}

	if v.Position.Y.Cmp(startY) >= 0 {
		t.Fatalf("expected vehicle to fall after going off edge: startY=%v currentY=%v", startY, v.Position.Y)
	}
	if v.OnGround {
		t.Fatalf("vehicle should NOT be grounded while falling off edge")
	}
}

// TestCCD_Fall_VeryHighSpawn_StillLandsCorrectly verifies a vehicle spawned at
// extreme altitude (100m) still lands without tunneling.
func TestCCD_Fall_VeryHighSpawn_StillLandsCorrectly(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(100), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Run for 10 seconds
	for i := 0; i < 600; i++ {
		v.Step(dt, g)
	}

	if !v.OnGround {
		t.Fatalf("expected vehicle to land from Y=100 within 10 seconds")
	}
	assertVehicleAnchorsNotBelowFlatGround(t, &v, g.Y)
}

// TestCCD_Fall_NoGround_Freefall verifies that with no ground at all,
// the vehicle continuously falls (Y decreases) and never reports OnGround.
func TestCCD_Fall_NoGround_FreefallingIndefinitely(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero))
	// Use an empty WorldGroundQuery with no triangles = no ground
	g := WorldGroundQuery{Triangles: nil}
	dt := fixed.FromFraction(1, 60)

	startY := v.Position.Y
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	if v.Position.Y.Cmp(startY) >= 0 {
		t.Fatalf("expected Y to decrease without ground: startY=%v endY=%v", startY, v.Position.Y)
	}
	if v.OnGround {
		t.Fatalf("vehicle should never be OnGround with no terrain")
	}
}

// TestCCD_Fall_OffSlopeEdge verifies that driving off a finite slope
// causes the vehicle to fall and lose ground contact.
func TestCCD_Fall_OffSlopeEdge(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(5), fixed.Zero))
	g := SlopeGround{
		BaseY: fixed.Zero,
		Slope: fixed.FromFraction(1, 10),
		MinX:  fixed.FromInt(-40), MaxX: fixed.FromInt(40),
		MinZ:  fixed.FromInt(-40), MaxZ: fixed.FromInt(40),
	}
	dt := fixed.FromFraction(1, 60)

	// Settle on slope
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}
	if !v.OnGround {
		t.Fatalf("expected vehicle to settle on slope")
	}

	// Move off slope edge
	v.Position.X = fixed.FromInt(100)
	v.Position.Z = fixed.FromInt(100)
	v.Step(dt, g)

	if v.OnGround {
		t.Fatalf("expected vehicle to fall after driving off slope edge")
	}
}

// TestCCD_Fall_MultipleVehicles_AllFallSimultaneously verifies that when
// multiple vehicles are pushed off the edge at the same time, all of them fall.
func TestCCD_Fall_MultipleVehicles_AllFallSimultaneously(t *testing.T) {
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
	}
	dt := fixed.FromFraction(1, 60)

	// Create 4 vehicles in a 2x2 grid, settle them
	world := VehicleWorldDay6{
		Ground: g,
		Vehicles: []Vehicle{
			NewVehicle(1, geom.V3(fixed.FromInt(2), fixed.FromInt(3), fixed.FromInt(2))),
			NewVehicle(2, geom.V3(fixed.FromInt(-2), fixed.FromInt(3), fixed.FromInt(2))),
			NewVehicle(3, geom.V3(fixed.FromInt(2), fixed.FromInt(3), fixed.FromInt(-2))),
			NewVehicle(4, geom.V3(fixed.FromInt(-2), fixed.FromInt(3), fixed.FromInt(-2))),
		},
	}

	for i := 0; i < 120; i++ {
		world.Step(dt)
	}

	// Verify all are grounded
	for i, v := range world.Vehicles {
		if !v.OnGround {
			t.Fatalf("vehicle %d should be on ground before test", i)
		}
	}

	// Move all vehicles off the edge
	for i := range world.Vehicles {
		world.Vehicles[i].Position.X = fixed.FromInt(200)
		world.Vehicles[i].Position.Z = fixed.FromInt(200)
	}

	world.Step(dt)

	// Verify all have fallen off
	for i, v := range world.Vehicles {
		if v.OnGround {
			t.Fatalf("vehicle %d should NOT be grounded after going off edge", i)
		}
	}
}
