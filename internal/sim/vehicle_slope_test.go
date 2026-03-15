package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// testSlopeGround returns a SlopeGround with 8m rise over 80m depth (~5.7°)
// Slope = rise/run = 8/80 = 0.1 (dy per dz)
func testSlopeGround() SlopeGround {
	return SlopeGround{
		BaseY: fixed.Zero,
		Slope: fixed.FromFraction(1, 10), // 0.1 = 8m/80m
		MinX:  fixed.FromInt(-100), MaxX: fixed.FromInt(100),
		MinZ: fixed.FromInt(-100), MaxZ: fixed.FromInt(100),
	}
}

// slopeHeightProxy measures chassis height relative to the analytic slope
// at the vehicle's current Z. It is not a true suspension length, but it is
// a useful regression proxy: if the car suddenly "sinks" into the slope,
// this value will collapse sharply compared to its settled baseline.
func slopeHeightProxy(v *Vehicle, g SlopeGround) fixed.Fixed {
	groundY := g.BaseY.Add(g.Slope.Mul(v.Position.Z))
	return v.Position.Y.Sub(groundY)
}

func settleVehicleOnSlope(v *Vehicle, g GroundQuery, dt fixed.Fixed, ticks int) {
	v.Input = VehicleInput{}
	for i := 0; i < ticks; i++ {
		v.Step(dt, g)
	}
}

// TestSlope_VehicleClimbsWithFullThrottle verifies the vehicle can climb
// the slope continuously when full throttle is applied.
func TestSlope_VehicleClimbsWithFullThrottle(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)

	// Spawn at low end of slope (Z = -30, Y ≈ 0.25 on slope)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-30)))

	// Settle on slope
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	startZ := v.Position.Z
	startY := v.Position.Y
	t.Logf("Start pos: Z=%v Y=%v OnGround=%v", startZ, startY, v.OnGround)

	// Drive forward (toward high end) for 5 seconds with full throttle
	for i := 0; i < 300; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	t.Logf("End pos: Z=%v Y=%v OnGround=%v speed=%v",
		v.Position.Z, v.Position.Y, v.OnGround,
		geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length())

	// Vehicle should have moved forward (higher Z = higher on slope)
	if v.Position.Z.Sub(startZ).Cmp(fixed.FromInt(5)) < 0 {
		t.Errorf("vehicle did not climb slope: deltaZ=%v expected > 5m",
			v.Position.Z.Sub(startZ))
	}
	// Y should have gone UP (slope rises with Z)
	if v.Position.Y.Cmp(startY) <= 0 {
		t.Errorf("vehicle Y should increase on slope climb: startY=%v endY=%v",
			startY, v.Position.Y)
	}
}

// TestSlope_VehicleDoesNotSlideDown verifies a grounded vehicle on a slope
// doesn't accelerate uncontrollably downhill with no input.
func TestSlope_VehicleDoesNotSlideDown(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)

	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	// Settle in the middle of slope
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	startZ := v.Position.Z

	// No input — let gravity + rolling resistance balance
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	deltaZ := v.Position.Z.Sub(startZ)
	t.Logf("DeltaZ with no input on slope: %v", deltaZ)

	// Acceptable: some slide from gravity. NOT acceptable: >5m runaway downhill in 2s
	maxDownhillSlide := fixed.FromInt(-5) // -5m
	if deltaZ.Cmp(maxDownhillSlide) < 0 {
		t.Errorf("vehicle slid too far down slope with no input: deltaZ=%v (expected > -5m)",
			deltaZ)
	}
}

// TestSlope_VehicleClimbsAtHalfThrottle verifies slope climbing at partial throttle.
func TestSlope_VehicleClimbsAtHalfThrottle(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)

	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-30)))

	// Settle
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	startZ := v.Position.Z

	// Half throttle for 5 seconds
	for i := 0; i < 300; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(5, 10)}
		v.Step(dt, g)
	}

	t.Logf("Half-throttle climb: startZ=%v endZ=%v OnGround=%v", startZ, v.Position.Z, v.OnGround)

	if v.Position.Z.Sub(startZ).Cmp(fixed.FromInt(2)) < 0 {
		t.Errorf("vehicle did not climb slope at half throttle: deltaZ=%v expected > 2m",
			v.Position.Z.Sub(startZ))
	}
}

// TestVehicleClimbsTriangleSlope verifies that the vehicle can climb a slope
// composed of actual geometric triangles (WorldGroundQuery).
func TestVehicleClimbsTriangleSlope(t *testing.T) {
	g := WorldGroundQuery{Triangles: GroundSlopeSmall()}
	dt := fixed.FromFraction(1, 60)

	// GroundSlopeSmall is -40 to 40 in XZ.
	// Spawn at Z=-30
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-30)))

	// Settle
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	startZ := v.Position.Z
	startY := v.Position.Y
	t.Logf("Start pos: Z=%v Y=%v OnGround=%v", startZ, startY, v.OnGround)

	// Drive forward for 3 seconds; current drive force is high enough that 3s is sufficient.
	for i := 0; i < 180; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	t.Logf("End pos: Z=%v Y=%v OnGround=%v speed=%v",
		v.Position.Z, v.Position.Y, v.OnGround,
		geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length())

	// Should have climbed
	if v.Position.Z.Sub(startZ).Cmp(fixed.FromInt(5)) < 0 {
		t.Errorf("vehicle did not climb triangle slope: deltaZ=%v", v.Position.Z.Sub(startZ))
	}
	if v.Position.Y.Cmp(startY) <= 0 {
		t.Errorf("vehicle Y should increase on triangle slope: startY=%v endY=%v", startY, v.Position.Y)
	}
}

// TestSlope_DriveForceExceedsGravityComponent is a pure physics sanity check.
// Drive force must exceed the gravity component along the slope.
func TestSlope_DriveForceExceedsGravityComponent(t *testing.T) {
	slope := testSlopeGround()
	tuning := DefaultTuning()

	// Slope = dy/dz = 0.1 → sin(θ) = slope / sqrt(1 + slope²)
	s := slope.Slope
	one := fixed.One
	sinTheta := s.Div(one.Add(s.Mul(s)).Sqrt())

	gravityAlongSlope := tuning.Mass.Mul(fixed.FromFraction(98, 10)).Mul(sinTheta)
	t.Logf("Slope dy/dz: %v → sin(θ)=%v", slope.Slope, sinTheta)
	t.Logf("Gravity along slope: %v N", gravityAlongSlope)
	t.Logf("Total drive force: %v N", tuning.DriveForce)

	if tuning.DriveForce.Cmp(gravityAlongSlope) <= 0 {
		t.Errorf("drive force %v <= gravity on slope %v: vehicle CANNOT climb",
			tuning.DriveForce, gravityAlongSlope)
	}
}

// TestSlope_SteeringAuthority verifies if the vehicle can turn while on a slope.
func TestSlope_SteeringAuthority(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)

	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	// 1) Settle on slope
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// 2) Drive forward for 1 second to gain some speed
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(5, 10)}
		v.Step(dt, g)
	}

	startYaw := v.Yaw

	// 3) Hold full steering and moderate throttle for 2 seconds
	for i := 0; i < 120; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(5, 10), Steer: fixed.One}
		v.Step(dt, g)
	}

	yawDelta := v.Yaw.Sub(startYaw).Abs()
	t.Logf("Yaw change after 2s steering on slope: %v rad", yawDelta)

	// In 2 seconds at moderate speed, we expect at least 0.05 rad of turning
	// (New physics with speed-sensitive steering and higher inertia)
	minExpectedTurn := fixed.FromFraction(5, 100) // 0.05 rad
	if yawDelta.Cmp(minExpectedTurn) < 0 {
		t.Errorf("insufficient steering authority on slope: delta=%v expected >= %v",
			yawDelta, minExpectedTurn)
	}
}

// TestSlope_SteeringStability measures yaw velocity jitter during sustained steering on a slope.
func TestSlope_SteeringStability(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)

	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	// 1) Settle on slope
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// 2) Drive forward to gain speed
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	// 3) Hold full steering and measure yaw velocity direction changes (jitter)
	lastYawVel := v.YawVelocity
	jitterCount := 0
	for i := 0; i < 120; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)

		// If yaw velocity flips sign between ticks, it's oscillating/jittering
		if (v.YawVelocity.Cmp(fixed.Zero) > 0 && lastYawVel.Cmp(fixed.Zero) < 0) ||
			(v.YawVelocity.Cmp(fixed.Zero) < 0 && lastYawVel.Cmp(fixed.Zero) > 0) {
			jitterCount++
		}
		lastYawVel = v.YawVelocity
	}

	t.Logf("Yaw velocity jitter count over 120 ticks: %v", jitterCount)

	// In a smooth turn, jitter should be near zero (maybe 1-2 for initial kick, but not continuous)
	if jitterCount > 10 {
		t.Errorf("excessive steering vibration on slope: jitterCount=%v expected < 10", jitterCount)
	}
}

// TestSlope_NoRapidGroundContactFlappingWhileClimbing catches a common CCD/contact bug:
// while climbing a slope under steady throttle, the vehicle should not rapidly
// alternate between "stable ground contact" and "not grounded enough".
func TestSlope_NoRapidGroundContactFlappingWhileClimbing(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-20)))

	settleVehicleOnSlope(&v, g, dt, 180)

	// Build a little speed first so the contact system is exercised under load.
	// With AWD-only vehicles, half the previous throttle keeps the nominal drive
	// load close to the old baseline from the former 2-wheel-drive setup.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(1, 4)}
		v.Step(dt, g)
	}

	stableGround := v.GroundedWheels >= 2
	flipCount := 0
	unstableTicks := 0
	samples := 180

	for i := 0; i < samples; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(3, 10)}
		v.Step(dt, g)

		nowStable := v.GroundedWheels >= 2
		if nowStable != stableGround {
			flipCount++
		}
		if !nowStable {
			unstableTicks++
		}
		stableGround = nowStable
	}

	t.Logf("Ground-contact flaps over %d ticks: %d", samples, flipCount)
	t.Logf("Ticks with <2 grounded wheels: %d", unstableTicks)

	// We allow some loss of contact, but not repeated flap/jitter behavior.
	if flipCount > 12 {
		t.Fatalf("excessive ground-contact flapping while climbing slope: flips=%d expected<=12", flipCount)
	}
	if unstableTicks > 36 { // >20% of the sampled ticks
		t.Fatalf("vehicle spent too long in unstable contact while climbing slope: unstableTicks=%d expected<=36", unstableTicks)
	}
}

// TestSlope_NoBounceBurstOrSinkDuringClimb catches the "สั้นรัว/เด้งรัว"
// and "จม slope" style failure. On a smooth analytic slope with steady throttle,
// chassis height relative to the slope should stay in a bounded band and vertical
// velocity should not repeatedly flip sign with meaningful magnitude.
func TestSlope_NoBounceBurstOrSinkDuringClimb(t *testing.T) {
	g := testSlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-20)))

	settleVehicleOnSlope(&v, g, dt, 180)
	baseProxy := slopeHeightProxy(&v, g)
	startZ := v.Position.Z

	prevVy := v.Velocity.Y
	significantFlipCount := 0
	minProxy := baseProxy

	for i := 0; i < 180; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		v.Step(dt, g)

		proxy := slopeHeightProxy(&v, g)
		if proxy.Cmp(minProxy) < 0 {
			minProxy = proxy
		}

		// Count only meaningful vertical sign flips, not tiny noise near zero.
		if prevVy.Abs().Cmp(fixed.FromFraction(2, 10)) > 0 &&
			v.Velocity.Y.Abs().Cmp(fixed.FromFraction(2, 10)) > 0 {
			if (prevVy.Cmp(fixed.Zero) > 0 && v.Velocity.Y.Cmp(fixed.Zero) < 0) ||
				(prevVy.Cmp(fixed.Zero) < 0 && v.Velocity.Y.Cmp(fixed.Zero) > 0) {
				significantFlipCount++
			}
		}
		prevVy = v.Velocity.Y
	}

	deltaZ := v.Position.Z.Sub(startZ)
	proxyDrop := baseProxy.Sub(minProxy)

	t.Logf("Slope climb deltaZ: %v", deltaZ)
	t.Logf("Base height proxy: %v", baseProxy)
	t.Logf("Min height proxy during climb: %v", minProxy)
	t.Logf("Height proxy drop: %v", proxyDrop)
	t.Logf("Significant vertical velocity sign flips: %d", significantFlipCount)

	if deltaZ.Cmp(fixed.FromInt(3)) < 0 {
		t.Fatalf("vehicle did not make enough uphill progress during bounce/sink test: deltaZ=%v expected>=3m", deltaZ)
	}

	// If the proxy drops too much from its settled baseline, the chassis likely
	// collapsed/sank relative to the slope.
	if proxyDrop.Cmp(fixed.One) > 0 {
		t.Fatalf("vehicle appears to sink too far into slope during climb: proxyDrop=%v expected<=1.0", proxyDrop)
	}

	// Too many meaningful Vy sign flips indicates bounce burst / chatter.
	if significantFlipCount > 14 {
		t.Fatalf("excessive vertical bounce burst while climbing slope: significantFlipCount=%d expected<=14", significantFlipCount)
	}
}

// TestTriangleSlope_SteeringDoesNotDropOutWhileClimbing catches the case where
// the vehicle initially turns, then suddenly stops responding to steering on a
// real triangle slope (often perceived as "จมแล้วคุมไม่ได้").
func TestTriangleSlope_SteeringDoesNotDropOutWhileClimbing(t *testing.T) {
	g := WorldGroundQuery{Triangles: GroundSlopeSmall()}
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-20)))

	settleVehicleOnSlope(&v, g, dt, 180)

	// Gain some forward speed before the steering windows begin.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(5, 10)}
		v.Step(dt, g)
	}

	lastYaw := v.Yaw
	lastX := v.Position.X
	weakWindows := 0
	windowSize := 30

	for i := 0; i < 180; i++ {
		v.Input = VehicleInput{
			Throttle: fixed.FromFraction(3, 5),
			Steer:    fixed.One,
		}
		v.Step(dt, g)

		if (i+1)%windowSize == 0 {
			yawDelta := v.Yaw.Sub(lastYaw)
			xDelta := v.Position.X.Sub(lastX)

			t.Logf("window %d: yawDelta=%v xDelta=%v groundedWheels=%d onGround=%v",
				(i+1)/windowSize, yawDelta, xDelta, v.GroundedWheels, v.OnGround)

			// We check for any meaningful yaw response (absolute delta) rather than
			// assuming the vehicle can easily turn "right" against the slope's
			// specific triangle normal layout.
			if yawDelta.Abs().Cmp(fixed.FromFraction(5, 1000)) <= 0 { // 0.005 rad per window
				weakWindows++
			}

			lastYaw = v.Yaw
			lastX = v.Position.X
		}
	}

	if weakWindows > 2 {
		t.Fatalf("steering response dropped out too often while climbing triangle slope: weakWindows=%d expected<=2", weakWindows)
	}
}
