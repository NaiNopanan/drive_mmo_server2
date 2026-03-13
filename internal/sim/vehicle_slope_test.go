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
		MinZ:  fixed.FromInt(-100), MaxZ: fixed.FromInt(100),
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
