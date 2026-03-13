package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// TestVehicle_StopsAfterReleasingControls is the core regression test.
// Drives forward 1 second, steers 1 second, then releases all input.
// After 3 more seconds the vehicle's speed should be very low (< 0.5 m/s).
func TestVehicle_StopsAfterReleasingControls(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle on ground
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Phase 1: Drive forward for 1 second
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	// Phase 2: Steer while driving for 1 second
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)
	}

	// Phase 3: Release all input, wait 3 seconds
	v.Input = VehicleInput{}
	for i := 0; i < 180; i++ {
		v.Step(dt, g)
	}

	// Check that the vehicle is nearly stopped
	speed := geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length()
	maxAllowed := fixed.FromFraction(5, 10) // 0.5 m/s

	t.Logf("Final speed after release: %v m/s", speed)
	t.Logf("Final YawVelocity: %v rad/s", v.YawVelocity)
	t.Logf("Final Velocity: X=%v Z=%v", v.Velocity.X, v.Velocity.Z)

	if speed.Cmp(maxAllowed) > 0 {
		t.Errorf("vehicle speed did not settle to near-zero after releasing controls: got=%v expected<=%v", speed, maxAllowed)
	}
}

// TestVehicle_YawDecaysAfterRelease verifies yaw rotation settles after releasing steer.
func TestVehicle_YawDecaysAfterRelease(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Drive forward for 1 second (build forward speed to make yaw meaningful)
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	// Drive + steer for 1 second
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)
	}

	// Release and wait 5 seconds
	v.Input = VehicleInput{}
	for i := 0; i < 300; i++ {
		v.Step(dt, g)
	}

	maxYaw := fixed.FromFraction(5, 100) // 0.05 rad/s
	t.Logf("Final YawVelocity: %v rad/s", v.YawVelocity)
	if v.YawVelocity.Abs().Cmp(maxYaw) > 0 {
		t.Errorf("yaw velocity did not decay: got=%v expected<=%v", v.YawVelocity, maxYaw)
	}
}

// TestVehicle_LatForceIsZeroWhenStationary verifies LatF = 0 when the vehicle
// is fully settled and not receiving any input.
func TestVehicle_LatForceIsZeroWhenStationary(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle for 5 seconds
	for i := 0; i < 300; i++ {
		v.Step(dt, g)
	}

	maxLatF := fixed.FromFraction(1, 10) // 0.1 N

	for i, w := range v.Wheels {
		if w.LateralForce.Abs().Cmp(maxLatF) > 0 {
			t.Errorf("wheel %d LateralForce not zero when stationary: got=%v", i, w.LateralForce)
		}
	}
}
