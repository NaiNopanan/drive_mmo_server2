package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func settleVehicleOnFlat(v *Vehicle, ticks int) {
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	v.Input = VehicleInput{}
	for i := 0; i < ticks; i++ {
		v.Step(dt, g)
	}
}

func planarSpeed(v *Vehicle) fixed.Fixed {
	return geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length()
}

func TestVehicle_SpeedDecreasesAfterReleasingControls(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle on ground first.
	settleVehicleOnFlat(&v, 120)

	// Phase 1: build forward speed.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	// Phase 2: build combined forward + steering state.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)
	}

	speedBeforeRelease := planarSpeed(&v)
	yawBeforeRelease := v.YawVelocity.Abs()

	// Phase 3: release all input and observe decay.
	v.Input = VehicleInput{}
	for i := 0; i < 600; i++ { // 10 seconds
		v.Step(dt, g)
	}

	speedAfterRelease := planarSpeed(&v)
	yawAfterRelease := v.YawVelocity.Abs()

	t.Logf("Speed before release: %v m/s", speedBeforeRelease)
	t.Logf("Speed after release:  %v m/s", speedAfterRelease)
	t.Logf("Yaw before release:   %v rad/s", yawBeforeRelease)
	t.Logf("Yaw after release:    %v rad/s", yawAfterRelease)
	t.Logf("Final velocity: X=%v Z=%v", v.Velocity.X, v.Velocity.Z)

	// Must lose at least 1 m/s due to rolling resistance (100N on 800kg for 10s = 1.25m/s loss)
	maxAllowedAfter := speedBeforeRelease.Sub(fixed.One)

	if speedAfterRelease.Cmp(speedBeforeRelease) >= 0 {
		t.Fatalf("vehicle did not slow down after releasing controls: before=%v after=%v",
			speedBeforeRelease, speedAfterRelease)
	}

	if speedAfterRelease.Cmp(maxAllowedAfter) > 0 {
		t.Fatalf("vehicle did not slow down enough after releasing controls: before=%v after=%v allowed<=%v",
			speedBeforeRelease, speedAfterRelease, maxAllowedAfter)
	}
}

func TestVehicle_YawDecaysAfterRelease(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle first.
	settleVehicleOnFlat(&v, 120)

	// Build some forward speed so yaw response is meaningful.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One}
		v.Step(dt, g)
	}

	// Build yaw motion.
	for i := 0; i < 60; i++ {
		v.Input = VehicleInput{Throttle: fixed.One, Steer: fixed.One}
		v.Step(dt, g)
	}

	yawBeforeRelease := v.YawVelocity.Abs()
	if yawBeforeRelease.Cmp(fixed.FromFraction(5, 100)) <= 0 {
		t.Fatalf("test setup did not build enough yaw velocity before release: got=%v", yawBeforeRelease)
	}

	// Release and wait.
	v.Input = VehicleInput{}
	for i := 0; i < 300; i++ { // 5 seconds
		v.Step(dt, g)
	}

	yawAfterRelease := v.YawVelocity.Abs()
	t.Logf("Yaw before release: %v rad/s", yawBeforeRelease)
	t.Logf("Yaw after release:  %v rad/s", yawAfterRelease)

	// Must clearly decay relative to the pre-release yaw.
	maxAllowedRelative := yawBeforeRelease.Mul(fixed.FromFraction(1, 3)) // at least 66% reduction
	maxAllowedAbsolute := fixed.FromFraction(8, 100)                     // 0.08 rad/s

	if yawAfterRelease.Cmp(yawBeforeRelease) >= 0 {
		t.Fatalf("yaw velocity did not decay after releasing steer: before=%v after=%v",
			yawBeforeRelease, yawAfterRelease)
	}

	if yawAfterRelease.Cmp(maxAllowedRelative) > 0 && yawAfterRelease.Cmp(maxAllowedAbsolute) > 0 {
		t.Fatalf("yaw velocity did not decay enough: before=%v after=%v relativeMax=%v absoluteMax=%v",
			yawBeforeRelease, yawAfterRelease, maxAllowedRelative, maxAllowedAbsolute)
	}
}

func TestVehicle_LatForceIsZeroWhenStationary(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Let the vehicle settle with no input.
	for i := 0; i < 300; i++ {
		v.Step(dt, g)
	}

	maxLatF := fixed.FromFraction(1, 10) // 0.1 N
	for i, w := range v.Wheels {
		if w.LateralForce.Abs().Cmp(maxLatF) > 0 {
			t.Fatalf("wheel %d lateral force not near zero when stationary: got=%v max=%v",
				i, w.LateralForce, maxLatF)
		}
	}
}
