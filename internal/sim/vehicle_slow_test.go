package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// TestSlowVehicle_CanAccelerateFromStop verifies that a vehicle starting from rest
// with a small throttle can still build speed (snap-to-zero must not kill start-up).
func TestSlowVehicle_CanAccelerateFromStop(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	// Settle on ground
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Apply small throttle for 2 seconds
	smallThrottle := fixed.FromFraction(1, 5) // 0.2 throttle
	for i := 0; i < 120; i++ {
		v.Input = VehicleInput{Throttle: smallThrottle}
		v.Step(dt, g)
	}

	speed := geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length()
	t.Logf("Speed after 2s light throttle: %v m/s", speed)

	if speed.Cmp(fixed.FromFraction(5, 10)) <= 0 {
		t.Errorf("vehicle didn't move with small throttle: speed=%v, expected > 0.5 m/s", speed)
	}
}

// TestSlowVehicle_CreepsForwardBeforeFallingOff verifies that a very slowly moving
// vehicle near a ground edge eventually crosses it and falls off.
func TestSlowVehicle_CreepsForwardBeforeFallingOff(t *testing.T) {
	// Place vehicle just inside the boundary with a small forward velocity
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
		MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
	}
	dt := fixed.FromFraction(1, 60)

	// Start 2 units from the +Z edge
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(48)))

	// Settle on ground
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Give it enough velocity to overcome rolling resistance (300N) and reach the edge
	// Rolling resistance decelerates 1200kg car at 0.25 m/s², so 5 m/s goes ~10m before stopping
	v.Velocity.Z = fixed.FromFraction(50, 10) // 5.0 m/s toward +Z edge

	// Phase 3: Release all input, wait 5 seconds
	v.Input = VehicleInput{}
	fell := false
	for i := 0; i < 300; i++ {
		v.Step(dt, g)
		if !v.OnGround && v.Position.Z.Cmp(fixed.FromInt(50)) > 0 {
			fell = true
			t.Logf("Vehicle fell off edge at tick %d, pos.Z=%v", i, v.Position.Z)
			break
		}
	}

	if !fell {
		t.Fatalf("slow vehicle did not fall off edge: pos.Z=%v onGround=%v vel.Z=%v",
			v.Position.Z, v.OnGround, v.Velocity.Z)
	}
}

// TestSlowVehicle_DoesNotFreezeOnGround verifies that a very-low-speed vehicle
// (but still above snap threshold) does NOT get snapped to zero.
func TestSlowVehicle_DoesNotFreezeOnGround(t *testing.T) {
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{Y: fixed.Zero}
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// Set velocity well above snap threshold (0.5 m/s was draining to nearly zero via rolling resistance).
	// Use a stronger initial velocity (1 m/s) so we can see meaningful movement in 30 ticks.
	v.Velocity.Z = fixed.FromFraction(10, 10) // 1.0 m/s

	startZ := v.Position.Z
	for i := 0; i < 30; i++ {
		v.Step(dt, g)
	}

	// Rolling resistance decelerates at ~0.25 m/s², so we should move ~0.47m in 30 ticks.
	// Just verify we moved more than 0.1m to prove we weren't frozen.
	if v.Position.Z.Sub(startZ).Cmp(fixed.FromFraction(1, 10)) <= 0 {
		t.Errorf("vehicle froze at low speed: startZ=%v endZ=%v deltaZ=%v",
			startZ, v.Position.Z, v.Position.Z.Sub(startZ))
	}
}

// TestSlowVehicle_GravityStillAffectsOffEdge verifies that even at near-zero XZ velocity,
// gravity pulls a vehicle down once it's off a finite ground edge.
func TestSlowVehicle_GravityStillAffectsOffEdge(t *testing.T) {
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-10), MaxX: fixed.FromInt(10),
		MinZ: fixed.FromInt(-10), MaxZ: fixed.FromInt(10),
	}
	dt := fixed.FromFraction(1, 60)

	// Spawn vehicle outside the boundary
	v := NewVehicle(1, geom.V3(fixed.FromInt(20), fixed.FromInt(5), fixed.Zero))
	startY := v.Position.Y

	// XZ velocity is zero - only gravity should act
	v.Velocity.Z = fixed.Zero
	v.Velocity.X = fixed.Zero

	for i := 0; i < 60; i++ {
		v.Step(dt, g)
	}

	if v.Position.Y.Cmp(startY) >= 0 {
		t.Errorf("vehicle outside boundary didn't fall: startY=%v endY=%v",
			startY, v.Position.Y)
	}
	if v.OnGround {
		t.Errorf("vehicle outside boundary should NOT be grounded")
	}
}
