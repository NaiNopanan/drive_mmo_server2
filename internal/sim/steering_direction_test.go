package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// TestVehicleSteeringLeftChangesYawNegative verifies that steering Left
// (Steer < 0) results in negative Yaw and negative X movement.
func TestVehicleSteeringLeftChangesYawNegative(t *testing.T) {
	v := NewAWDVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-100), MaxX: fixed.FromInt(100),
		MinZ: fixed.FromInt(-100), MaxZ: fixed.FromInt(100),
	}
	dt := fixed.FromFraction(1, 60)

	// 1) Settle on ground
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// 2) Drive forward + Steer Left (Full Left) for 3 seconds
	for i := 0; i < 180; i++ {
		v.Input = VehicleInput{
			Throttle: fixed.FromFraction(1, 2), // Half throttle for stability
			Steer:    fixed.One.Neg(),          // Full Left
		}
		v.Step(dt, g)
	}

	t.Logf("Final Yaw: %v", v.Yaw)
	t.Logf("Final Pos.X: %v", v.Position.X)

	// Steering Left (negative steer) should rotate the vehicle such that Yaw decreases (becomes negative)
	// and the vehicle moves toward negative X.
	if v.Yaw.Cmp(fixed.Zero) >= 0 {
		t.Errorf("expected negative yaw when steering left, got %v", v.Yaw)
	}
	if v.Position.X.Cmp(fixed.Zero) >= 0 {
		t.Errorf("expected negative X when steering left, got %v", v.Position.X)
	}
}

// TestVehicleSteeringRightChangesYawPositive verifies that steering Right
// (Steer > 0) results in positive Yaw and positive X movement.
func TestVehicleSteeringRightChangesYawPositive(t *testing.T) {
	v := NewAWDVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))
	g := FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-100), MaxX: fixed.FromInt(100),
		MinZ: fixed.FromInt(-100), MaxZ: fixed.FromInt(100),
	}
	dt := fixed.FromFraction(1, 60)

	// 1) Settle on ground
	for i := 0; i < 120; i++ {
		v.Step(dt, g)
	}

	// 2) Drive forward + Steer Right (Full Right) for 3 seconds
	for i := 0; i < 180; i++ {
		v.Input = VehicleInput{
			Throttle: fixed.FromFraction(1, 2),
			Steer:    fixed.One, // Full Right
		}
		v.Step(dt, g)
	}

	t.Logf("Final Yaw: %v", v.Yaw)
	t.Logf("Final Pos.X: %v", v.Position.X)

	if v.Yaw.Cmp(fixed.Zero) <= 0 {
		t.Errorf("expected positive yaw when steering right, got %v", v.Yaw)
	}
	if v.Position.X.Cmp(fixed.Zero) <= 0 {
		t.Errorf("expected positive X when steering right, got %v", v.Position.X)
	}
}
