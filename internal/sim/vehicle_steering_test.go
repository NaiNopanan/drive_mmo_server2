package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func TestUpdateSteering_UsesHorizontalSpeedOnly(t *testing.T) {
	v := NewVehicle(1, geom.Zero())
	v.Tuning.MaxSteerAngleRad = fixed.FromFraction(72, 100)
	v.Input.Steer = fixed.One
	v.Velocity = geom.V3(fixed.Zero, fixed.FromInt(20), fixed.Zero)

	v.UpdateSteering(fixed.FromFraction(1, 60))

	if v.Wheels[0].SteerAngleRad == fixed.Zero {
		t.Fatalf("expected steering to respond even when only vertical velocity is present")
	}
}

func TestLowSpeedSteeringAssist_IncreasesSteerAngleAtLowSpeed(t *testing.T) {
	v := NewVehicle(1, geom.Zero())
	v.Tuning.MaxSteerAngleRad = fixed.FromFraction(72, 100)
	v.Tuning.SteerRateRadPerSec = fixed.FromInt(100)
	v.Input.Steer = fixed.One

	v.Velocity = geom.V3(fixed.Zero, fixed.Zero, fixed.FromInt(2))
	v.UpdateSteering(fixed.FromFraction(1, 60))
	lowSpeedAngle := v.Wheels[0].SteerAngleRad

	for i := range v.Wheels {
		v.Wheels[i].SteerAngleRad = fixed.Zero
	}

	v.Velocity = geom.V3(fixed.Zero, fixed.Zero, fixed.FromInt(20))
	v.UpdateSteering(fixed.FromFraction(1, 60))
	highSpeedAngle := v.Wheels[0].SteerAngleRad

	if lowSpeedAngle.Cmp(highSpeedAngle) <= 0 {
		t.Fatalf("expected low-speed steer assist to produce larger angle: low=%v high=%v", lowSpeedAngle, highSpeedAngle)
	}
}
