//go:build driveability_lenient

package sim

import (
	"testing"

	"server2/internal/fixed"
)

func lenientDriveProfile() driveProfile {
	return driveProfile{
		Name: "lenient",

		ForwardMinProgressZ: fixed.FromInt(6),
		ForwardMaxYawAbs:    fixed.FromFraction(5, 100),
		ForwardGroundedMin:  fixed.FromFraction(90, 100),

		SteerMinYawAbs:      fixed.FromFraction(6, 100),
		SteerMinArcXAbs:     fixed.FromFraction(1, 2),
		SteerGroundedMin:    fixed.FromFraction(90, 100),
		SteerReleaseKeepMin: fixed.FromFraction(5, 100),

		ReverseMinForwardSpeed: fixed.FromInt(-1),
		ReverseMinProgressZ:    fixed.FromInt(-3),
		ReverseMaxYawAbs:       fixed.FromFraction(8, 100),
		ReverseGroundedMin:     fixed.FromFraction(90, 100),

		UphillMinProgressZ: fixed.FromInt(5),
		UphillMinGainY:     fixed.FromFraction(4, 10),
		UphillGroundedMin:  fixed.FromFraction(85, 100),
		UphillMaxAbsVelY:   fixed.FromInt(12), // Higher to allow high-speed uphill motion (100m/s * 0.1)

		DownhillMinProgressZ: fixed.FromInt(-1),
		DownhillGroundedMin:  fixed.FromFraction(85, 100),
		DownhillMaxAbsVelY:   fixed.FromInt(2),

		SlopeSteerMinProgressZ: fixed.FromInt(3),
		SlopeSteerMinArcXAbs:   fixed.FromInt(1),
		SlopeSteerMinGainY:     fixed.FromFraction(3, 10),
		SlopeSteerGroundedMin:  fixed.FromFraction(80, 100),
		SlopeSteerMinMaxYawVel: fixed.FromFraction(3, 100),
	}
}

func TestDriveabilityLenient_ForwardSmoothFlat(t *testing.T) {
	runDriveabilityForwardSmoothFlat(t, lenientDriveProfile())
}

func TestDriveabilityLenient_SteeringSmoothFlat(t *testing.T) {
	runDriveabilitySteeringSmoothFlat(t, lenientDriveProfile())
}

func TestDriveabilityLenient_ReverseSmoothFlat(t *testing.T) {
	runDriveabilityReverseSmoothFlat(t, lenientDriveProfile())
}

func TestDriveabilityLenient_UphillSmooth(t *testing.T) {
	runDriveabilityUphillSmooth(t, lenientDriveProfile())
}

func TestDriveabilityLenient_DownhillSmooth(t *testing.T) {
	runDriveabilityDownhillSmooth(t, lenientDriveProfile())
}

func TestDriveabilityLenient_SteerOnSlopeSmooth(t *testing.T) {
	runDriveabilitySteerOnSlopeSmooth(t, lenientDriveProfile())
}
