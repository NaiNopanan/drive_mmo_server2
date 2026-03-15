//go:build driveability_strict

package sim

import (
	"testing"

	"server2/internal/fixed"
)

func strictDriveProfile() driveProfile {
	return driveProfile{
		Name: "strict",

		ForwardMinProgressZ: fixed.FromInt(10),
		ForwardMaxYawAbs:    fixed.FromFraction(2, 100),
		ForwardGroundedMin:  fixed.FromFraction(97, 100),

		SteerMinYawAbs:      fixed.FromFraction(12, 100),
		SteerMinArcXAbs:     fixed.FromFraction(3, 2), // 1.5
		SteerGroundedMin:    fixed.FromFraction(95, 100),
		SteerReleaseKeepMin: fixed.FromFraction(2, 10), // 20%

		ReverseMinForwardSpeed: fixed.FromInt(-2),
		ReverseMinProgressZ:    fixed.FromInt(-5),
		ReverseMaxYawAbs:       fixed.FromFraction(5, 100),
		ReverseGroundedMin:     fixed.FromFraction(95, 100),

		UphillMinProgressZ: fixed.FromInt(8),
		UphillMinGainY:     fixed.FromFraction(7, 10),
		UphillGroundedMin:  fixed.FromFraction(90, 100),
		UphillMaxAbsVelY:   fixed.FromInt(12),

		DownhillMinProgressZ: fixed.FromInt(-2),
		DownhillGroundedMin:  fixed.FromFraction(90, 100),
		DownhillMaxAbsVelY:   fixed.FromFraction(15, 10), // 1.5

		SlopeSteerMinProgressZ: fixed.FromInt(5),
		SlopeSteerMinArcXAbs:   fixed.FromInt(2),
		SlopeSteerMinGainY:     fixed.FromFraction(5, 10),
		SlopeSteerGroundedMin:  fixed.FromFraction(85, 100),
		SlopeSteerMinMaxYawVel: fixed.FromFraction(5, 100),
	}
}

func TestDriveabilityStrict_ForwardSmoothFlat(t *testing.T) {
	runDriveabilityForwardSmoothFlat(t, strictDriveProfile())
}

func TestDriveabilityStrict_SteeringSmoothFlat(t *testing.T) {
	runDriveabilitySteeringSmoothFlat(t, strictDriveProfile())
}

func TestDriveabilityStrict_ReverseSmoothFlat(t *testing.T) {
	runDriveabilityReverseSmoothFlat(t, strictDriveProfile())
}

func TestDriveabilityStrict_UphillSmooth(t *testing.T) {
	runDriveabilityUphillSmooth(t, strictDriveProfile())
}

func TestDriveabilityStrict_DownhillSmooth(t *testing.T) {
	runDriveabilityDownhillSmooth(t, strictDriveProfile())
}

func TestDriveabilityStrict_SteerOnSlopeSmooth(t *testing.T) {
	runDriveabilitySteerOnSlopeSmooth(t, strictDriveProfile())
}
