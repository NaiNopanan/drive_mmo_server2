package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func moveTowards(current, target, maxDelta fixed.Fixed) fixed.Fixed {
	delta := target.Sub(current)
	if delta.Abs().Cmp(maxDelta) <= 0 {
		return target
	}
	if delta.Cmp(fixed.Zero) > 0 {
		return current.Add(maxDelta)
	}
	return current.Sub(maxDelta)
}

func (v *Vehicle) UpdateSteering(dt fixed.Fixed) {
	speed := geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length()
	// Speed-sensitive steering: reduce max angle at high speeds
	// factor = 1 / (1 + speed/28)
	// At 28m/s (~101km/h), steer is halved, but a floor remains so city driving
	// and slopes don't suddenly feel "locked out".
	steerFactor := fixed.One.Div(fixed.One.Add(speed.Div(fixed.FromInt(28))))
	minSteerFactor := fixed.FromFraction(45, 100)
	if steerFactor.Cmp(minSteerFactor) < 0 {
		steerFactor = minSteerFactor
	}
	if speed.Cmp(fixed.FromInt(6)) < 0 {
		lowSpeedBoost := fixed.One.Add(fixed.FromInt(6).Sub(speed).Div(fixed.FromInt(12)))
		steerFactor = steerFactor.Mul(lowSpeedBoost)
		maxSteerFactor := fixed.FromFraction(6, 5) // 1.2x cap
		if steerFactor.Cmp(maxSteerFactor) > 0 {
			steerFactor = maxSteerFactor
		}
	}

	target := v.Input.Steer.Mul(v.Tuning.MaxSteerAngleRad).Mul(steerFactor)
	maxDelta := v.Tuning.SteerRateRadPerSec.Mul(dt)

	for i := range v.Wheels {
		if v.WheelDefs[i].IsFront {
			v.Wheels[i].SteerAngleRad = moveTowards(v.Wheels[i].SteerAngleRad, target, maxDelta)
		} else {
			v.Wheels[i].SteerAngleRad = fixed.Zero
		}
	}
}
