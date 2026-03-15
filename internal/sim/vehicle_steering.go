package sim

import "server2/internal/fixed"

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
	speed := v.Velocity.Length()
	// Speed-sensitive steering: reduce max angle at high speeds
	// factor = 1 / (1 + speed/20)
	// At 20m/s (72km/h), steer is halved. At 100m/s, it's ~1/6.
	steerFactor := fixed.One.Div(fixed.One.Add(speed.Div(fixed.FromInt(20))))

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
