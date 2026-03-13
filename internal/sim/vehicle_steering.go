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
	target := v.Input.Steer.Mul(v.Tuning.MaxSteerAngleRad)
	maxDelta := v.Tuning.SteerRateRadPerSec.Mul(dt)

	for i := range v.Wheels {
		if v.WheelDefs[i].IsFront {
			v.Wheels[i].SteerAngleRad = moveTowards(v.Wheels[i].SteerAngleRad, target, maxDelta)
		} else {
			v.Wheels[i].SteerAngleRad = fixed.Zero
		}
	}
}
