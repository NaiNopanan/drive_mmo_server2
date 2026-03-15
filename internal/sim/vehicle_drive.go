package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) pointVelocityAt(p geom.Vec3) geom.Vec3 {
	r := p.Sub(v.Position)
	// Omega = YawVelocity * UpWS
	omega := v.UpWS.Scale(v.YawVelocity)
	// velocity = v + omega x r
	angVel := omega.Cross(r)
	return v.Velocity.Add(angVel)
}

func (v *Vehicle) projectOntoPlane(vec, normal geom.Vec3) geom.Vec3 {
	return vec.Sub(normal.Scale(vec.Dot(normal)))
}

func (v *Vehicle) safeNormalize(vec geom.Vec3) geom.Vec3 {
	ls := vec.LengthSq()
	if ls.Cmp(fixed.Zero) == 0 {
		return geom.Zero()
	}
	return vec.Normalize()
}

func (v *Vehicle) computeWheelMotionForces(i int) geom.Vec3 {
	w := &v.Wheels[i]
	if !w.InContact {
		return geom.Zero()
	}

	// 1) Calculate raw heading based on body and steering angle
	// We now rotate the body's horizontal forward/right around the actual ground NORMAL
	// to ensure steering stays "on the plane" even when heavily banked.
	yawSin := fixed.Sin(w.SteerAngleRad)
	yawCos := fixed.Cos(w.SteerAngleRad)

	// Local basis for steering on this specific ground normal
	bodyFwd := v.ForwardWS
	bodyRight := v.RightWS

	// rawFwd/rawRight are now tilted to the surface
	// steerFwd = cos(angle)*bodyFwd + sin(angle)*bodyRight
	rawFwd := bodyFwd.Scale(yawCos).Add(bodyRight.Scale(yawSin))
	// steerRight = cos(angle)*bodyRight - sin(angle)*bodyFwd
	rawRight := bodyRight.Scale(yawCos).Sub(bodyFwd.Scale(yawSin))

	// Ensure they are strictly projected and normalized on the ground plane
	fwd := v.safeNormalize(v.projectOntoPlane(rawFwd, w.ContactNormal))
	right := v.safeNormalize(v.projectOntoPlane(rawRight, w.ContactNormal))

	// Fallback if projection fails (e.g. edge case)
	if fwd.LengthSq().Cmp(fixed.Zero) == 0 {
		fwd = rawFwd
	}
	if right.LengthSq().Cmp(fixed.Zero) == 0 {
		right = rawRight
	}

	w.WheelForwardWS = fwd
	w.WheelRightWS = right

	pv := v.pointVelocityAt(w.ContactPoint)
	w.LongSpeed = pv.Dot(fwd)
	w.LatSpeed = pv.Dot(right)

	// Drive Force
	if v.WheelDefs[i].IsDriven {
		w.DriveForce = v.Input.Throttle.Mul(v.Tuning.DriveForce)
	}

	// Brake Force
	if w.LongSpeed.Abs().Cmp(fixed.FromFraction(1, 100)) > 0 {
		brakeDir := fixed.One
		if w.LongSpeed.Cmp(fixed.Zero) < 0 {
			brakeDir = fixed.One.Neg()
		}
		w.BrakeForce = brakeDir.Mul(v.Input.Brake).Mul(v.Tuning.BrakeForce).Neg()
	}

	// Lateral Force - smoothed to avoid high-frequency vibration
	// We use a small slip threshold (e.g. 0.1 m/s) to ramp up the force linearly
	// instead of snapping to full grip immediately.
	slipThreshold := fixed.FromFraction(1, 10) // 0.1 m/s
	if w.LatSpeed.Abs().Cmp(fixed.FromFraction(1, 1000)) > 0 {
		// Calculate potential force
		potForce := w.LatSpeed.Mul(v.Tuning.LateralGrip).Neg()

		// Ramp factor: min(1, |LatSpeed| / slipThreshold)
		ramp := w.LatSpeed.Abs().Div(slipThreshold)
		if ramp.Cmp(fixed.One) > 0 {
			ramp = fixed.One
		}
		w.LateralForce = potForce.Mul(ramp)

		// Clamp by load
		maxLat := w.SuspensionForce
		if w.LateralForce.Abs().Cmp(maxLat) > 0 {
			if w.LateralForce.Cmp(fixed.Zero) > 0 {
				w.LateralForce = maxLat
			} else {
				w.LateralForce = maxLat.Neg()
			}
		}
	} else {
		w.LateralForce = fixed.Zero
	}

	totalLong := w.DriveForce.Add(w.BrakeForce)
	// Add rolling resistance
	if w.LongSpeed.Abs().Cmp(fixed.Zero) != 0 {
		resDir := fixed.One
		if w.LongSpeed.Cmp(fixed.Zero) < 0 {
			resDir = fixed.One.Neg()
		}
		
		totalLong = totalLong.Sub(resDir.Mul(v.Tuning.RollingResistance))
	}

	// Sum world forces
	f := w.ContactNormal.Scale(w.SuspensionForce).
		Add(fwd.Scale(totalLong)).
		Add(right.Scale(w.LateralForce))

	return f
}
