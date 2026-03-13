package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) pointVelocityAt(p geom.Vec3) geom.Vec3 {
	r := p.Sub(v.Position)
	// omega = (0, YawVelocity, 0)
	// cross(omega, r) = (YawVelocity*r.Z, 0, -YawVelocity*r.X)
	angVel := geom.V3(v.YawVelocity.Mul(r.Z), fixed.Zero, v.YawVelocity.Mul(r.X).Neg())
	return v.Velocity.Add(angVel)
}

func (v *Vehicle) computeWheelMotionForces(i int) geom.Vec3 {
	w := &v.Wheels[i]
	if !w.InContact {
		return geom.Zero()
	}

	// basis for this wheel
	steerYaw := v.Yaw
	if v.WheelDefs[i].IsFront {
		steerYaw = steerYaw.Add(w.SteerAngleRad)
	}
	fwd, right := HeadingFromYaw(steerYaw)

	// Project fwd/right onto ground plane to avoid "lift" force on slopes
	// projected = v - (v dot n) * n
	fwd = fwd.Sub(w.ContactNormal.Scale(fwd.Dot(w.ContactNormal)))
	right = right.Sub(w.ContactNormal.Scale(right.Dot(w.ContactNormal)))

	// Re-normalize to keep unit vectors
	if fwd.LengthSq().Cmp(fixed.Zero) > 0 {
		fwd = fwd.Normalize()
	}
	if right.LengthSq().Cmp(fixed.Zero) > 0 {
		right = right.Normalize()
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

	// Lateral Force - only apply if above threshold to avoid clamp saturation at rest
	latThreshold := fixed.FromFraction(1, 100) // 0.01 m/s dead zone
	if w.LatSpeed.Abs().Cmp(latThreshold) > 0 {
		w.LateralForce = w.LatSpeed.Mul(v.Tuning.LateralGrip).Neg()
		// Clamp lat force by load (simplified friction circle)
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
