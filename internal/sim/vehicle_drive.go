package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) pointVelocityAt(p geom.Vec3) geom.Vec3 {
	r := p.Sub(v.Position)
	omega := v.UpWS.Scale(v.YawVelocity)
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

// blendedTractionNormal returns a shared plane normal for traction basis.
// When grounded with 2+ wheels, it blends the per-wheel contact normal toward
// the chassis UpWS (80/20) so all wheels share a stable, consistent basis.
// Without this, each wheel on a tessellated slope gets a slightly different
// triangle normal, causing lateral/drive forces to fight each other every frame.
func (v *Vehicle) blendedTractionNormal(contactNormal geom.Vec3) geom.Vec3 {
	planeNormal := contactNormal

	if v.OnGround && v.GroundedWheels >= 2 {
		blended := v.safeNormalize(
			v.UpWS.Scale(fixed.FromFraction(4, 5)).
				Add(contactNormal.Scale(fixed.FromFraction(1, 5))),
		)
		if blended.LengthSq().Cmp(fixed.Zero) != 0 {
			planeNormal = blended
		}
	}

	if planeNormal.LengthSq().Cmp(fixed.Zero) == 0 {
		planeNormal = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
	}
	return planeNormal.Normalize()
}

// stableWheelBasis returns (forward, right) wheel vectors projected onto
// the blended traction plane rather than the raw per-wheel contact normal.
func (v *Vehicle) stableWheelBasis(w *WheelState) (geom.Vec3, geom.Vec3) {
	planeNormal := v.blendedTractionNormal(w.ContactNormal)

	bodyFwd := v.safeNormalize(v.projectOntoPlane(v.ForwardWS, planeNormal))
	if bodyFwd.LengthSq().Cmp(fixed.Zero) == 0 {
		bodyFwd = v.ForwardWS
	}

	bodyRight := planeNormal.Cross(bodyFwd)
	if bodyRight.LengthSq().Cmp(fixed.Zero) == 0 {
		bodyRight = v.safeNormalize(v.projectOntoPlane(v.RightWS, planeNormal))
	} else {
		bodyRight = bodyRight.Normalize()
	}

	yawSin := fixed.Sin(w.SteerAngleRad)
	yawCos := fixed.Cos(w.SteerAngleRad)

	rawFwd := bodyFwd.Scale(yawCos).Add(bodyRight.Scale(yawSin))
	fwd := v.safeNormalize(v.projectOntoPlane(rawFwd, planeNormal))
	if fwd.LengthSq().Cmp(fixed.Zero) == 0 {
		fwd = bodyFwd
	}

	right := planeNormal.Cross(fwd)
	if right.LengthSq().Cmp(fixed.Zero) == 0 {
		right = bodyRight
	} else {
		right = right.Normalize()
	}

	return fwd, right
}

func (v *Vehicle) computeWheelMotionForces(i int) geom.Vec3 {
	w := &v.Wheels[i]
	if !w.InContact {
		return geom.Zero()
	}

	fwd, right := v.stableWheelBasis(w)
	w.WheelForwardWS = fwd
	w.WheelRightWS = right

	pv := v.pointVelocityAt(w.ContactPoint)
	w.LongSpeed = pv.Dot(fwd)
	w.LatSpeed = pv.Dot(right)

	// Drive force
	w.DriveForce = v.Input.Throttle.Mul(v.Tuning.DriveForce)

	// Brake force
	w.BrakeForce = fixed.Zero
	if w.LongSpeed.Abs().Cmp(fixed.FromFraction(1, 100)) > 0 && v.Input.Brake.Cmp(fixed.Zero) > 0 {
		brakeDir := fixed.One
		if w.LongSpeed.Cmp(fixed.Zero) < 0 {
			brakeDir = fixed.One.Neg()
		}
		w.BrakeForce = brakeDir.Mul(v.Input.Brake).Mul(v.Tuning.BrakeForce).Neg()
	}

	// Lateral force target with slip ramp
	targetLatForce := fixed.Zero
	slipThreshold := fixed.FromFraction(1, 10) // 0.1 m/s

	if w.LatSpeed.Abs().Cmp(fixed.FromFraction(1, 1000)) > 0 {
		potForce := w.LatSpeed.Mul(v.Tuning.LateralGrip).Neg()
		ramp := w.LatSpeed.Abs().Div(slipThreshold)
		if ramp.Cmp(fixed.One) > 0 {
			ramp = fixed.One
		}
		targetLatForce = potForce.Mul(ramp)

		maxLat := w.SuspensionForce
		if targetLatForce.Abs().Cmp(maxLat) > 0 {
			if targetLatForce.Cmp(fixed.Zero) > 0 {
				targetLatForce = maxLat
			} else {
				targetLatForce = maxLat.Neg()
			}
		}
	}

	// Low-pass lateral response to reduce chatter on triangle slopes.
	// If the target and current force are in opposite directions, snap to zero
	// first to prevent wrong-sign carry-over from fighting steering input.
	if targetLatForce.Cmp(fixed.Zero) != 0 &&
		w.LateralForce.Cmp(fixed.Zero) != 0 &&
		(targetLatForce.Cmp(fixed.Zero) > 0) != (w.LateralForce.Cmp(fixed.Zero) > 0) {
		w.LateralForce = fixed.Zero
	}
	// 60% old + 40% new: enough smoothing for slope chatter, responsive enough for steering.
	w.LateralForce = w.LateralForce.Mul(fixed.FromFraction(6, 10)).
		Add(targetLatForce.Mul(fixed.FromFraction(4, 10)))

	if w.LateralForce.Abs().Cmp(fixed.FromFraction(1, 100)) < 0 {
		w.LateralForce = fixed.Zero
	}

	totalLong := w.DriveForce.Add(w.BrakeForce)

	// Rolling resistance
	w.RollingForce = fixed.Zero
	if w.LongSpeed.Abs().Cmp(fixed.Zero) != 0 {
		resDir := fixed.One
		if w.LongSpeed.Cmp(fixed.Zero) < 0 {
			resDir = fixed.One.Neg()
		}
		w.RollingForce = resDir.Mul(v.Tuning.RollingResistance).Neg()
		totalLong = totalLong.Add(w.RollingForce)
	}

	// Soft low-speed hold to prevent micro-creep / sink-feel on slopes when
	// neither throttle nor brake is applied (parked or idle on incline).
	steerActive := v.Input.Steer.Abs().Cmp(fixed.FromFraction(2, 100)) >= 0
	if v.Input.Throttle.Abs().Cmp(fixed.FromFraction(1, 100)) < 0 &&
		v.Input.Brake.Abs().Cmp(fixed.FromFraction(1, 100)) < 0 &&
		!steerActive &&
		w.LongSpeed.Abs().Cmp(fixed.FromFraction(3, 10)) < 0 {
		holdForce := w.LongSpeed.Mul(v.Tuning.BrakeForce.Div(fixed.FromInt(8))).Neg()
		totalLong = totalLong.Add(holdForce)
	}

	// Use the same smoothed plane normal for suspension as traction so
	// tessellated ramps do not kick the chassis upward on every triangle seam.
	suspensionNormal := v.blendedTractionNormal(w.ContactNormal)
	f := suspensionNormal.Scale(w.SuspensionForce).
		Add(fwd.Scale(totalLong)).
		Add(right.Scale(w.LateralForce))

	return f
}
