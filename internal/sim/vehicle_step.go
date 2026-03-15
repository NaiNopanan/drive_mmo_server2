package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

var (
	VehicleGravity = geom.V3(fixed.Zero, fixed.FromFraction(-98, 10), fixed.Zero)
)

func (v *Vehicle) clearAccumulators() {
	v.TotalForce = geom.Zero()
	v.TotalTorqueY = fixed.Zero
}

func (v *Vehicle) collectCurrentForces(dt fixed.Fixed, g GroundQuery) {
	v.clearAccumulators()

	for i := range v.Wheels {
		v.queryWheelGround(i, g)
		v.computeWheelSuspensionForce(i, dt)

		if !v.Wheels[i].InContact {
			continue
		}

		f := v.computeWheelMotionForces(i)
		v.TotalForce = v.TotalForce.Add(f)

		r := v.Wheels[i].ContactPoint.Sub(v.Position)
		torque := r.Cross(f)
		v.TotalTorqueY = v.TotalTorqueY.Add(torque.Dot(v.UpWS))
	}
}

func (v *Vehicle) refreshFinalContactState(dt fixed.Fixed, g GroundQuery) {
	for i := range v.Wheels {
		v.queryWheelGround(i, g)
		v.computeWheelSuspensionForce(i, dt)
	}
	v.finalizeSuspensionState()
}

// finalAverageGroundNormal computes a blended average normal from all grounded
// wheels at the final pose of the tick. Blending 80% toward the previous UpWS
// damps high-frequency basis chatter from per-frame triangle normal variation.
func (v *Vehicle) finalAverageGroundNormal() geom.Vec3 {
	rawAvg := geom.V3(fixed.Zero, fixed.One, fixed.Zero)

	if v.GroundedWheels > 0 {
		sumN := geom.Zero()
		count := 0
		for i := range v.Wheels {
			if v.Wheels[i].InContact {
				sumN = sumN.Add(v.Wheels[i].ContactNormal)
				count++
			}
		}
		if count > 0 {
			rawAvg = sumN.Scale(fixed.FromFraction(1, int64(count))).Normalize()
		}
	}

	blended := v.safeNormalize(
		v.UpWS.Scale(fixed.FromFraction(4, 5)).
			Add(rawAvg.Scale(fixed.FromFraction(1, 5))),
	)
	if blended.LengthSq().Cmp(fixed.Zero) == 0 {
		return rawAvg
	}
	return blended
}

func (v *Vehicle) Step(dt fixed.Fixed, g GroundQuery) {
	v.UpdateSteering(dt)

	startPose := VehiclePose{
		Position: v.Position,
		Yaw:      v.Yaw,
	}

	// 1) Collect forces from current stable basis/state.
	v.collectCurrentForces(dt, g)

	// 2) Predict end of tick.
	fullPos, fullVel, fullYaw, fullYawVel := v.predictFromCurrentState(dt, VehicleGravity)
	endPose := VehiclePose{
		Position: fullPos,
		Yaw:      fullYaw,
	}

	// 3) Sweep for earliest ground TOI.
	toi := v.FindGroundTOI(startPose, endPose, g)
	if !toi.Hit {
		v.Position = fullPos
		v.Velocity = fullVel
		v.Yaw = fullYaw
		v.YawVelocity = fullYawVel
	} else {
		// 4) Advance to hit time.
		hitDt := dt.Mul(toi.Time)
		v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(hitDt, VehicleGravity)

		// 5) Resolve impact velocity and nudge out of penetration.
		v.resolveGroundImpactVelocity(toi.Normal)
		v.applyCCDSeparationBias(toi.Normal, toi.Depth)

		// 6) Rebuild basis + recollect forces before integrating remaining time.
		remDt := dt.Sub(hitDt)
		if remDt.Cmp(fixed.Zero) > 0 {
			v.UpdateBasis(toi.Normal)
			v.collectCurrentForces(remDt, g)
			v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(remDt, VehicleGravity)
		}
	}

	// 7) Refresh actual grounded/contact state at the FINAL pose.
	v.refreshFinalContactState(dt, g)

	// 8) Blended average from actual final wheel contacts.
	avgNormal := v.finalAverageGroundNormal()

	// 9) One more inward-normal cleanup at the FINAL contact state.
	// This handles the case where the car is grounded but still carries a tiny
	// inward-normal velocity from CCD that causes sink / bounce chatter.
	if v.OnGround && v.GroundedWheels >= 2 {
		inwardNormalSpeed := v.Velocity.Dot(avgNormal)
		if inwardNormalSpeed.Cmp(fixed.Zero) < 0 {
			v.Velocity = v.Velocity.Sub(avgNormal.Scale(inwardNormalSpeed))
		}
	}

	// 10) Update basis once at end; post-step damping uses the newest slope basis.
	v.UpdateBasis(avgNormal)

	// 11) Damping after final basis/contact state is known.
	v.applyPostStepDamping()
}

// applyPostStepDamping is called once per full physics tick (not per sub-step).
func (v *Vehicle) applyPostStepDamping() {
	steerDeadzone := fixed.FromFraction(2, 100) // 0.02
	steerActive := v.Input.Steer.Abs().Cmp(steerDeadzone) >= 0

	if !steerActive {
		v.YawVelocity = v.YawVelocity.Mul(fixed.FromFraction(92, 100))
		if v.YawVelocity.Abs().Cmp(fixed.FromFraction(5, 1000)) < 0 {
			v.YawVelocity = fixed.Zero
		}
	}

	if v.OnGround {
		lateralSpeed := v.Velocity.Dot(v.RightWS)

		keep := fixed.FromFraction(88, 100)
		if steerActive {
			keep = fixed.FromFraction(92, 100)
			if v.Input.Throttle.Abs().Cmp(fixed.FromFraction(2, 100)) < 0 &&
				v.Input.Brake.Abs().Cmp(fixed.FromFraction(2, 100)) < 0 {
				keep = fixed.FromFraction(97, 100)
			}
		}

		targetLateral := lateralSpeed.Mul(keep)
		removeLateral := lateralSpeed.Sub(targetLateral)
		v.Velocity = v.Velocity.Sub(v.RightWS.Scale(removeLateral))
	}
}

type VehicleWorldDay6 struct {
	Tick     uint64
	Ground   GroundQuery
	Vehicles []Vehicle
}

func (w *VehicleWorldDay6) Step(dt fixed.Fixed) {
	for i := range w.Vehicles {
		w.Vehicles[i].Step(dt, w.Ground)
	}
	w.Tick++
}
