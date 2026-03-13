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
		torque := r.X.Mul(f.Z).Sub(r.Z.Mul(f.X))
		v.TotalTorqueY = v.TotalTorqueY.Add(torque)
	}
}

func (v *Vehicle) refreshFinalContactState(dt fixed.Fixed, g GroundQuery) {
	v.finalizeSuspensionState()
}

func (v *Vehicle) Step(dt fixed.Fixed, g GroundQuery) {
	v.UpdateBasisFromYaw()
	v.UpdateSteering(dt)

	startPose := VehiclePose{
		Position: v.Position,
		Yaw:      v.Yaw,
	}

	// 1) Collect forces from current state
	v.collectCurrentForces(dt, g)

	// 2) Predict end of tick
	fullPos, fullVel, fullYaw, fullYawVel := v.predictFromCurrentState(dt, VehicleGravity)
	endPose := VehiclePose{
		Position: fullPos,
		Yaw:      fullYaw,
	}

	// 3) Sweep for earliest Ground TOI
	toi := v.FindGroundTOI(startPose, endPose, g)

	if !toi.Hit {
		// Normal discrete step
		v.Position = fullPos
		v.Velocity = fullVel
		v.Yaw = fullYaw
		v.YawVelocity = fullYawVel

		v.applyPostStepDamping()

		v.UpdateBasisFromYaw()
		v.refreshFinalContactState(dt, g)
		return
	}

	// 4) Impact! Advance to hit time
	hitDt := dt.Mul(toi.Time)
	v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(hitDt, VehicleGravity)

	// 5) Resolve impact
	v.resolveGroundImpactVelocity(toi.Normal)
	v.applyCCDSeparationBias(toi.Normal, toi.Depth)
	v.UpdateBasisFromYaw()

	// 6) Integrate remaining time
	remDt := dt.Sub(hitDt)
	if remDt.Cmp(fixed.Zero) > 0 {
		v.collectCurrentForces(remDt, g)
		v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(remDt, VehicleGravity)
	}

	// 7) Finalize
	v.applyPostStepDamping()
	v.UpdateBasisFromYaw()
	v.refreshFinalContactState(dt, g)
}

// applyPostStepDamping is called once per full physics tick (not per sub-step).
// It applies:
// 1) Input-aware yaw stabilization: when not steering, 50% yaw drag breaks
//    the lateral-torque feedback loop that keeps the car spinning.
// 2) Light XZ velocity damping (1%/tick): drains residual lateral drift after
//    releasing controls without freezing slow-moving vehicles.
func (v *Vehicle) applyPostStepDamping() {
	// Yaw stabilization when not actively steering
	if v.Input.Steer.Abs().Cmp(fixed.FromFraction(1, 100)) < 0 {
		v.YawVelocity = v.YawVelocity.Mul(fixed.FromFraction(50, 100))
		if v.YawVelocity.Abs().Cmp(fixed.FromFraction(5, 1000)) < 0 {
			v.YawVelocity = fixed.Zero
		}
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
