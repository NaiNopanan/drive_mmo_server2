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

		// 3D Torque = r x f
		r := v.Wheels[i].ContactPoint.Sub(v.Position)
		torque := r.Cross(f)

		// Project torque onto local basis.
		// We care about rotation around the vehicle's local up axis so
		// the car still behaves predictably on slopes.
		v.TotalTorqueY = v.TotalTorqueY.Add(torque.Dot(v.UpWS))
	}
}

func (v *Vehicle) refreshFinalContactState(dt fixed.Fixed, g GroundQuery) {
	// Re-sample contacts at the FINAL pose of this tick.
	// เดิมฟังก์ชันนี้ finalize อย่างเดียว ทำให้สถานะ grounded/contactNormal
	// ตอนจบ tick ยังอิงข้อมูลเก่าจากต้น tick อยู่
	for i := range v.Wheels {
		v.queryWheelGround(i, g)
		v.computeWheelSuspensionForce(i, dt)
	}
	v.finalizeSuspensionState()
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
		// Normal discrete step.
		v.Position = fullPos
		v.Velocity = fullVel
		v.Yaw = fullYaw
		v.YawVelocity = fullYawVel
	} else {
		// 4) Impact: advance to hit time.
		hitDt := dt.Mul(toi.Time)
		v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(hitDt, VehicleGravity)

		// 5) Resolve impact velocity and nudge out of penetration.
		v.resolveGroundImpactVelocity(toi.Normal)
		v.applyCCDSeparationBias(toi.Normal, toi.Depth)

		// 6) Rebuild basis + recollect forces from the hit pose before
		// integrating the remaining time. This is the big fix that makes
		// landing / entering a slope feel much less “floaty”.
		remDt := dt.Sub(hitDt)
		if remDt.Cmp(fixed.Zero) > 0 {
			v.UpdateBasis(toi.Normal)
			v.collectCurrentForces(remDt, g)
			v.Position, v.Velocity, v.Yaw, v.YawVelocity = v.predictFromCurrentState(remDt, VehicleGravity)
		}
	}

	// 7) Refresh actual grounded/contact state at the FINAL pose.
	v.refreshFinalContactState(dt, g)

	// 8) Build final average normal from actual wheel contacts.
	avgNormal := geom.V3(fixed.Zero, fixed.One, fixed.Zero)
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
			avgNormal = sumN.Scale(fixed.FromFraction(1, int64(count))).Normalize()
		}
	}

	// 9) Update basis once at the end so post-step damping uses the newest slope basis
	// and the next frame starts from the final orientation.
	v.UpdateBasis(avgNormal)

	// 10) Damping after final basis/contact state is known.
	v.applyPostStepDamping()
}

// applyPostStepDamping is called once per full physics tick (not per sub-step).
// Goals:
// 1) Keep yaw from “snapping dead” when steering is released.
// 2) Bleed lateral drift gently while preserving forward / reverse motion.
func (v *Vehicle) applyPostStepDamping() {
	steerDeadzone := fixed.FromFraction(1, 100) // 0.01
	steerActive := v.Input.Steer.Abs().Cmp(steerDeadzone) >= 0

	// Softer yaw stabilization when not actively steering.
	if !steerActive {
		v.YawVelocity = v.YawVelocity.Mul(fixed.FromFraction(90, 100))
		if v.YawVelocity.Abs().Cmp(fixed.FromFraction(5, 1000)) < 0 {
			v.YawVelocity = fixed.Zero
		}
	}

	// Bleed sideways slip without killing longitudinal motion.
	// Use the FINAL basis so this still behaves sensibly on slopes.
	if v.OnGround {
		lateralSpeed := v.Velocity.Dot(v.RightWS)

		keep := fixed.FromFraction(90, 100) // 10% bleed by default
		if steerActive {
			keep = fixed.FromFraction(94, 100) // keep a bit more lateral motion while actively turning
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
