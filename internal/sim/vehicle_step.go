package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func StepVehicleWorld(w *VehicleWorld) {
	for i := range w.Vehicles {
		StepVehicle(w, &w.Vehicles[i])
	}
	w.Tick++
}

func StepVehicle(w *VehicleWorld, v *Vehicle) {
	applySteering(v)

	totalForce := geom.V3(
		fixed.Zero,
		VehicleGravityY.Mul(v.Body.Mass),
		fixed.Zero,
	)

	hardMinY, hasHardMin := fixed.Zero, false

	forward, right, _ := VehicleBasis(*v)

	for wi := range v.Wheels {
		wheel := &v.Wheels[wi]

		wheel.Contact = false
		wheel.ContactPoint = geom.Zero()
		wheel.ContactNormal = geom.Zero()
		wheel.Load = fixed.Zero

		mount := VehicleWheelMountWorld(*v, wi)

		hit := SampleGroundAtXZ(
			w.GroundTriangles,
			mount.X,
			mount.Z,
			mount.Y.Add(fixed.One), // Add 1m slop to catch ground if we slightly penetrate
		)

		wheel.PrevCompression = wheel.Compression
		wheel.Compression = fixed.Zero
		wheel.SuspensionLen = wheel.SuspensionRest

		if !hit.Hit {
			continue
		}

		suspLen := mount.Y.Sub(hit.Point.Y.Add(wheel.Radius))
		if suspLen.Cmp(wheel.SuspensionRest) > 0 {
			continue
		}

		if suspLen.Cmp(fixed.Zero) < 0 {
			suspLen = fixed.Zero
		}

		compression := wheel.SuspensionRest.Sub(suspLen)
		compVel := compression.Sub(wheel.PrevCompression).Div(VehicleDt)

		springForce := compression.Mul(wheel.SpringK)
		damperForce := compVel.Mul(wheel.DamperC)
		supportForce := springForce.Add(damperForce)

		if supportForce.Cmp(fixed.Zero) < 0 {
			supportForce = fixed.Zero
		}

		wheel.Contact = true
		wheel.ContactPoint = hit.Point
		wheel.ContactNormal = hit.Normal
		wheel.SuspensionLen = suspLen
		wheel.Compression = compression
		wheel.Load = supportForce

		totalForce = totalForce.Add(hit.Normal.Scale(supportForce))

		// drive
		if wheel.Driven && v.Input.Throttle.Cmp(fixed.Zero) > 0 {
			totalForce = totalForce.Add(
				forward.Scale(v.Input.Throttle.Mul(wheel.DriveForce)),
			)
		}

		// brake
		if v.Input.Brake.Cmp(fixed.Zero) > 0 {
			fwdSpeed := v.Body.Vel.Dot(forward)
			if fwdSpeed.Cmp(fixed.Zero) != 0 {
				brakeDir := signUnit(fwdSpeed)
				totalForce = totalForce.Sub(
					forward.Scale(brakeDir.Mul(v.Input.Brake).Mul(wheel.BrakeForce)),
				)
			}
		}

		// lateral grip: remove sideways motion gradually
		lateralSpeed := v.Body.Vel.Dot(right)
		totalForce = totalForce.Add(
			right.Scale(lateralSpeed.Neg().Mul(wheel.LateralGrip)),
		)

		// hard safety minimum body height
		requiredBodyY := hit.Point.Y.Add(wheel.Radius).Sub(wheel.LocalMount.Y)
		if !hasHardMin || requiredBodyY.Cmp(hardMinY) > 0 {
			hardMinY = requiredBodyY
			hasHardMin = true
		}
	}

	acc := totalForce.Scale(v.Body.InvMass)

	v.Body.Vel = v.Body.Vel.Add(acc.Scale(VehicleDt))
	v.Body.Vel = applyVehicleLinearDrag(v.Body.Vel)

	v.Body.Vel = clampVehicleSpeed(v.Body.Vel)
	v.Body.Pos = v.Body.Pos.Add(v.Body.Vel.Scale(VehicleDt))

	if hasHardMin && v.Body.Pos.Y.Cmp(hardMinY) < 0 {
		v.Body.Pos.Y = hardMinY
		if v.Body.Vel.Y.Cmp(fixed.Zero) < 0 {
			v.Body.Vel.Y = fixed.Zero
		}
	}

	refreshWheelContacts(w, v)
}

func refreshWheelContacts(w *VehicleWorld, v *Vehicle) {
	v.Body.OnGround = false

	for wi := range v.Wheels {
		wheel := &v.Wheels[wi]
		mount := VehicleWheelMountWorld(*v, wi)

		hit := SampleGroundAtXZ(
			w.GroundTriangles,
			mount.X,
			mount.Z,
			mount.Y.Add(fixed.One), // Add 1m slop to catch ground if we slightly penetrate
		)

		wheel.Contact = false
		wheel.ContactPoint = geom.Zero()
		wheel.ContactNormal = geom.Zero()
		wheel.Load = fixed.Zero

		if !hit.Hit {
			wheel.SuspensionLen = wheel.SuspensionRest
			wheel.Compression = fixed.Zero
			continue
		}

		suspLen := mount.Y.Sub(hit.Point.Y.Add(wheel.Radius))
		if suspLen.Cmp(wheel.SuspensionRest) > 0 {
			wheel.SuspensionLen = wheel.SuspensionRest
			wheel.Compression = fixed.Zero
			continue
		}

		if suspLen.Cmp(fixed.Zero) < 0 {
			suspLen = fixed.Zero
		}

		compression := wheel.SuspensionRest.Sub(suspLen)
		if compression.Cmp(fixed.Zero) < 0 {
			compression = fixed.Zero
		}

		wheel.Contact = true
		wheel.ContactPoint = hit.Point
		wheel.ContactNormal = hit.Normal
		wheel.SuspensionLen = suspLen
		wheel.Compression = compression

		if hit.Normal.Y.Cmp(fixed.FromFraction(1, 5)) > 0 {
			v.Body.OnGround = true
		}
	}
}

func applySteering(v *Vehicle) {
	if v.Input.Steer == fixed.Zero {
		return
	}

	forward, right, _ := VehicleBasis(*v)

	rotStep := v.Input.Steer.Mul(VehicleSteerRate).Mul(VehicleDt)
	newForward := forward.Add(right.Scale(rotStep))
	newForward = normalizeHorizontal(newForward)

	v.Body.Forward = newForward
}

func applyVehicleLinearDrag(vel geom.Vec3) geom.Vec3 {
	dragStep := VehicleLinearDrag.Mul(VehicleDt)

	vel.X = approachZero(vel.X, dragStep)
	vel.Z = approachZero(vel.Z, dragStep)

	return vel
}

func clampVehicleSpeed(vel geom.Vec3) geom.Vec3 {
	h := geom.V3(vel.X, fixed.Zero, vel.Z)
	speedSq := h.LengthSq()
	maxSq := VehicleMaxSpeed.Mul(VehicleMaxSpeed)

	if speedSq.Cmp(maxSq) <= 0 || speedSq == fixed.Zero {
		return vel
	}

	speed := speedSq.Sqrt()
	scale := VehicleMaxSpeed.Div(speed)

	vel.X = vel.X.Mul(scale)
	vel.Z = vel.Z.Mul(scale)

	return vel
}

func signUnit(v fixed.Fixed) fixed.Fixed {
	if v.Cmp(fixed.Zero) > 0 {
		return fixed.One
	}
	if v.Cmp(fixed.Zero) < 0 {
		return fixed.One.Neg()
	}
	return fixed.Zero
}
