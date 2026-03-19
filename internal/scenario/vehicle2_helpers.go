package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

var vehicle2WheelLocalOffsetsSphere = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.Zero, fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.Zero, fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.Zero, fixed.FromFraction(-8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.Zero, fixed.FromFraction(-8, 5)),
}

func makeVehicle2MMOSceneState(position geometry.Vector3, orientation physics.Quaternion) SceneState {
	chassis := physics.NewRigidBoxBody3D(
		fixed.FromInt(8),
		geometry.NewVector3(fixed.FromFraction(9, 10), fixed.FromFraction(7, 20), fixed.FromFraction(8, 5)),
		position,
	)
	chassis.Restitution = fixed.Zero
	chassis.Orientation = orientation

	probeOffsets := append([]geometry.Vector3(nil), vehicle2WheelLocalOffsetsSphere...)

	return SceneState{
		VehicleChassis:              chassis,
		VehicleProbeLocalOffsets:    probeOffsets,
		VehicleWheelRadius:          fixed.Zero,
		VehicleUseWheelCollider:     false,
		VehicleWheelCorrectionClamp: fixed.Zero,
		VehicleWheelProbes:          make([]VehicleWheelProbeState, len(probeOffsets)),
		GroundTriangles:             makeFlatGroundTriangles(),
	}
}

func StepVehicle2MMOCurbSupportScene(state *SceneState) {
	planePoint, planeNormal := vehicle2CurbBumpPlane(vehicle2BodyPositionZ(state))
	stepVehicle2SceneOnPlane(state, fixed.FromInt(40), fixed.Zero, planePoint, planeNormal)
}

func stepVehicle2SceneOnPlane(state *SceneState, driveForce, steerTorque fixed.Fixed, planePoint, planeNormal geometry.Vector3) {
	if state == nil {
		return
	}
	if planeNormal.LengthSquared() == fixed.Zero {
		planeNormal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	planeNormal = planeNormal.Normalize()

	body := &state.VehicleChassis
	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.VehicleGroundedProbeCount = 0
	state.VehicleAverageCompression = fixed.Zero
	state.VehicleSettled = false
	if len(state.VehicleProbeLocalOffsets) == 0 {
		state.VehicleProbeLocalOffsets = append([]geometry.Vector3(nil), vehicle2WheelLocalOffsetsSphere...)
	}
	if len(state.VehicleWheelProbes) != len(state.VehicleProbeLocalOffsets) {
		state.VehicleWheelProbes = make([]VehicleWheelProbeState, len(state.VehicleProbeLocalOffsets))
	}

	const (
		restLengthNum    = 7
		restLengthDen    = 10
		springNum        = 42
		springDen        = 1
		damperNum        = 6
		damperDen        = 1
		wheelColliderNum = 90
		wheelColliderDen = 1
	)
	restLength := fixed.FromFraction(restLengthNum, restLengthDen)
	springStrength := fixed.FromFraction(springNum, springDen)
	damperStrength := fixed.FromFraction(damperNum, damperDen)
	wheelColliderStrength := fixed.FromFraction(wheelColliderNum, wheelColliderDen)

	for index, localOffset := range state.VehicleProbeLocalOffsets {
		probe := VehicleWheelProbeState{LocalOffset: localOffset}
		worldOffset := body.Orientation.RotateVector(localOffset)
		probe.WorldPosition = body.Motion.Position.Add(worldOffset)
		centerDistance := probe.WorldPosition.Sub(planePoint).Dot(planeNormal)
		if state.VehicleUseWheelCollider && state.VehicleWheelRadius.Cmp(fixed.Zero) > 0 && centerDistance.Cmp(state.VehicleWheelRadius) < 0 {
			penetration := state.VehicleWheelRadius.Sub(centerDistance)
			if state.VehicleWheelCorrectionClamp.Cmp(fixed.Zero) > 0 && penetration.Cmp(state.VehicleWheelCorrectionClamp) > 0 {
				penetration = state.VehicleWheelCorrectionClamp
			}
			vehicle2ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, planeNormal.Scale(penetration.Mul(wheelColliderStrength)), physics.DefaultTimeStep)
			body.Motion.Position = body.Motion.Position.Add(planeNormal.Scale(penetration))
			probe.WorldPosition = probe.WorldPosition.Add(planeNormal.Scale(penetration))
			centerDistance = state.VehicleWheelRadius
		}
		suspensionLength := centerDistance.Sub(state.VehicleWheelRadius)
		if suspensionLength.Cmp(restLength) <= 0 {
			compression := restLength.Sub(suspensionLength)
			if compression.Cmp(fixed.Zero) < 0 {
				compression = fixed.Zero
			}
			pointVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(worldOffset))
			dampingForce := pointVelocity.Dot(planeNormal).Mul(damperStrength)
			forceMagnitude := compression.Mul(springStrength).Sub(dampingForce)
			if forceMagnitude.Cmp(fixed.Zero) < 0 {
				forceMagnitude = fixed.Zero
			}
			vehicle2ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, planeNormal.Scale(forceMagnitude), physics.DefaultTimeStep)
			probe.Grounded = true
			probe.Compression = compression
			probe.ContactPoint = probe.WorldPosition.Sub(planeNormal.Scale(suspensionLength.Add(state.VehicleWheelRadius)))
			probe.ContactNormal = planeNormal
			state.VehicleGroundedProbeCount++
			state.VehicleAverageCompression = state.VehicleAverageCompression.Add(compression)
		}
		state.VehicleWheelProbes[index] = probe
	}

	if state.VehicleGroundedProbeCount > 0 {
		state.VehicleAverageCompression = state.VehicleAverageCompression.Div(fixed.FromInt(int64(state.VehicleGroundedProbeCount)))
	}
	if driveForce.Cmp(fixed.Zero) != 0 && state.VehicleGroundedProbeCount >= 2 {
		forward := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One))
		forward = forward.Sub(planeNormal.Scale(forward.Dot(planeNormal)))
		if forward.LengthSquared() != fixed.Zero {
			physics.ApplyForce(&body.Motion, forward.Normalize().Scale(driveForce))
		}
	}
	if steerTorque.Cmp(fixed.Zero) != 0 && state.VehicleGroundedProbeCount >= 2 {
		vehicle2ApplyTorqueToRigidBox(body, geometry.NewVector3(fixed.Zero, steerTorque, fixed.Zero), physics.DefaultTimeStep)
	}

	vehicle2ApplyUprightAssist(body, physics.DefaultTimeStep, planeNormal)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(99, 100))
	body.AngularVelocity = body.AngularVelocity.Scale(fixed.FromFraction(47, 50))

	result := physics.StepRigidBoxBody3DWithGravityAndPlaneOverride(
		body,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		planePoint,
		planeNormal,
		fixed.Zero,
	)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(99, 100))
	if result.HadContact {
		state.LastContact = result.LastContact
		state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
		state.EverTouchedGround = true
	}
	vehicle2ResolveWallContacts(state, body)

	bodyUp := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	state.VehicleUprightDot = bodyUp.Dot(planeNormal)
	if state.VehicleUprightDot.Cmp(fixed.One) > 0 {
		state.VehicleUprightDot = fixed.One
	}
	if state.VehicleGroundedProbeCount >= 2 &&
		body.Motion.Velocity.Length().Cmp(fixed.FromFraction(1, 5)) <= 0 &&
		body.AngularVelocity.Length().Cmp(fixed.FromFraction(1, 4)) <= 0 &&
		state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) >= 0 {
		state.VehicleSettled = true
	}

	state.RigidBox = *body
}

func vehicle2ApplyForceAtRigidBoxPoint(body *physics.RigidBoxBody3D, worldPoint, force geometry.Vector3, dt fixed.Fixed) {
	if body == nil || force.LengthSquared() == fixed.Zero {
		return
	}

	physics.ApplyForce(&body.Motion, force)
	torque := worldPoint.Sub(body.Motion.Position).Cross(force)
	vehicle2ApplyTorqueToRigidBox(body, torque, dt)
}

func vehicle2ApplyTorqueToRigidBox(body *physics.RigidBoxBody3D, torque geometry.Vector3, dt fixed.Fixed) {
	if body == nil || torque.LengthSquared() == fixed.Zero {
		return
	}

	localTorque := body.Orientation.Conjugate().RotateVector(torque)
	localAngularAccel := geometry.NewVector3(
		localTorque.X.Mul(body.InverseInertiaBody.X),
		localTorque.Y.Mul(body.InverseInertiaBody.Y),
		localTorque.Z.Mul(body.InverseInertiaBody.Z),
	)
	worldAngularAccel := body.Orientation.RotateVector(localAngularAccel)
	body.AngularVelocity = body.AngularVelocity.Add(worldAngularAccel.Scale(dt))
}

func vehicle2ApplyUprightAssist(body *physics.RigidBoxBody3D, dt fixed.Fixed, targetUp geometry.Vector3) {
	if body == nil {
		return
	}
	if targetUp.LengthSquared() == fixed.Zero {
		targetUp = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	targetUp = targetUp.Normalize()
	bodyUp := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	alignAxis := bodyUp.Cross(targetUp)
	if alignAxis.LengthSquared() != fixed.Zero {
		alignTorque := alignAxis.Scale(fixed.FromInt(10))
		vehicle2ApplyTorqueToRigidBox(body, alignTorque, dt)
	}
	body.AngularVelocity = geometry.NewVector3(
		body.AngularVelocity.X.Mul(fixed.FromFraction(9, 10)),
		body.AngularVelocity.Y.Mul(fixed.FromFraction(19, 20)),
		body.AngularVelocity.Z.Mul(fixed.FromFraction(9, 10)),
	)
}

func vehicle2ResolveWallContacts(state *SceneState, body *physics.RigidBoxBody3D) {
	if state == nil || body == nil || len(state.GroundBoxes) == 0 {
		return
	}

	for _, bounds := range state.GroundBoxes {
		sizeX := bounds.Max.X.Sub(bounds.Min.X)
		sizeZ := bounds.Max.Z.Sub(bounds.Min.Z)
		if sizeZ.Cmp(sizeX) <= 0 {
			planeZ := bounds.Min.Z
			planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
			planeNormal := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())
			if vehicle2FixedAbs(body.Motion.Position.Z.Sub(bounds.Max.Z)).Cmp(vehicle2FixedAbs(body.Motion.Position.Z.Sub(bounds.Min.Z))) < 0 {
				planeZ = bounds.Max.Z
				planePoint = geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
				planeNormal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
			}
			if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
				state.LastContact = result.LastContact
				state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
				state.VehicleEverHitWall = true
			}
			continue
		}

		planeX := bounds.Min.X
		planePoint := geometry.NewVector3(planeX, fixed.Zero, fixed.Zero)
		planeNormal := geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero)
		if vehicle2FixedAbs(body.Motion.Position.X.Sub(bounds.Max.X)).Cmp(vehicle2FixedAbs(body.Motion.Position.X.Sub(bounds.Min.X))) < 0 {
			planeX = bounds.Max.X
			planePoint = geometry.NewVector3(planeX, fixed.Zero, fixed.Zero)
			planeNormal = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
		}
		if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
			state.LastContact = result.LastContact
			state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
			state.VehicleEverHitWall = true
		}
	}
}

func makeVehicle2WallImpactBounds() geometry.AxisAlignedBoundingBox {
	return geometry.NewAxisAlignedBoundingBox(
		geometry.NewVector3(fixed.FromInt(-4), fixed.Zero, fixed.FromInt(8)),
		geometry.NewVector3(fixed.FromInt(4), fixed.FromInt(4), fixed.FromFraction(17, 2)),
	)
}

func makeVehicle2WallImpactTriangles() []geometry.Triangle {
	return makeVehicle2WallTrianglesFromBounds(makeVehicle2WallImpactBounds())
}

func makeVehicle2WallTrianglesFromBounds(wall geometry.AxisAlignedBoundingBox) []geometry.Triangle {
	a := geometry.NewVector3(wall.Min.X, wall.Min.Y, wall.Min.Z)
	b := geometry.NewVector3(wall.Max.X, wall.Min.Y, wall.Min.Z)
	c := geometry.NewVector3(wall.Min.X, wall.Max.Y, wall.Min.Z)
	d := geometry.NewVector3(wall.Max.X, wall.Max.Y, wall.Min.Z)
	e := geometry.NewVector3(wall.Min.X, wall.Min.Y, wall.Max.Z)
	f := geometry.NewVector3(wall.Max.X, wall.Min.Y, wall.Max.Z)
	g := geometry.NewVector3(wall.Min.X, wall.Max.Y, wall.Max.Z)
	h := geometry.NewVector3(wall.Max.X, wall.Max.Y, wall.Max.Z)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(b, c, d),
		geometry.NewTriangle(e, f, g),
		geometry.NewTriangle(f, h, g),
		geometry.NewTriangle(a, e, c),
		geometry.NewTriangle(e, g, c),
		geometry.NewTriangle(b, d, f),
		geometry.NewTriangle(f, d, h),
		geometry.NewTriangle(c, g, d),
		geometry.NewTriangle(g, h, d),
		geometry.NewTriangle(a, b, e),
		geometry.NewTriangle(e, b, f),
	}
}

func vehicle2FixedAbs(value fixed.Fixed) fixed.Fixed {
	if value.Cmp(fixed.Zero) < 0 {
		return value.Neg()
	}
	return value
}

func vehicle2CurbBumpPlane(positionZ fixed.Fixed) (geometry.Vector3, geometry.Vector3) {
	rampStartZ := fixed.FromInt(-1)
	topStartZ := fixed.One
	topEndZ := fixed.FromInt(3)
	rampEndZ := fixed.FromInt(5)
	bumpHeight := fixed.FromFraction(3, 5)
	rampSlope := bumpHeight.Div(topStartZ.Sub(rampStartZ))

	switch {
	case positionZ.Cmp(rampStartZ) < 0:
		return geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	case positionZ.Cmp(topStartZ) < 0:
		return geometry.NewVector3(fixed.Zero, rampStartZ.Mul(rampSlope.Neg()), fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, rampSlope.Neg()).Normalize()
	case positionZ.Cmp(topEndZ) < 0:
		return geometry.NewVector3(fixed.Zero, bumpHeight, fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	case positionZ.Cmp(rampEndZ) < 0:
		return geometry.NewVector3(fixed.Zero, rampEndZ.Mul(rampSlope), fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, rampSlope).Normalize()
	default:
		return geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
}

func makeVehicle2CurbBumpTriangles() []geometry.Triangle {
	minX := fixed.FromInt(-8)
	maxX := fixed.FromInt(8)
	minZ := fixed.FromInt(-20)
	rampStartZ := fixed.FromInt(-1)
	topStartZ := fixed.One
	topEndZ := fixed.FromInt(3)
	rampEndZ := fixed.FromInt(5)
	maxZ := fixed.FromInt(20)
	bumpHeight := fixed.FromFraction(3, 5)

	a := geometry.NewVector3(minX, fixed.Zero, minZ)
	b := geometry.NewVector3(maxX, fixed.Zero, minZ)
	c := geometry.NewVector3(minX, fixed.Zero, rampStartZ)
	d := geometry.NewVector3(maxX, fixed.Zero, rampStartZ)
	e := geometry.NewVector3(minX, bumpHeight, topStartZ)
	f := geometry.NewVector3(maxX, bumpHeight, topStartZ)
	g := geometry.NewVector3(minX, bumpHeight, topEndZ)
	h := geometry.NewVector3(maxX, bumpHeight, topEndZ)
	i := geometry.NewVector3(minX, fixed.Zero, rampEndZ)
	j := geometry.NewVector3(maxX, fixed.Zero, rampEndZ)
	k := geometry.NewVector3(minX, fixed.Zero, maxZ)
	l := geometry.NewVector3(maxX, fixed.Zero, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
		geometry.NewTriangle(c, e, d),
		geometry.NewTriangle(e, f, d),
		geometry.NewTriangle(e, g, f),
		geometry.NewTriangle(g, h, f),
		geometry.NewTriangle(g, i, h),
		geometry.NewTriangle(i, j, h),
		geometry.NewTriangle(i, k, j),
		geometry.NewTriangle(k, l, j),
	}
}

func vehicle2BodyPositionZ(state *SceneState) fixed.Fixed {
	if state == nil {
		return fixed.Zero
	}
	return state.VehicleChassis.Motion.Position.Z
}
