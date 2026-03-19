package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

var vehicle3WheelLocalOffsets = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-19, 20), fixed.FromFraction(-1, 10), fixed.FromFraction(17, 10)),
	geometry.NewVector3(fixed.FromFraction(19, 20), fixed.FromFraction(-1, 10), fixed.FromFraction(17, 10)),
	geometry.NewVector3(fixed.FromFraction(-19, 20), fixed.FromFraction(-1, 10), fixed.FromFraction(-17, 10)),
	geometry.NewVector3(fixed.FromFraction(19, 20), fixed.FromFraction(-1, 10), fixed.FromFraction(-17, 10)),
}

type vehicle3WheelRoadContact struct {
	hit         bool
	point       geometry.Vector3
	normal      geometry.Vector3
	penetration fixed.Fixed
}

type vehicle3AxleState struct {
	leftCompression    fixed.Fixed
	rightCompression   fixed.Fixed
	leftGrounded       bool
	rightGrounded      bool
	groundedProbeCount int
}

func makeVehicle3RayCitySceneState(position geometry.Vector3, orientation physics.Quaternion) SceneState {
	chassis := physics.NewRigidBoxBody3D(
		fixed.FromInt(7),
		geometry.NewVector3(fixed.FromFraction(19, 20), fixed.FromFraction(2, 5), fixed.FromFraction(17, 10)),
		position,
	)
	chassis.Restitution = fixed.Zero
	chassis.Orientation = orientation

	probeOffsets := append([]geometry.Vector3(nil), vehicle3WheelLocalOffsets...)

	return SceneState{
		VehicleChassis:                 chassis,
		VehicleProbeLocalOffsets:       probeOffsets,
		VehicleWheelRadius:             fixed.FromFraction(2, 5),
		VehicleWheelWidth:              fixed.FromFraction(1, 5),
		VehicleUseWheelCollider:        true,
		VehicleWheelCorrectionClamp:    fixed.FromFraction(1, 10),
		VehicleWheelProbes:             make([]VehicleWheelProbeState, len(probeOffsets)),
		VehicleThrottleInput:           fixed.Zero,
		VehicleSteerInput:              fixed.Zero,
		VehicleSuspensionRestLength:    fixed.FromFraction(7, 10),
		VehicleSuspensionSweepDistance: fixed.FromInt(4),
		VehicleSuspensionSpring:        fixed.FromInt(56),
		VehicleSuspensionDamper:        fixed.FromInt(8),
		VehicleDriveForce:              fixed.FromInt(36),
		VehicleEngineBrake:             fixed.FromInt(2),
		VehicleFrontDriveShare:         fixed.FromFraction(7, 20),
		VehicleFrontGrip:               fixed.FromInt(22),
		VehicleRearGrip:                fixed.FromInt(16),
		VehicleAntiRoll:                fixed.FromInt(14),
		VehicleMaxSteerAngle:           fixed.FromFraction(7, 25),
		VehicleRearSteerAssist:         fixed.FromFraction(1, 12),
		VehicleWheelGroundedGraceTicks: make([]int, len(probeOffsets)),
		GroundTriangles:                makeFlatGroundTriangles(),
	}
}

func StepVehicle3RayCityCruiseScene(state *SceneState) {
	if state == nil {
		return
	}
	state.VehicleThrottleInput = fixed.One
	state.VehicleSteerInput = fixed.Zero
	stepVehicle3RayCityScene(state)
}

func StepVehicle3RayCityCurbScene(state *SceneState) {
	if state == nil {
		return
	}
	state.VehicleThrottleInput = fixed.One
	state.VehicleSteerInput = fixed.Zero
	stepVehicle3RayCityScene(state)
}

func StepVehicle3RayCityTurnScene(state *SceneState) {
	if state == nil {
		return
	}
	state.VehicleThrottleInput = fixed.FromFraction(19, 20)
	state.VehicleSteerInput = fixed.FromFraction(11, 20)
	stepVehicle3RayCityScene(state)
}

func StepVehicle3RayCityManualScene(state *SceneState) {
	if state == nil {
		return
	}
	stepVehicle3RayCityScene(state)
}

func ApplyVehicle3AutoDriveInput(state *SceneState) {
	if state == nil {
		return
	}

	state.VehicleThrottleInput = fixed.One

	const segmentTicks = 90
	const waveSteerNum = 3
	const waveSteerDen = 20
	const centerPullDen = 6
	const maxSteerClampNum = 7
	const maxSteerClampDen = 20

	phase := int(state.Tick % (segmentTicks * 4))
	waveSteer := fixed.FromFraction(waveSteerNum, waveSteerDen)
	maxSteerClamp := fixed.FromFraction(maxSteerClampNum, maxSteerClampDen)
	steer := fixed.Zero

	switch {
	case phase < segmentTicks:
		ratio := fixed.FromFraction(int64(phase), segmentTicks)
		steer = waveSteer.Mul(ratio)
	case phase < segmentTicks*2:
		ratio := fixed.FromFraction(int64(phase-segmentTicks), segmentTicks)
		steer = waveSteer.Sub(waveSteer.Mul(ratio).Mul(fixed.FromInt(2)))
	case phase < segmentTicks*3:
		ratio := fixed.FromFraction(int64(phase-segmentTicks*2), segmentTicks)
		steer = waveSteer.Neg().Mul(fixed.One.Sub(ratio))
	default:
		ratio := fixed.FromFraction(int64(phase-segmentTicks*3), segmentTicks)
		steer = waveSteer.Neg().Add(waveSteer.Mul(ratio))
	}

	centerCorrection := state.VehicleChassis.Motion.Position.X.Div(fixed.FromInt(centerPullDen))
	steer = steer.Add(centerCorrection)
	if steer.Cmp(maxSteerClamp) > 0 {
		steer = maxSteerClamp
	}
	if steer.Cmp(maxSteerClamp.Neg()) < 0 {
		steer = maxSteerClamp.Neg()
	}
	state.VehicleSteerInput = steer
}

func stepVehicle3RayCityScene(state *SceneState) {
	if state == nil {
		return
	}

	body := &state.VehicleChassis
	previousWallContactActive := state.VehicleWallContactActive
	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.VehicleGroundedProbeCount = 0
	state.VehicleFrontGroundedProbeCount = 0
	state.VehicleRearGroundedProbeCount = 0
	state.VehicleWallContactActive = false
	state.VehicleAverageCompression = fixed.Zero
	state.VehicleSettled = false
	if len(state.VehicleProbeLocalOffsets) == 0 {
		state.VehicleProbeLocalOffsets = append([]geometry.Vector3(nil), vehicle3WheelLocalOffsets...)
	}
	if len(state.VehicleWheelProbes) != len(state.VehicleProbeLocalOffsets) {
		state.VehicleWheelProbes = make([]VehicleWheelProbeState, len(state.VehicleProbeLocalOffsets))
	}
	if len(state.VehicleWheelGroundedGraceTicks) != len(state.VehicleProbeLocalOffsets) {
		state.VehicleWheelGroundedGraceTicks = make([]int, len(state.VehicleProbeLocalOffsets))
	}

	wheelColliderStrength := fixed.FromInt(110)
	restLength := state.VehicleSuspensionRestLength
	springStrength := state.VehicleSuspensionSpring
	damperStrength := state.VehicleSuspensionDamper
	frontGripStrength := state.VehicleFrontGrip
	rearGripStrength := state.VehicleRearGrip
	driveForceMagnitude := state.VehicleDriveForce
	engineBrakeMagnitude := state.VehicleEngineBrake
	frontDriveShare := state.VehicleFrontDriveShare
	rearDriveShare := fixed.One.Sub(frontDriveShare)
	antiRollStrength := state.VehicleAntiRoll
	maxSteerAngle := state.VehicleMaxSteerAngle
	rearSteerAssist := state.VehicleRearSteerAssist
	suspensionRadius := restLength.Add(state.VehicleWheelRadius)
	sweepDistance := state.VehicleSuspensionSweepDistance
	if sweepDistance.Cmp(suspensionRadius) < 0 {
		sweepDistance = suspensionRadius
	}
	persistenceDistance := sweepDistance.Add(fixed.FromFraction(4, 5))
	const groundedGraceTicks = 6

	targetUp := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero)
	axles := [2]vehicle3AxleState{}
	previousProbes := append([]VehicleWheelProbeState(nil), state.VehicleWheelProbes...)
	previousGraceTicks := append([]int(nil), state.VehicleWheelGroundedGraceTicks...)
	wallForceScale := fixed.One
	if previousWallContactActive {
		wallForceScale = fixed.FromFraction(7, 20)
	}

	bodyForward := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One))
	if bodyForward.LengthSquared() == fixed.Zero {
		bodyForward = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
	}
	bodyForward = bodyForward.Normalize()
	bodyRight := body.Orientation.RotateVector(geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero))
	if bodyRight.LengthSquared() == fixed.Zero {
		bodyRight = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
	}
	bodyRight = bodyRight.Normalize()
	bodyUp := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	if bodyUp.LengthSquared() == fixed.Zero {
		bodyUp = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	sweepDown := geometry.NewVector3(fixed.Zero, fixed.One.Neg(), fixed.Zero)

	for index, localOffset := range state.VehicleProbeLocalOffsets {
		probe := VehicleWheelProbeState{LocalOffset: localOffset}
		worldOffset := body.Orientation.RotateVector(localOffset)
		probe.WorldPosition = body.Motion.Position.Add(worldOffset)

		contact := vehicle3SampleWheelRoadContact(state.GroundTriangles, probe.WorldPosition, suspensionRadius, bodyRight, state.VehicleWheelWidth, sweepDown, sweepDistance)
		if !contact.hit && index < len(previousProbes) {
			contact = vehicle3PersistentWheelRoadContact(previousProbes[index], previousGraceTicks[index], probe.WorldPosition, persistenceDistance)
		}
		if !contact.hit {
			if index < len(state.VehicleWheelGroundedGraceTicks) && index < len(previousGraceTicks) && previousGraceTicks[index] > 0 {
				state.VehicleWheelGroundedGraceTicks[index] = previousGraceTicks[index] - 1
			}
			state.VehicleWheelProbes[index] = probe
			continue
		}

		centerDelta := probe.WorldPosition.Sub(contact.point)
		centerDistance := centerDelta.Length()
		if centerDistance.Cmp(fixed.Zero) < 0 {
			centerDistance = fixed.Zero
		}

		if state.VehicleUseWheelCollider && state.VehicleWheelRadius.Cmp(fixed.Zero) > 0 && centerDistance.Cmp(state.VehicleWheelRadius) < 0 {
			penetration := state.VehicleWheelRadius.Sub(centerDistance)
			if state.VehicleWheelCorrectionClamp.Cmp(fixed.Zero) > 0 && penetration.Cmp(state.VehicleWheelCorrectionClamp) > 0 {
				penetration = state.VehicleWheelCorrectionClamp
			}
			vehicle3ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, contact.normal.Scale(penetration.Mul(wheelColliderStrength)), physics.DefaultTimeStep)
			body.Motion.Position = body.Motion.Position.Add(contact.normal.Scale(penetration))
			probe.WorldPosition = probe.WorldPosition.Add(contact.normal.Scale(penetration))
			centerDistance = state.VehicleWheelRadius
		}

		suspensionLength := centerDistance.Sub(state.VehicleWheelRadius)
		if suspensionLength.Cmp(restLength) > 0 {
			state.VehicleWheelProbes[index] = probe
			continue
		}

		compression := restLength.Sub(suspensionLength)
		if compression.Cmp(fixed.Zero) < 0 {
			compression = fixed.Zero
		}

		contactNormal := contact.normal
		targetUp = targetUp.Add(contactNormal)

		wheelForward := bodyForward.Sub(contactNormal.Scale(bodyForward.Dot(contactNormal)))
		if wheelForward.LengthSquared() == fixed.Zero {
			wheelForward = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
		}
		wheelForward = wheelForward.Normalize()

		isFrontAxle := localOffset.Z.Cmp(fixed.Zero) >= 0
		steerAngle := fixed.Zero
		if isFrontAxle {
			steerAngle = state.VehicleSteerInput.Mul(maxSteerAngle)
		} else {
			steerAngle = state.VehicleSteerInput.Mul(maxSteerAngle).Mul(rearSteerAssist)
		}
		if steerAngle.Cmp(fixed.Zero) != 0 {
			wheelForward = vehicle3RotateAroundAxis(wheelForward, contactNormal, steerAngle).Normalize()
		}
		wheelRight := contactNormal.Cross(wheelForward)
		if wheelRight.LengthSquared() == fixed.Zero {
			wheelRight = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
		}
		wheelRight = wheelRight.Normalize()
		probe.WheelRight = wheelRight

		pointVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(worldOffset))
		dampingForce := pointVelocity.Dot(contactNormal).Mul(damperStrength)
		supportForceMagnitude := compression.Mul(springStrength).Sub(dampingForce)
		if supportForceMagnitude.Cmp(fixed.Zero) < 0 {
			supportForceMagnitude = fixed.Zero
		}
		vehicle3ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, contactNormal.Scale(supportForceMagnitude.Mul(wallForceScale)), physics.DefaultTimeStep)

		gripStrength := rearGripStrength
		if isFrontAxle {
			gripStrength = frontGripStrength
		}
		lateralSpeed := pointVelocity.Dot(wheelRight)
		if lateralSpeed.Cmp(fixed.Zero) != 0 {
			vehicle3ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, wheelRight.Scale(lateralSpeed.Mul(gripStrength).Mul(wallForceScale).Neg()), physics.DefaultTimeStep)
		}

		longitudinalSpeed := pointVelocity.Dot(wheelForward)
		driveShare := rearDriveShare
		axleIndex := 1
		if isFrontAxle {
			driveShare = frontDriveShare
			axleIndex = 0
		}
		driveForce := state.VehicleThrottleInput.Mul(driveForceMagnitude).Mul(driveShare)
		if driveForce.Cmp(fixed.Zero) != 0 {
			vehicle3ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, wheelForward.Scale(driveForce.Mul(wallForceScale)), physics.DefaultTimeStep)
		}
		if state.VehicleThrottleInput.Cmp(fixed.Zero) == 0 && longitudinalSpeed.Cmp(fixed.Zero) != 0 {
			vehicle3ApplyForceAtRigidBoxPoint(body, probe.WorldPosition, wheelForward.Scale(longitudinalSpeed.Mul(engineBrakeMagnitude).Neg()), physics.DefaultTimeStep)
		}

		probe.Grounded = true
		probe.Compression = compression
		probe.ContactPoint = contact.point
		probe.ContactNormal = contactNormal
		state.VehicleWheelGroundedGraceTicks[index] = groundedGraceTicks
		state.VehicleGroundedProbeCount++
		state.VehicleAverageCompression = state.VehicleAverageCompression.Add(compression)
		if isFrontAxle {
			state.VehicleFrontGroundedProbeCount++
		} else {
			state.VehicleRearGroundedProbeCount++
		}

		if localOffset.X.Cmp(fixed.Zero) < 0 {
			axles[axleIndex].leftCompression = compression
			axles[axleIndex].leftGrounded = true
		} else {
			axles[axleIndex].rightCompression = compression
			axles[axleIndex].rightGrounded = true
		}
		axles[axleIndex].groundedProbeCount++
		state.VehicleWheelProbes[index] = probe
	}

	if vehicle3ResolveWallContacts(state, body) {
		wallForceScale = fixed.FromFraction(7, 20)
		state.VehicleWallContactActive = true
	}

	for axleIndex, axle := range axles {
		if !axle.leftGrounded && !axle.rightGrounded {
			continue
		}
		compressionDelta := axle.leftCompression.Sub(axle.rightCompression)
		if compressionDelta.Cmp(fixed.Zero) == 0 {
			continue
		}

		leftIndex := axleIndex * 2
		rightIndex := leftIndex + 1
		leftProbe := state.VehicleWheelProbes[leftIndex]
		rightProbe := state.VehicleWheelProbes[rightIndex]
		antiRollForce := compressionDelta.Mul(antiRollStrength).Mul(wallForceScale)
		if leftProbe.Grounded {
			vehicle3ApplyForceAtRigidBoxPoint(body, leftProbe.WorldPosition, leftProbe.ContactNormal.Scale(antiRollForce.Neg()), physics.DefaultTimeStep)
		}
		if rightProbe.Grounded {
			vehicle3ApplyForceAtRigidBoxPoint(body, rightProbe.WorldPosition, rightProbe.ContactNormal.Scale(antiRollForce), physics.DefaultTimeStep)
		}
	}

	if state.VehicleGroundedProbeCount > 0 {
		state.VehicleAverageCompression = state.VehicleAverageCompression.Div(fixed.FromInt(int64(state.VehicleGroundedProbeCount)))
		targetUp = targetUp.Scale(fixed.One.Div(fixed.FromInt(int64(state.VehicleGroundedProbeCount)))).Normalize()
	} else {
		targetUp = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}

	if state.VehicleGroundedProbeCount == 0 {
		speed := body.Motion.Velocity.Length()
		if speed.Cmp(fixed.FromFraction(1, 10)) > 0 {
			stickForce := speed.Mul(fixed.FromInt(14))
			physics.ApplyForce(&body.Motion, geometry.NewVector3(fixed.Zero, stickForce.Neg(), fixed.Zero))
		}
	}

	vehicle3ApplyUprightAssist(body, physics.DefaultTimeStep, targetUp)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(197, 200))
	body.AngularVelocity = body.AngularVelocity.Scale(fixed.FromFraction(23, 25))

	floorPoint, floorNormal := vehicle3ChassisSupportPlane(state, body)
	result := physics.StepRigidBoxBody3DWithGravityAndPlaneOverride(
		body,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		floorPoint,
		floorNormal,
		fixed.Zero,
	)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(197, 200))
	if result.HadContact {
		state.LastContact = result.LastContact
		state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
		state.EverTouchedGround = true
	}
	bodyUp = body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	state.VehicleUprightDot = bodyUp.Dot(targetUp)
	if state.VehicleUprightDot.Cmp(fixed.One) > 0 {
		state.VehicleUprightDot = fixed.One
	}
	if state.VehicleGroundedProbeCount >= 2 &&
		body.Motion.Velocity.Length().Cmp(fixed.FromFraction(1, 4)) <= 0 &&
		body.AngularVelocity.Length().Cmp(fixed.FromFraction(7, 20)) <= 0 &&
		state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) >= 0 {
		state.VehicleSettled = true
	}

	state.RigidBox = *body
}

func vehicle3SampleWheelRoadContact(triangles []geometry.Triangle, center geometry.Vector3, radius fixed.Fixed, lateralAxis geometry.Vector3, wheelWidth fixed.Fixed, downDirection geometry.Vector3, sweepDistance fixed.Fixed) vehicle3WheelRoadContact {
	best := vehicle3WheelRoadContact{}
	sampleCenters := []geometry.Vector3{center}
	if lateralAxis.LengthSquared() != fixed.Zero && wheelWidth.Cmp(fixed.Zero) > 0 {
		halfWidth := wheelWidth.Div(fixed.FromInt(2))
		lateralOffset := lateralAxis.Normalize().Scale(halfWidth)
		sampleCenters = append(sampleCenters, center.Add(lateralOffset), center.Sub(lateralOffset))
	}
	for _, sampleCenter := range sampleCenters {
		best = vehicle3AccumulateWheelContact(best, vehicle3SampleWheelRoadContactLegacy(triangles, sampleCenter, radius))
	}
	if best.hit {
		return best
	}
	if downDirection.LengthSquared() == fixed.Zero || sweepDistance.Cmp(fixed.Zero) <= 0 {
		return best
	}
	for _, sampleCenter := range sampleCenters {
		sweep := physics.SweepSphereTriangleMesh(sampleCenter, downDirection.Normalize().Scale(sweepDistance), radius, fixed.One, triangles)
		if !sweep.Hit || sweep.Normal.Y.Cmp(fixed.FromFraction(1, 5)) <= 0 {
			continue
		}
		best = vehicle3AccumulateWheelContact(best, vehicle3WheelRoadContact{
			hit:         true,
			point:       sweep.ContactPoint,
			normal:      sweep.Normal.Normalize(),
			penetration: fixed.Zero,
		})
	}
	return best
}

func vehicle3PersistentWheelRoadContact(previousProbe VehicleWheelProbeState, previousGraceTicks int, center geometry.Vector3, maxDistance fixed.Fixed) vehicle3WheelRoadContact {
	if !previousProbe.Grounded || previousGraceTicks <= 0 || previousProbe.ContactNormal.LengthSquared() == fixed.Zero {
		return vehicle3WheelRoadContact{}
	}
	normal := previousProbe.ContactNormal.Normalize()
	centerDistance := center.Sub(previousProbe.ContactPoint).Dot(normal)
	if centerDistance.Cmp(fixed.Zero) < 0 || centerDistance.Cmp(maxDistance) > 0 {
		return vehicle3WheelRoadContact{}
	}
	projectedPoint := center.Sub(normal.Scale(centerDistance))
	return vehicle3WheelRoadContact{
		hit:         true,
		point:       projectedPoint,
		normal:      normal,
		penetration: fixed.Zero,
	}
}

func vehicle3SampleWheelRoadContactLegacy(triangles []geometry.Triangle, center geometry.Vector3, radius fixed.Fixed) vehicle3WheelRoadContact {
	best := vehicle3WheelRoadContact{}
	for _, triangle := range triangles {
		contact := physics.FindSphereTriangleContact(center, radius, triangle)
		if !contact.Hit {
			continue
		}
		if contact.Normal.Y.Cmp(fixed.FromFraction(1, 5)) <= 0 {
			continue
		}
		if !best.hit || contact.Penetration.Cmp(best.penetration) > 0 || (contact.Penetration == best.penetration && contact.Normal.Y.Cmp(best.normal.Y) > 0) {
			best = vehicle3WheelRoadContact{
				hit:         true,
				point:       contact.Point,
				normal:      contact.Normal.Normalize(),
				penetration: contact.Penetration,
			}
		}
	}
	return best
}

func vehicle3AccumulateWheelContact(best, candidate vehicle3WheelRoadContact) vehicle3WheelRoadContact {
	if !candidate.hit {
		return best
	}
	if !best.hit ||
		candidate.point.Y.Cmp(best.point.Y) > 0 ||
		(candidate.point.Y == best.point.Y && candidate.penetration.Cmp(best.penetration) > 0) ||
		(candidate.point.Y == best.point.Y && candidate.penetration == best.penetration && candidate.normal.Y.Cmp(best.normal.Y) > 0) {
		return candidate
	}
	return best
}

func vehicle3AverageGroundPlane(triangles []geometry.Triangle) (geometry.Vector3, geometry.Vector3) {
	for _, triangle := range triangles {
		normal := triangle.Normal()
		if normal.Y.Cmp(fixed.FromFraction(1, 5)) > 0 {
			return triangle.A, normal.Normalize()
		}
	}
	return geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
}

func vehicle3ChassisSupportPlane(state *SceneState, body *physics.RigidBoxBody3D) (geometry.Vector3, geometry.Vector3) {
	if state != nil && state.VehicleGroundedProbeCount > 0 {
		accumulatedPoint := geometry.ZeroVector3()
		accumulatedNormal := geometry.ZeroVector3()
		count := int64(0)
		for _, probe := range state.VehicleWheelProbes {
			if !probe.Grounded {
				continue
			}
			accumulatedPoint = accumulatedPoint.Add(probe.ContactPoint)
			accumulatedNormal = accumulatedNormal.Add(probe.ContactNormal)
			count++
		}
		if count > 0 {
			invCount := fixed.One.Div(fixed.FromInt(count))
			planePoint := accumulatedPoint.Scale(invCount)
			planeNormal := accumulatedNormal.Scale(invCount).Normalize()
			if planeNormal.LengthSquared() == fixed.Zero {
				planeNormal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
			}
			return planePoint, planeNormal
		}
	}
	if body != nil {
		return body.Motion.Position.Add(geometry.NewVector3(fixed.Zero, fixed.FromInt(-100), fixed.Zero)), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	return geometry.NewVector3(fixed.Zero, fixed.FromInt(-100), fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
}

func vehicle3RotateAroundAxis(vector, axis geometry.Vector3, angle fixed.Fixed) geometry.Vector3 {
	if vector.LengthSquared() == fixed.Zero || axis.LengthSquared() == fixed.Zero || angle.Cmp(fixed.Zero) == 0 {
		return vector
	}

	axis = axis.Normalize()
	cosAngle := fixed.Cos(angle)
	sinAngle := fixed.Sin(angle)
	parallel := axis.Scale(axis.Dot(vector))
	perpendicular := vector.Sub(parallel)
	rotatedPerpendicular := perpendicular.Scale(cosAngle).Add(axis.Cross(perpendicular).Scale(sinAngle))
	return parallel.Add(rotatedPerpendicular)
}

func vehicle3ApplyForceAtRigidBoxPoint(body *physics.RigidBoxBody3D, worldPoint, force geometry.Vector3, dt fixed.Fixed) {
	if body == nil || force.LengthSquared() == fixed.Zero {
		return
	}

	physics.ApplyForce(&body.Motion, force)
	torque := worldPoint.Sub(body.Motion.Position).Cross(force)
	vehicle3ApplyTorqueToRigidBox(body, torque, dt)
}

func vehicle3ApplyTorqueToRigidBox(body *physics.RigidBoxBody3D, torque geometry.Vector3, dt fixed.Fixed) {
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

func vehicle3ApplyUprightAssist(body *physics.RigidBoxBody3D, dt fixed.Fixed, targetUp geometry.Vector3) {
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
		vehicle3ApplyTorqueToRigidBox(body, alignAxis.Scale(fixed.FromInt(12)), dt)
	}

	body.AngularVelocity = geometry.NewVector3(
		body.AngularVelocity.X.Mul(fixed.FromFraction(17, 20)),
		body.AngularVelocity.Y.Mul(fixed.FromFraction(19, 20)),
		body.AngularVelocity.Z.Mul(fixed.FromFraction(17, 20)),
	)
}

func vehicle3ResolveWallContacts(state *SceneState, body *physics.RigidBoxBody3D) bool {
	if state == nil || body == nil || len(state.GroundBoxes) == 0 {
		return false
	}

	bodyBounds := vehicle3RigidBoxWorldBounds(body)
	hadWallContact := false
	for _, bounds := range state.GroundBoxes {
		sizeX := bounds.Max.X.Sub(bounds.Min.X)
		sizeZ := bounds.Max.Z.Sub(bounds.Min.Z)
		if sizeZ.Cmp(sizeX) <= 0 {
			if bodyBounds.Max.X.Cmp(bounds.Min.X) < 0 || bodyBounds.Min.X.Cmp(bounds.Max.X) > 0 ||
				bodyBounds.Max.Y.Cmp(bounds.Min.Y) < 0 || bodyBounds.Min.Y.Cmp(bounds.Max.Y) > 0 {
				continue
			}
			planeZ := bounds.Min.Z
			planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
			planeNormal := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())
			if vehicle2FixedAbs(body.Motion.Position.Z.Sub(bounds.Max.Z)).Cmp(vehicle2FixedAbs(body.Motion.Position.Z.Sub(bounds.Min.Z))) < 0 {
				planeZ = bounds.Max.Z
				planePoint = geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
				planeNormal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
			}
			beforePosition := body.Motion.Position
			if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
				vehicle3ClampWallCorrection(body, beforePosition, fixed.FromFraction(3, 20))
				body.Motion.Velocity = geometry.NewVector3(
					body.Motion.Velocity.X.Mul(fixed.FromFraction(4, 5)),
					body.Motion.Velocity.Y,
					body.Motion.Velocity.Z.Mul(fixed.FromFraction(4, 5)),
				)
				state.LastContact = result.LastContact
				state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
				state.VehicleEverHitWall = true
				hadWallContact = true
			}
			continue
		}

		if bodyBounds.Max.Z.Cmp(bounds.Min.Z) < 0 || bodyBounds.Min.Z.Cmp(bounds.Max.Z) > 0 ||
			bodyBounds.Max.Y.Cmp(bounds.Min.Y) < 0 || bodyBounds.Min.Y.Cmp(bounds.Max.Y) > 0 {
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
		beforePosition := body.Motion.Position
		if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
			vehicle3ClampWallCorrection(body, beforePosition, fixed.FromFraction(3, 20))
			body.Motion.Velocity = geometry.NewVector3(
				body.Motion.Velocity.X.Mul(fixed.FromFraction(4, 5)),
				body.Motion.Velocity.Y,
				body.Motion.Velocity.Z.Mul(fixed.FromFraction(4, 5)),
			)
			state.LastContact = result.LastContact
			state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
			state.VehicleEverHitWall = true
			hadWallContact = true
		}
	}
	return hadWallContact
}

func vehicle3RigidBoxWorldBounds(body *physics.RigidBoxBody3D) geometry.AxisAlignedBoundingBox {
	if body == nil {
		return geometry.AxisAlignedBoundingBox{}
	}
	corners := body.WorldCorners()
	if len(corners) == 0 {
		return geometry.AxisAlignedBoundingBox{}
	}

	minCorner := corners[0]
	maxCorner := corners[0]
	for _, corner := range corners[1:] {
		if corner.X.Cmp(minCorner.X) < 0 {
			minCorner.X = corner.X
		}
		if corner.Y.Cmp(minCorner.Y) < 0 {
			minCorner.Y = corner.Y
		}
		if corner.Z.Cmp(minCorner.Z) < 0 {
			minCorner.Z = corner.Z
		}
		if corner.X.Cmp(maxCorner.X) > 0 {
			maxCorner.X = corner.X
		}
		if corner.Y.Cmp(maxCorner.Y) > 0 {
			maxCorner.Y = corner.Y
		}
		if corner.Z.Cmp(maxCorner.Z) > 0 {
			maxCorner.Z = corner.Z
		}
	}
	return geometry.NewAxisAlignedBoundingBox(minCorner, maxCorner)
}

func vehicle3ClampWallCorrection(body *physics.RigidBoxBody3D, previousPosition geometry.Vector3, maxCorrection fixed.Fixed) {
	if body == nil || maxCorrection.Cmp(fixed.Zero) <= 0 {
		return
	}
	delta := body.Motion.Position.Sub(previousPosition)
	length := delta.Length()
	if length.Cmp(maxCorrection) <= 0 || length == fixed.Zero {
		return
	}
	body.Motion.Position = previousPosition.Add(delta.Normalize().Scale(maxCorrection))
}

func vehicle3BodyPositionZ(state *SceneState) fixed.Fixed {
	if state == nil {
		return fixed.Zero
	}
	return state.VehicleChassis.Motion.Position.Z
}

func makeVehicle3RayCityMapTriangles() []geometry.Triangle {
	triangles := make([]geometry.Triangle, 0, 4096)

	const (
		minX = -120
		maxX = 120
		minZ = -120
		maxZ = 220
		step = 8
	)

	for z := minZ; z < maxZ; z += step {
		for x := minX; x < maxX; x += step {
			a := geometry.NewVector3(fixed.FromInt(int64(x)), vehicle3TerrainHeight(x, z), fixed.FromInt(int64(z)))
			b := geometry.NewVector3(fixed.FromInt(int64(x+step)), vehicle3TerrainHeight(x+step, z), fixed.FromInt(int64(z)))
			c := geometry.NewVector3(fixed.FromInt(int64(x)), vehicle3TerrainHeight(x, z+step), fixed.FromInt(int64(z+step)))
			d := geometry.NewVector3(fixed.FromInt(int64(x+step)), vehicle3TerrainHeight(x+step, z+step), fixed.FromInt(int64(z+step)))
			triangles = append(triangles,
				geometry.NewTriangle(a, c, b),
				geometry.NewTriangle(c, d, b),
			)
		}
	}

	triangles = append(triangles, makeVehicle3RoadDeckTriangles()...)
	triangles = append(triangles, makeVehicle3BridgeSupportTriangles()...)
	triangles = append(triangles, makeVehicle3BuildingTriangles()...)
	return triangles
}

func makeVehicle3RayCityMapBoxes() []geometry.AxisAlignedBoundingBox {
	return makeVehicle3BuildingBoxes()
}

func vehicle3TerrainHeight(x, z int) fixed.Fixed {
	roadHalfWidth := 9
	shoulderWidth := 18
	absX := x
	if absX < 0 {
		absX = -absX
	}

	terrain := vehicle3NoiseHeight(x, z)
	terrain = terrain.Add(vehicle3ValleyShape(x, z))

	roadHeight := vehicle3RoadHeight(z)
	if absX <= roadHalfWidth {
		return roadHeight
	}
	if absX <= shoulderWidth {
		blendNum := int64(absX - roadHalfWidth)
		blendDen := int64(shoulderWidth - roadHalfWidth)
		blend := fixed.FromFraction(blendNum, blendDen)
		return roadHeight.Mul(fixed.One.Sub(blend)).Add(terrain.Mul(blend))
	}
	return terrain
}

func vehicle3RoadHeight(z int) fixed.Fixed {
	switch {
	case z < -40:
		return fixed.Zero
	case z < 10:
		return fixed.FromFraction(int64(z+40), 25)
	case z < 90:
		return fixed.FromInt(2)
	case z < 130:
		return fixed.FromInt(4)
	case z < 170:
		return fixed.FromInt(4).Sub(fixed.FromFraction(int64(z-130), 20))
	default:
		return fixed.FromInt(2).Add(fixed.FromFraction(int64(z-170), 50))
	}
}

func vehicle3NoiseHeight(x, z int) fixed.Fixed {
	n0 := vehicle3ValueNoise2D(x, z, 40).Mul(fixed.FromFraction(11, 5))
	n1 := vehicle3ValueNoise2D(x+17, z-31, 20).Mul(fixed.FromFraction(7, 10))
	n2 := vehicle3ValueNoise2D(x-53, z+11, 64).Mul(fixed.FromFraction(8, 5))
	return n0.Add(n1).Add(n2)
}

func vehicle3ValleyShape(x, z int) fixed.Fixed {
	if z < 88 || z > 136 {
		return fixed.Zero
	}
	depth := fixed.FromFraction(9, 2)
	centerX := x
	if centerX < 0 {
		centerX = -centerX
	}
	widthFalloff := fixed.FromFraction(int64(centerX), 28)
	if widthFalloff.Cmp(fixed.One) > 0 {
		widthFalloff = fixed.One
	}
	return depth.Neg().Mul(fixed.One.Sub(widthFalloff))
}

func vehicle3ValueNoise2D(x, z, cellSize int) fixed.Fixed {
	if cellSize <= 0 {
		return fixed.Zero
	}
	x0 := floorDiv(x, cellSize)
	z0 := floorDiv(z, cellSize)
	x1 := x0 + 1
	z1 := z0 + 1

	fx := fixed.FromFraction(int64(x-x0*cellSize), int64(cellSize))
	fz := fixed.FromFraction(int64(z-z0*cellSize), int64(cellSize))

	v00 := vehicle3HashNoise(x0, z0)
	v10 := vehicle3HashNoise(x1, z0)
	v01 := vehicle3HashNoise(x0, z1)
	v11 := vehicle3HashNoise(x1, z1)

	top := v00.Mul(fixed.One.Sub(fx)).Add(v10.Mul(fx))
	bottom := v01.Mul(fixed.One.Sub(fx)).Add(v11.Mul(fx))
	return top.Mul(fixed.One.Sub(fz)).Add(bottom.Mul(fz))
}

func vehicle3HashNoise(x, z int) fixed.Fixed {
	n := int64(x*374761393 + z*668265263 + 1442695040888963407)
	n = (n ^ (n >> 13)) * 1274126177
	n = n ^ (n >> 16)
	value := n & 1023
	return fixed.FromFraction(value, 1023).Mul(fixed.FromInt(2)).Sub(fixed.One)
}

func floorDiv(value, divisor int) int {
	if divisor == 0 {
		return 0
	}
	if value >= 0 {
		return value / divisor
	}
	return -((-value + divisor - 1) / divisor)
}

func makeVehicle3RoadDeckTriangles() []geometry.Triangle {
	triangles := make([]geometry.Triangle, 0, 128)
	for z := -80; z < 200; z += 8 {
		startZ := fixed.FromInt(int64(z))
		endZ := fixed.FromInt(int64(z + 8))
		startY := vehicle3RoadHeight(z)
		endY := vehicle3RoadHeight(z + 8)
		triangles = append(triangles, makeVehicle3RoadStripTriangles(fixed.FromInt(-8), fixed.FromInt(8), startZ, startY, endZ, endY)...)
	}
	return triangles
}

func makeVehicle3BridgeSupportTriangles() []geometry.Triangle {
	supports := []geometry.AxisAlignedBoundingBox{
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromFraction(-3, 2), fixed.FromInt(-6), fixed.FromInt(96)),
			geometry.NewVector3(fixed.FromFraction(3, 2), fixed.FromInt(4), fixed.FromInt(102)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromFraction(-3, 2), fixed.FromInt(-6), fixed.FromInt(116)),
			geometry.NewVector3(fixed.FromFraction(3, 2), fixed.FromInt(4), fixed.FromInt(122)),
		),
	}

	triangles := make([]geometry.Triangle, 0, len(supports)*12)
	for _, support := range supports {
		triangles = append(triangles, makeVehicle2WallTrianglesFromBounds(support)...)
	}
	return triangles
}

func makeVehicle3BuildingBoxes() []geometry.AxisAlignedBoundingBox {
	return []geometry.AxisAlignedBoundingBox{
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(-36), fixed.Zero, fixed.FromInt(-20)),
			geometry.NewVector3(fixed.FromInt(-24), fixed.FromInt(8), fixed.FromInt(-6)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(22), fixed.Zero, fixed.FromInt(8)),
			geometry.NewVector3(fixed.FromInt(34), fixed.FromInt(10), fixed.FromInt(22)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(-42), fixed.Zero, fixed.FromInt(54)),
			geometry.NewVector3(fixed.FromInt(-28), fixed.FromInt(12), fixed.FromInt(70)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(28), fixed.Zero, fixed.FromInt(76)),
			geometry.NewVector3(fixed.FromInt(44), fixed.FromInt(14), fixed.FromInt(92)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(-34), fixed.Zero, fixed.FromInt(138)),
			geometry.NewVector3(fixed.FromInt(-20), fixed.FromInt(11), fixed.FromInt(154)),
		),
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromInt(24), fixed.Zero, fixed.FromInt(168)),
			geometry.NewVector3(fixed.FromInt(40), fixed.FromInt(9), fixed.FromInt(186)),
		),
	}
}

func makeVehicle3BuildingTriangles() []geometry.Triangle {
	boxes := makeVehicle3BuildingBoxes()
	triangles := make([]geometry.Triangle, 0, len(boxes)*12)
	for _, box := range boxes {
		triangles = append(triangles, makeVehicle2WallTrianglesFromBounds(box)...)
	}
	return triangles
}

func makeVehicle3RoadStripTriangles(minX, maxX, startZ, startY, endZ, endY fixed.Fixed) []geometry.Triangle {
	a := geometry.NewVector3(minX, startY, startZ)
	b := geometry.NewVector3(maxX, startY, startZ)
	c := geometry.NewVector3(minX, endY, endZ)
	d := geometry.NewVector3(maxX, endY, endZ)
	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}
