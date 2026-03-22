package physics

import "server2/pkg/geom"

const (
	vehicleKinematicSubsteps           = 4
	vehicleCollisionIterations         = 6
	vehicleCollisionSkin       float32 = 0.001
	vehicleGroundNormalMinY    float32 = 0.35
	vehicleGroundSnapDistance  float32 = 0.35
	vehicleGroundStickSpeed    float32 = 2.4
)

type kinematicContactState struct {
	Grounded      bool
	GroundHeight  float32
	SupportHits   int
	ContactCount  int
	ContactNormal geom.Vec3
	GroundNormal  geom.Vec3
}

type groundProbeResult struct {
	Hit    bool
	Origin geom.Vec3
	Point  geom.Vec3
	Normal geom.Vec3
}

func (w *PhysicsWorld) stepVehicleKinematic(vehicle VehicleBody, input DriveInput, dt float32) VehicleBody {
	subDT := dt / vehicleKinematicSubsteps
	grounded := vehicle.SupportState != SupportStateFalling
	contactState := kinematicContactState{
		Grounded:     grounded,
		GroundHeight: vehicle.GroundHeight,
		SupportHits:  vehicle.SupportHits,
	}
	debug := KinematicDebug{
		Grounded: grounded,
		Substeps: vehicleKinematicSubsteps,
		Throttle: input.Throttle,
		Brake:    input.Brake,
		Steering: input.Steering,
	}
	groundNormal := vehicle.Kinematic.GroundNormal
	if groundNormal.LengthSquared() == 0 {
		groundNormal = geom.V3(0, 1, 0)
	}

	for step := 0; step < vehicleKinematicSubsteps; step++ {
		vehicle = applyVehicleControls(vehicle, input, subDT)
		forward, right, _ := vehicleAxes(vehicle.Heading, 0, 0)
		debug.ForwardVector = forward
		debug.RightVector = right

		moveVelocity := geom.V3(vehicle.Velocity.X, 0, vehicle.Velocity.Z)
		debug.PlanarVelocity = moveVelocity
		if grounded {
			if lookAheadProbe := w.probeVehicleGround(vehicle, vehicle.Params.BodyLength*0.45); lookAheadProbe.Hit {
				debug.LookAheadHit = true
				debug.LookAheadOrigin = lookAheadProbe.Origin
				debug.LookAheadPoint = lookAheadProbe.Point
				debug.LookAheadNormal = lookAheadProbe.Normal
				groundNormal = lookAheadProbe.Normal
			}
			moveVelocity = projectVelocityOnPlane(moveVelocity, groundNormal)
			vehicle.VerticalVel = -vehicleGroundStickSpeed
		} else {
			vehicle.VerticalVel -= spawnDropGravity * subDT
			groundNormal = geom.V3(0, 1, 0)
		}

		moveDelta := moveVelocity.MulScalar(subDT).Add(geom.V3(0, vehicle.VerticalVel*subDT, 0))
		debug.ProjectedVelocity = moveVelocity
		debug.MoveDelta = moveDelta
		vehicle, contactState = w.moveVehicleKinematic(vehicle, moveDelta)
		if snappedVehicle, snappedState, snapped := w.trySnapVehicleToGround(vehicle); snapped && snappedVehicle.Height >= vehicle.Height {
			vehicle = snappedVehicle
			contactState.Grounded = snappedState.Grounded
			contactState.GroundHeight = snappedState.GroundHeight
			if snappedState.SupportHits > contactState.SupportHits {
				contactState.SupportHits = snappedState.SupportHits
			}
			if snappedState.GroundNormal.LengthSquared() > 0 {
				contactState.GroundNormal = snappedState.GroundNormal
			}
			debug.SnapApplied = true
		}
		grounded = contactState.Grounded
		debug.Grounded = grounded
		debug.ContactCount += contactState.ContactCount
		if contactState.ContactNormal.LengthSquared() > 0 {
			debug.ContactNormal = contactState.ContactNormal
		}
		if contactState.GroundNormal.LengthSquared() > 0 {
			debug.GroundNormal = contactState.GroundNormal
			groundNormal = contactState.GroundNormal
		}
		if groundProbe := w.probeVehicleGround(vehicle, 0); groundProbe.Hit {
			debug.GroundProbeHit = true
			debug.GroundProbeOrigin = groundProbe.Origin
			debug.GroundProbePoint = groundProbe.Point
			debug.GroundProbeNormal = groundProbe.Normal
		}
		if grounded {
			vehicle.VerticalVel = 0
		}

		vehicle = w.applyWorldConstraints(vehicle)
	}

	if contactState.Grounded {
		vehicle.SupportState = SupportStateStable
		vehicle.SupportHits = contactState.SupportHits
		if vehicle.SupportHits < 1 {
			vehicle.SupportHits = 1
		}
		vehicle.GroundHeight = contactState.GroundHeight
	} else {
		vehicle.SupportState = SupportStateFalling
		vehicle.SupportHits = 0
		vehicle.GroundHeight = vehicle.Height
	}

	vehicle = applyVehicleGroundTilt(vehicle, contactState)
	vehicle.Wheels = wheelDebugSnapshot(vehicle)
	vehicle.Speed = vehicle.Velocity.Length()
	vehicle.Kinematic = debug
	return vehicle
}

func (w *PhysicsWorld) moveVehicleKinematic(vehicle VehicleBody, delta geom.Vec3) (VehicleBody, kinematicContactState) {
	vehicle.Position = vehicle.Position.Add(geom.Planar(delta.X, delta.Z))
	vehicle.Height += delta.Y

	contactState := kinematicContactState{
		GroundHeight: vehicle.Height,
	}
	groundNormal := geom.Vec3{}

	for iteration := 0; iteration < vehicleCollisionIterations; iteration++ {
		hit, intersects := w.queryBodyCapsuleMapHit(vehicle)
		if !intersects {
			break
		}

		push := hit.Penetration + vehicleCollisionSkin
		vehicle.Position.X += hit.Normal.X * push
		vehicle.Position.Z += hit.Normal.Z * push
		vehicle.Height += hit.Normal.Y * push
		vehicle.BodyHitMap = true
		contactState.ContactCount++
		contactState.ContactNormal = hit.Normal

		if hit.Normal.Y >= vehicleGroundNormalMinY {
			contactState.Grounded = true
			contactState.SupportHits++
			contactState.GroundHeight = maxf(contactState.GroundHeight, vehicle.Height)
			groundNormal = groundNormal.Add(hit.Normal)
			if vehicle.VerticalVel < 0 {
				vehicle.VerticalVel = 0
			}
		} else {
			vehicle = slideBodyVelocityAgainstNormal(vehicle, hit.Normal)
		}
	}

	if groundNormal.LengthSquared() > 0 {
		contactState.GroundNormal = groundNormal.Normalize()
	}
	if !contactState.Grounded && groundNormal.LengthSquared() == 0 {
		snappedVehicle, snappedState, snapped := w.trySnapVehicleToGround(vehicle)
		if snapped {
			vehicle = snappedVehicle
			contactState = snappedState
		}
	}

	return vehicle, contactState
}

func (w *PhysicsWorld) trySnapVehicleToGround(vehicle VehicleBody) (VehicleBody, kinematicContactState, bool) {
	groundProbe := w.probeVehicleGround(vehicle, 0)
	if !groundProbe.Hit || groundProbe.Normal.Y < vehicleGroundNormalMinY {
		return vehicle, kinematicContactState{}, false
	}

	targetHeight := groundProbe.Point.Y
	heightDelta := targetHeight - vehicle.Height
	if heightDelta > 0 {
		vehicle.Height = targetHeight
		return vehicle, kinematicContactState{
			Grounded:     true,
			GroundHeight: vehicle.Height,
			SupportHits:  1,
			GroundNormal: groundProbe.Normal,
		}, true
	}
	if absf(heightDelta) > vehicleGroundSnapDistance*2 {
		return vehicle, kinematicContactState{}, false
	}

	vehicle.Height = targetHeight
	return vehicle, kinematicContactState{
		Grounded:     true,
		GroundHeight: vehicle.Height,
		SupportHits:  1,
		GroundNormal: groundProbe.Normal,
	}, true
}

func applyVehicleGroundTilt(vehicle VehicleBody, contactState kinematicContactState) VehicleBody {
	targetPitch := float32(0)
	targetRoll := float32(0)
	smooth := float32(0.28)

	if contactState.Grounded && contactState.GroundNormal.LengthSquared() > 0 {
		targetPitch, targetRoll = tiltFromNormal(vehicle.Heading, contactState.GroundNormal.Normalize())
		targetPitch = clamp(targetPitch, -0.6, 0.6)
		targetRoll = clamp(targetRoll, -0.6, 0.6)
		smooth = 0.35
	}

	vehicle.Pitch += (targetPitch - vehicle.Pitch) * smooth
	vehicle.Roll += (targetRoll - vehicle.Roll) * smooth
	return vehicle
}

func (w *PhysicsWorld) probeVehicleGround(vehicle VehicleBody, forwardOffset float32) groundProbeResult {
	probePosition := vehicle.Position.Add(geom.FromHeading(vehicle.Heading).MulScalar(forwardOffset))
	originY := vehicle.Height + vehicle.Params.BodyHeight + vehicleGroundSnapDistance
	origin := geom.V3(
		probePosition.X,
		originY,
		probePosition.Z,
	)
	maxDistance := vehicle.Params.BodyHeight + vehicleGroundSnapDistance*4
	hit := w.queryGroundHit(origin, maxDistance)
	if !hit.Hit {
		return groundProbeResult{
			Origin: origin,
		}
	}

	return groundProbeResult{
		Hit:    true,
		Origin: origin,
		Point:  hit.Point,
		Normal: hit.Normal.Normalize(),
	}
}

func projectVelocityOnPlane(velocity geom.Vec3, normal geom.Vec3) geom.Vec3 {
	if normal.LengthSquared() == 0 {
		return velocity
	}

	normal = normal.Normalize()
	return velocity.Sub(normal.MulScalar(velocity.Dot(normal)))
}

func wheelDebugSnapshot(vehicle VehicleBody) [wheelCount]WheelState {
	suspension := vehicle.Params.Suspension
	localOffsets := wheelLocalOffsets(suspension)
	wheelLabels := [wheelCount]string{"FL", "FR", "RL", "RR"}
	wheelCenterY := maxf(suspension.WheelRadius, suspension.MountHeight-suspension.RestLength)

	var wheels [wheelCount]WheelState
	for index, localOffset := range localOffsets {
		wheels[index] = WheelState{
			Label:       wheelLabels[index],
			MountPoint:  supportMountPoint(vehicle, localOffset.X, suspension.MountHeight, localOffset.Z),
			WheelCenter: supportMountPoint(vehicle, localOffset.X, wheelCenterY, localOffset.Z),
		}
	}

	return wheels
}
