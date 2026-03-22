package physics

import "server2/pkg/geom"

const (
	vehicleKinematicSubsteps           = 4
	vehicleCollisionIterations         = 6
	vehicleCollisionSkin       float32 = 0.001
	vehicleGroundNormalMinY    float32 = 0.55
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
	}

	for step := 0; step < vehicleKinematicSubsteps; step++ {
		vehicle = applyVehicleControls(vehicle, input, subDT)

		if grounded {
			vehicle.VerticalVel = -vehicleGroundStickSpeed
		} else {
			vehicle.VerticalVel -= spawnDropGravity * subDT
		}

		moveDelta := geom.V3(
			vehicle.Velocity.X*subDT,
			vehicle.VerticalVel*subDT,
			vehicle.Velocity.Z*subDT,
		)
		vehicle, contactState = w.moveVehicleKinematic(vehicle, moveDelta)
		grounded = contactState.Grounded
		debug.Grounded = grounded
		debug.ContactCount += contactState.ContactCount
		if contactState.ContactNormal.LengthSquared() > 0 {
			debug.ContactNormal = contactState.ContactNormal
		}
		if contactState.GroundNormal.LengthSquared() > 0 {
			debug.GroundNormal = contactState.GroundNormal
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

	vehicle.Pitch = 0
	vehicle.Roll = 0
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
		vehicle = slideBodyVelocityAgainstNormal(vehicle, hit.Normal)
		vehicle.BodyHitMap = true
		contactState.ContactCount++
		contactState.ContactNormal = hit.Normal

		if hit.Normal.Y >= vehicleGroundNormalMinY {
			contactState.Grounded = true
			contactState.SupportHits++
			contactState.GroundHeight = maxf(contactState.GroundHeight, vehicle.Height)
			groundNormal = groundNormal.Add(hit.Normal)
		}
	}

	if groundNormal.LengthSquared() > 0 {
		contactState.GroundNormal = groundNormal.Normalize()
	}
	if !contactState.Grounded && groundNormal.LengthSquared() == 0 {
		supportHit, intersects := w.queryBodyCapsuleMapHit(vehicle)
		if intersects && supportHit.Normal.Y >= vehicleGroundNormalMinY {
			contactState.Grounded = true
			contactState.SupportHits = 1
			contactState.GroundHeight = vehicle.Height
			contactState.GroundNormal = supportHit.Normal.Normalize()
		}
	}

	return vehicle, contactState
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
