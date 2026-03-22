package physics

import "server2/pkg/geom"

func (w *PhysicsWorld) applyBodyOBBCCDWithSlide(previous, current VehicleBody, dt float32) VehicleBody {
	ccdHit, ccdIntersects := w.queryBodyOBBMapCCD(previous, current)
	if !ccdIntersects {
		return current
	}

	current = moveBodyToCCDContact(previous, current, ccdHit)
	current.OBBCCD = OBBCCDDebug{
		Hit:      true,
		Time:     ccdHit.Time,
		Position: current.Position,
		Height:   current.Height,
		Heading:  current.Heading,
		Pitch:    current.Pitch,
		Roll:     current.Roll,
		Normal:   ccdHit.Normal,
	}
	current = applyCCDContactSkin(current, ccdHit.Normal)
	current.BodyHitMap = true
	current = slideBodyVelocityAgainstNormal(current, ccdHit.Normal)

	remainingDT := dt * (1 - ccdHit.Time)
	if remainingDT <= 0 {
		return current
	}

	advanced := advanceVehicleWithRemainingTime(current, remainingDT)
	secondHit, secondIntersects := w.queryBodyOBBMapCCD(current, advanced)
	if !secondIntersects {
		return advanced
	}

	advanced = moveBodyToCCDContact(current, advanced, secondHit)
	advanced = applyCCDContactSkin(advanced, secondHit.Normal)
	advanced.BodyHitMap = true
	advanced = slideBodyVelocityAgainstNormal(advanced, secondHit.Normal)
	return advanced
}

func moveBodyToCCDContact(previous, current VehicleBody, hit bodyMapCCDHit) VehicleBody {
	current.Position = geom.LerpPlanar(previous.Position, current.Position, hit.Time)
	current.Height = lerpFloat32(previous.Height, current.Height, hit.Time)
	current.Heading = lerpAngle(previous.Heading, current.Heading, hit.Time)
	current.Pitch = lerpFloat32(previous.Pitch, current.Pitch, hit.Time)
	current.Roll = lerpFloat32(previous.Roll, current.Roll, hit.Time)
	return current
}

func applyCCDContactSkin(vehicle VehicleBody, normal geom.Vec3) VehicleBody {
	vehicle.Position.X += normal.X * bodyOBBCCDSkin
	vehicle.Position.Z += normal.Z * bodyOBBCCDSkin
	vehicle.Height += normal.Y * bodyOBBCCDSkin
	return vehicle
}

func slideBodyVelocityAgainstNormal(vehicle VehicleBody, normal geom.Vec3) VehicleBody {
	velocity3D := geom.V3(vehicle.Velocity.X, vehicle.VerticalVel, vehicle.Velocity.Z)
	slidingVelocity := slideVelocityAgainstNormal(velocity3D, normal)
	vehicle.Velocity = geom.Planar(slidingVelocity.X, slidingVelocity.Z)
	vehicle.VerticalVel = slidingVelocity.Y
	vehicle.Speed = vehicle.Velocity.Length()
	return vehicle
}

func slideVelocityAgainstNormal(velocity geom.Vec3, normal geom.Vec3) geom.Vec3 {
	if normal.LengthSquared() <= obbTriangleEpsilon {
		return velocity
	}

	normal = normal.Normalize()
	intoSurface := velocity.Dot(normal)
	if intoSurface >= 0 {
		return velocity
	}

	return velocity.Sub(normal.MulScalar(intoSurface))
}

func advanceVehicleWithRemainingTime(vehicle VehicleBody, remainingDT float32) VehicleBody {
	if remainingDT <= 0 {
		return vehicle
	}

	vehicle.Position = vehicle.Position.Add(vehicle.Velocity.MulScalar(remainingDT))
	vehicle.Height += vehicle.VerticalVel * remainingDT
	vehicle.Heading = normalizeAngle(vehicle.Heading + vehicle.YawRate*remainingDT)
	vehicle.Speed = vehicle.Velocity.Length()
	return vehicle
}
