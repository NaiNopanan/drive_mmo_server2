package physics

import "server2/pkg/geom"

const ccdStartOverlapEpsilon float32 = 0.0001
const ccdStartOverlapPenetrationThreshold float32 = 0.05
const bodyFloorLikeNormalThreshold float32 = 0.6

func (w *PhysicsWorld) applyBodyCapsuleCCDWithSlide(previous, current VehicleBody, dt float32) VehicleBody {
	ccdHit, ccdIntersects := w.queryBodyCapsuleMapCCD(previous, current)
	if !ccdIntersects {
		return current
	}
	if ccdHit.Time <= ccdStartOverlapEpsilon {
		startHit, startIntersects := w.queryBodyCapsuleMapHit(previous)
		if !startIntersects || startHit.Penetration <= ccdStartOverlapPenetrationThreshold {
			// A tiny initial overlap is often just contact skin / numerical noise.
			// Keep the normal CCD+slide path for those shallow contacts.
			if shouldIgnoreCCDStartOverlap(current, ccdHit, startHit, startIntersects) {
				return current
			}
		} else {
			// Start-overlap means the previous pose is already intersecting.
			// Fall back to depenetrating from that earlier pose instead of letting the
			// body advance deeper into geometry before the depenetration loop runs.
			current.Position = previous.Position
			current.Height = previous.Height
			current.Heading = previous.Heading
			current.Pitch = previous.Pitch
			current.Roll = previous.Roll
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
			current.BodyHitMap = true
			return current
		}
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
	current = applyCCDContactSkin(current, ccdHit.Normal, bodyCapsuleCCDSkin)
	current.BodyHitMap = true
	current = slideBodyVelocityAgainstNormal(current, ccdHit.Normal)

	remainingDT := dt * (1 - ccdHit.Time)
	if remainingDT <= 0 {
		return current
	}

	advanced := advanceVehicleWithRemainingTime(current, remainingDT)
	secondHit, secondIntersects := w.queryBodyCapsuleMapCCD(current, advanced)
	if !secondIntersects {
		return advanced
	}

	advanced = moveBodyToCCDContact(current, advanced, secondHit)
	advanced = applyCCDContactSkin(advanced, secondHit.Normal, bodyCapsuleCCDSkin)
	advanced.BodyHitMap = true
	advanced = slideBodyVelocityAgainstNormal(advanced, secondHit.Normal)
	return advanced
}

func (w *PhysicsWorld) applyBodyOBBCCDWithSlide(previous, current VehicleBody, dt float32) VehicleBody {
	return w.applyBodyCapsuleCCDWithSlide(previous, current, dt)
}

func shouldIgnoreCCDStartOverlap(vehicle VehicleBody, ccdHit bodyMapCCDHit, startHit bodyMapHit, startIntersects bool) bool {
	if !startIntersects {
		return false
	}
	if ccdHit.Normal.Y < bodyFloorLikeNormalThreshold {
		return false
	}
	if vehicle.SupportState == SupportStateFalling || vehicle.SupportHits < 2 {
		return false
	}
	if startHit.Penetration > ccdStartOverlapPenetrationThreshold {
		return false
	}
	return true
}

func moveBodyToCCDContact(previous, current VehicleBody, hit bodyMapCCDHit) VehicleBody {
	return interpolateVehiclePose(previous, current, hit.Time)
}

func interpolateVehiclePose(previous, current VehicleBody, t float32) VehicleBody {
	current.Position = geom.LerpPlanar(previous.Position, current.Position, t)
	current.Height = lerpFloat32(previous.Height, current.Height, t)
	current.Heading = lerpAngle(previous.Heading, current.Heading, t)
	current.Pitch = lerpFloat32(previous.Pitch, current.Pitch, t)
	current.Roll = lerpFloat32(previous.Roll, current.Roll, t)
	return current
}

func applyCCDContactSkin(vehicle VehicleBody, normal geom.Vec3, skin float32) VehicleBody {
	vehicle.Position.X += normal.X * skin
	vehicle.Position.Z += normal.Z * skin
	vehicle.Height += normal.Y * skin
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
