package physics

import (
	"math"

	"server2/pkg/geom"
)

const (
	vehicleKinematicSubsteps           = 4
	vehicleCollisionIterations         = 6
	vehicleCollisionSkin       float32 = 0.001
	vehicleGroundNormalMinY    float32 = 0.35
	vehicleGroundSnapDistance  float32 = 0.35
	vehicleGroundStickSpeed    float32 = 2.4
	vehicleSupportBlend                = 0.45
	vehicleTiltStep            float32 = 0.04
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

type supportPlaneResult struct {
	Valid        bool
	Center       groundProbeResult
	Front        groundProbeResult
	Rear         groundProbeResult
	Left         groundProbeResult
	Right        groundProbeResult
	Pitch        float32
	Roll         float32
	HasPitch     bool
	HasRoll      bool
	Normal       geom.Vec3
	GroundHeight float32
}

func (w *PhysicsWorld) stepVehicleKinematic(vehicle VehicleBody, input DriveInput, dt float32) VehicleBody {
	subDT := dt / vehicleKinematicSubsteps
	grounded := vehicle.SupportState != SupportStateFalling
	startedGrounded := grounded
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
		groundProbe := groundProbeResult{}
		if grounded {
			if supportPlane := w.probeVehicleSupportPlane(vehicle); supportPlane.Valid {
				groundProbe = supportPlane.Center
				groundNormal = blendGroundNormal(groundNormal, supportPlane.Normal, vehicleSupportBlend)

				if supportPlane.Front.Hit {
					debug.LookAheadHit = true
					debug.LookAheadOrigin = supportPlane.Front.Origin
					debug.LookAheadPoint = supportPlane.Front.Point
					debug.LookAheadNormal = supportPlane.Front.Normal
				}
			} else if supportProbe := w.probeVehicleGround(vehicle, 0); supportProbe.Hit {
				groundProbe = supportProbe
				groundNormal = blendGroundNormal(groundNormal, supportProbe.Normal, vehicleSupportBlend)
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
		if !groundProbe.Hit {
			groundProbe = w.probeVehicleGround(vehicle, 0)
		}
		if groundProbe.Hit {
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

	finalSupportPlane := w.probeVehicleSupportPlane(vehicle)
	if finalSupportPlane.Valid {
		contactState.Grounded = true
		resolvedNormal := finalSupportPlane.Normal
		if contactState.GroundNormal.LengthSquared() > 0 {
			resolvedNormal = blendGroundNormal(contactState.GroundNormal, finalSupportPlane.Normal, 0.35)
		}
		contactState.GroundNormal = resolvedNormal
		contactState.GroundHeight = maxf(contactState.GroundHeight, finalSupportPlane.GroundHeight)
		if contactState.SupportHits < 1 {
			contactState.SupportHits = 1
		}
		debug.Grounded = true
		debug.GroundNormal = resolvedNormal
		if finalSupportPlane.Center.Hit {
			debug.GroundProbeHit = true
			debug.GroundProbeOrigin = finalSupportPlane.Center.Origin
			debug.GroundProbePoint = finalSupportPlane.Center.Point
			debug.GroundProbeNormal = finalSupportPlane.Center.Normal
		}
	}

	if !contactState.Grounded && startedGrounded {
		heightAboveGround := vehicle.Height - vehicle.GroundHeight
		if heightAboveGround <= vehicleGroundSnapDistance*1.5 && groundNormal.LengthSquared() > 0 && groundNormal.Y >= vehicleGroundNormalMinY {
			contactState.Grounded = true
			contactState.GroundNormal = groundNormal.Normalize()
			contactState.GroundHeight = maxf(contactState.GroundHeight, vehicle.GroundHeight)
			if contactState.SupportHits < 1 {
				contactState.SupportHits = 1
			}
			debug.Grounded = true
			debug.GroundNormal = contactState.GroundNormal
		}
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

	vehicle = applyVehicleGroundTilt(vehicle, contactState, finalSupportPlane)
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
	targetHeight := float32(0)
	targetNormal := geom.Vec3{}
	hasSupport := false

	if supportPlane := w.probeVehicleSupportPlane(vehicle); supportPlane.Valid && supportPlane.Normal.Y >= vehicleGroundNormalMinY {
		targetHeight = supportPlane.GroundHeight
		targetNormal = supportPlane.Normal
		hasSupport = true
		if supportPlane.Center.Hit {
			targetHeight = supportPlane.Center.Point.Y
		}
	}

	if groundProbe.Hit && groundProbe.Normal.Y >= vehicleGroundNormalMinY {
		targetHeight = groundProbe.Point.Y
		targetNormal = groundProbe.Normal
		hasSupport = true
	}

	if !hasSupport {
		return vehicle, kinematicContactState{}, false
	}

	heightDelta := targetHeight - vehicle.Height
	if heightDelta > 0 {
		vehicle.Height = targetHeight
		return vehicle, kinematicContactState{
			Grounded:     true,
			GroundHeight: vehicle.Height,
			SupportHits:  1,
			GroundNormal: targetNormal,
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
		GroundNormal: targetNormal,
	}, true
}

func applyVehicleGroundTilt(vehicle VehicleBody, contactState kinematicContactState, supportPlane supportPlaneResult) VehicleBody {
	targetPitch := float32(0)
	targetRoll := float32(0)
	smooth := float32(0.22)

	if contactState.Grounded && contactState.GroundNormal.LengthSquared() > 0 {
		targetPitch, targetRoll = tiltFromNormal(vehicle.Heading, contactState.GroundNormal.Normalize())
		targetPitch = clamp(targetPitch, -0.6, 0.6)
		targetRoll = clamp(targetRoll, -0.6, 0.6)
		smooth = 0.28
	}
	if supportPlane.Valid {
		if supportPlane.HasPitch && absf(supportPlane.Pitch) > absf(targetPitch) {
			targetPitch = clamp(supportPlane.Pitch, -0.6, 0.6)
		}
		if supportPlane.HasRoll && absf(supportPlane.Roll) > absf(targetRoll) {
			targetRoll = clamp(supportPlane.Roll, -0.6, 0.6)
		}
	}

	pitchDelta := clamp((targetPitch-vehicle.Pitch)*smooth, -vehicleTiltStep, vehicleTiltStep)
	rollDelta := clamp((targetRoll-vehicle.Roll)*smooth, -vehicleTiltStep, vehicleTiltStep)
	vehicle.Pitch += pitchDelta
	vehicle.Roll += rollDelta
	return vehicle
}

func (w *PhysicsWorld) probeVehicleGround(vehicle VehicleBody, forwardOffset float32) groundProbeResult {
	return w.probeVehicleGroundOffset(vehicle, forwardOffset, 0)
}

func (w *PhysicsWorld) probeVehicleGroundOffset(vehicle VehicleBody, forwardOffset, rightOffset float32) groundProbeResult {
	forward := geom.FromHeading(vehicle.Heading)
	right := geom.Planar(-forward.Z, forward.X)
	probePosition := vehicle.Position.
		Add(forward.MulScalar(forwardOffset)).
		Add(right.MulScalar(rightOffset))
	supportBaseHeight := vehicle.Height
	if vehicle.GroundHeight > supportBaseHeight {
		supportBaseHeight = vehicle.GroundHeight
	}

	probe := vehicle
	probe.Position = probePosition
	probe.Height = supportBaseHeight - vehicleGroundSnapDistance

	origin := geom.V3(probePosition.X, supportBaseHeight, probePosition.Z)
	normalSum := geom.Vec3{}
	groundContacts := 0

	for iteration := 0; iteration < vehicleCollisionIterations; iteration++ {
		hit, intersects := w.queryBodyCapsuleMapHit(probe)
		if !intersects {
			break
		}
		if hit.Normal.Y < vehicleGroundNormalMinY {
			break
		}

		push := hit.Penetration + vehicleCollisionSkin
		probe.Position.X += hit.Normal.X * push
		probe.Position.Z += hit.Normal.Z * push
		probe.Height += hit.Normal.Y * push
		normalSum = normalSum.Add(hit.Normal)
		groundContacts++
	}

	if groundContacts == 0 {
		return groundProbeResult{Origin: origin}
	}

	normal := normalSum.Normalize()
	if normal.LengthSquared() == 0 {
		normal = geom.V3(0, 1, 0)
	}

	return groundProbeResult{
		Hit:    true,
		Origin: origin,
		Point:  geom.V3(probe.Position.X, probe.Height, probe.Position.Z),
		Normal: normal,
	}
}

func (w *PhysicsWorld) probeVehicleSupportPlane(vehicle VehicleBody) supportPlaneResult {
	radius, halfSegment := bodyCapsuleDimensions(vehicle.Params)
	forwardOffset := maxf(halfSegment*0.75, vehicle.Params.BodyLength*0.24)
	rightOffset := maxf(vehicle.Params.BodyWidth*0.28, radius*0.9)

	center := w.probeVehicleGroundOffset(vehicle, 0, 0)
	front := w.probeVehicleGroundOffset(vehicle, forwardOffset, 0)
	rear := w.probeVehicleGroundOffset(vehicle, -forwardOffset, 0)
	left := w.probeVehicleGroundOffset(vehicle, 0, -rightOffset)
	right := w.probeVehicleGroundOffset(vehicle, 0, rightOffset)

	result := supportPlaneResult{
		Center: center,
		Front:  front,
		Rear:   rear,
		Left:   left,
		Right:  right,
	}

	samples := []groundProbeResult{center, front, rear, left, right}
	totalHeight := float32(0)
	hitCount := 0
	normalSum := geom.Vec3{}
	for _, sample := range samples {
		if !sample.Hit {
			continue
		}
		totalHeight += sample.Point.Y
		normalSum = normalSum.Add(sample.Normal)
		hitCount++
	}
	if hitCount == 0 {
		return result
	}
	result.GroundHeight = totalHeight / float32(hitCount)

	pitch, pitchHit := supportPitchFromProbes(center, front, rear, forwardOffset)
	roll, rollHit := supportRollFromProbes(center, left, right, rightOffset)
	result.Pitch = pitch
	result.Roll = roll
	result.HasPitch = pitchHit
	result.HasRoll = rollHit

	if pitchHit || rollHit {
		_, _, supportUp := vehicleAxes(vehicle.Heading, pitch, roll)
		if supportUp.LengthSquared() > 0 {
			result.Normal = supportUp.Normalize()
			result.Valid = true
		}
	}

	if normalSum.LengthSquared() > 0 {
		averagedNormal := normalSum.Normalize()
		if result.Valid {
			result.Normal = blendGroundNormal(result.Normal, averagedNormal, 0.5)
		} else {
			result.Normal = averagedNormal
			result.Valid = true
		}
	}

	return result
}

func supportPitchFromProbes(center, front, rear groundProbeResult, forwardOffset float32) (float32, bool) {
	span := forwardOffset * 2
	switch {
	case front.Hit && rear.Hit && span > 0:
		return atan2f(front.Point.Y-rear.Point.Y, span), true
	case center.Hit && front.Hit && forwardOffset > 0:
		return atan2f(front.Point.Y-center.Point.Y, forwardOffset), true
	case center.Hit && rear.Hit && forwardOffset > 0:
		return atan2f(center.Point.Y-rear.Point.Y, forwardOffset), true
	default:
		return 0, false
	}
}

func supportRollFromProbes(center, left, right groundProbeResult, rightOffset float32) (float32, bool) {
	span := rightOffset * 2
	switch {
	case left.Hit && right.Hit && span > 0:
		return atan2f(left.Point.Y-right.Point.Y, span), true
	case center.Hit && left.Hit && rightOffset > 0:
		return atan2f(left.Point.Y-center.Point.Y, rightOffset), true
	case center.Hit && right.Hit && rightOffset > 0:
		return atan2f(center.Point.Y-right.Point.Y, rightOffset), true
	default:
		return 0, false
	}
}

func projectVelocityOnPlane(velocity geom.Vec3, normal geom.Vec3) geom.Vec3 {
	if normal.LengthSquared() == 0 {
		return velocity
	}

	normal = normal.Normalize()
	return velocity.Sub(normal.MulScalar(velocity.Dot(normal)))
}

func blendGroundNormal(current, target geom.Vec3, blend float32) geom.Vec3 {
	if target.LengthSquared() == 0 {
		return current
	}
	target = target.Normalize()
	if current.LengthSquared() == 0 {
		return target
	}

	current = current.Normalize()
	blend = clamp(blend, 0, 1)
	mixed := current.MulScalar(1 - blend).Add(target.MulScalar(blend))
	if mixed.LengthSquared() == 0 {
		return target
	}
	return mixed.Normalize()
}

func atan2f(y, x float32) float32 {
	return float32(math.Atan2(float64(y), float64(x)))
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
