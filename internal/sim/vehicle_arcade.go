package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type ArcadeDriveTuning struct {
	MaxForwardSpeed fixed.Fixed
	MaxReverseSpeed fixed.Fixed
	AccelRate       fixed.Fixed
	BrakeRate       fixed.Fixed
	CoastDecel      fixed.Fixed

	TurnRateLowSpeed  fixed.Fixed
	TurnRateHighSpeed fixed.Fixed
	TurnSpeedRange    fixed.Fixed
	YawAccelRate      fixed.Fixed
	YawReleaseRate    fixed.Fixed
	CoastTurnAssist   fixed.Fixed
	AirTurnMultiplier fixed.Fixed

	SideDecel       fixed.Fixed
	CoastSideDecel  fixed.Fixed
	RideHeight      fixed.Fixed
	RideSpring      fixed.Fixed
	RideDamping     fixed.Fixed
	Gravity         fixed.Fixed
	GroundSnapSpeed fixed.Fixed
}

type ArcadeVehicle struct {
	Vehicle
	ArcadeTuning ArcadeDriveTuning
}

type ArcadeCityWorld struct {
	Tick    uint64
	Map     CityMap
	Vehicle ArcadeVehicle
}

func EasyArcadeDriveTuning() ArcadeDriveTuning {
	return ArcadeDriveTuning{
		MaxForwardSpeed: fixed.FromInt(28),
		MaxReverseSpeed: fixed.FromInt(10),
		AccelRate:       fixed.FromInt(12),
		BrakeRate:       fixed.FromInt(24),
		CoastDecel:      fixed.FromInt(8),

		TurnRateLowSpeed:  fixed.FromFraction(9, 5), // 1.8 rad/s
		TurnRateHighSpeed: fixed.FromFraction(4, 5), // 0.8 rad/s
		TurnSpeedRange:    fixed.FromInt(24),
		YawAccelRate:      fixed.FromInt(7),
		YawReleaseRate:    fixed.FromInt(10),
		CoastTurnAssist:   fixed.FromFraction(1, 4), // 0.25 rad/s
		AirTurnMultiplier: fixed.FromFraction(3, 10),

		SideDecel:       fixed.FromInt(18),
		CoastSideDecel:  fixed.FromInt(8),
		RideHeight:      fixed.FromFraction(17, 20), // 0.85m
		RideSpring:      fixed.FromInt(16),
		RideDamping:     fixed.FromInt(10),
		Gravity:         fixed.FromFraction(-98, 10),
		GroundSnapSpeed: fixed.FromInt(14),
	}
}

func NewArcadeVehicle(id uint32, pos geom.Vec3) ArcadeVehicle {
	base := NewVehicleWithTuning(id, pos, EasyDriveTuning())
	base.UpdateBasisFromYaw()
	return ArcadeVehicle{
		Vehicle:      base,
		ArcadeTuning: EasyArcadeDriveTuning(),
	}
}

func NewArcadeCityWorld() ArcadeCityWorld {
	city := BuildCurvedOverpassCity()
	v := NewArcadeVehicle(1, city.SpawnPosition)
	v.Yaw = city.SpawnYaw
	v.UpdateBasisFromYaw()

	return ArcadeCityWorld{
		Map:     city,
		Vehicle: v,
	}
}

func (w *ArcadeCityWorld) Reset() {
	if w == nil {
		return
	}
	reset := NewArcadeCityWorld()
	*w = reset
}

func (w *ArcadeCityWorld) Step(dt fixed.Fixed) {
	if w == nil {
		return
	}
	w.Vehicle.Step(dt, w.Map.Ground)
	CollideVehicleWithObstacles(&w.Vehicle.Vehicle, w.Map.Obstacles)
	CollideVehicleWithWalls(&w.Vehicle.Vehicle, w.Map.Bounds)
	w.Tick++
}

func (v *ArcadeVehicle) Step(dt fixed.Fixed, g GroundQuery) {
	if v == nil {
		return
	}

	v.UpdateSteering(dt)

	currentNormal, currentGroundY := v.sampleGround(g)
	groundContact := v.OnGround && v.GroundedWheels > 0
	grounded := groundContact
	if groundContact {
		v.UpdateBasis(currentNormal)
	}

	planarNormal := v.UpWS
	if planarNormal.LengthSq().Cmp(fixed.Zero) == 0 {
		planarNormal = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
	}
	planarForward, planarRight := v.arcadePlanarBasis(planarNormal)
	bodyForward, _ := HeadingFromYaw(v.Yaw)
	driveForward := v.arcadeHorizontalDriveBasis(planarForward, bodyForward)
	horizontalVelocity := geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z)

	forwardSpeed := v.Velocity.Dot(planarForward)
	// Ignore ride-height snap velocity when the car is effectively stationary in XZ.
	// Without this, pure vertical correction on a slope can masquerade as reverse motion.
	if horizontalVelocity.Length().Cmp(fixed.FromFraction(1, 5)) < 0 {
		forwardSpeed = horizontalVelocity.Dot(driveForward)
	}
	sideSpeed := v.Velocity.Dot(planarRight)
	speedAbs := forwardSpeed.Abs()

	targetForward := fixed.Zero
	if v.Input.Throttle.Cmp(fixed.Zero) > 0 {
		targetForward = v.Input.Throttle.Mul(v.ArcadeTuning.MaxForwardSpeed)
	} else if v.Input.Throttle.Cmp(fixed.Zero) < 0 {
		targetForward = v.Input.Throttle.Abs().Mul(v.ArcadeTuning.MaxReverseSpeed).Neg()
	}

	forwardRate := v.ArcadeTuning.CoastDecel
	if targetForward.Cmp(fixed.Zero) != 0 {
		forwardRate = v.ArcadeTuning.AccelRate
		if (forwardSpeed.Cmp(fixed.Zero) > 0) != (targetForward.Cmp(fixed.Zero) > 0) {
			forwardRate = v.ArcadeTuning.BrakeRate
		}
	}
	if v.Input.Brake.Cmp(fixed.Zero) > 0 {
		targetForward = fixed.Zero
		forwardRate = v.ArcadeTuning.BrakeRate.Add(v.Input.Brake.Mul(v.ArcadeTuning.BrakeRate))
	}
	forwardSpeed = moveTowards(forwardSpeed, targetForward, forwardRate.Mul(dt))

	sideRate := v.ArcadeTuning.SideDecel
	coastingSteer := v.Input.Steer.Abs().Cmp(fixed.FromFraction(2, 100)) >= 0 &&
		v.Input.Throttle.Abs().Cmp(fixed.FromFraction(2, 100)) < 0 &&
		v.Input.Brake.Abs().Cmp(fixed.FromFraction(2, 100)) < 0
	if coastingSteer {
		sideRate = v.ArcadeTuning.CoastSideDecel
	}
	sideSpeed = moveTowards(sideSpeed, fixed.Zero, sideRate.Mul(dt))

	turnRate := v.arcadeTurnRate(speedAbs)
	if coastingSteer {
		turnRate = turnRate.Add(v.ArcadeTuning.CoastTurnAssist)
	}
	if !grounded {
		turnRate = turnRate.Mul(v.ArcadeTuning.AirTurnMultiplier)
	}

	reverseFactor := fixed.One
	if forwardSpeed.Cmp(fixed.FromFraction(-1, 5)) < 0 {
		reverseFactor = fixed.One.Neg()
	}
	targetYawVel := v.Input.Steer.Mul(turnRate).Mul(reverseFactor)
	yawRateDelta := v.ArcadeTuning.YawReleaseRate.Mul(dt)
	if v.Input.Steer.Abs().Cmp(fixed.FromFraction(2, 100)) >= 0 {
		yawRateDelta = v.ArcadeTuning.YawAccelRate.Mul(dt)
	}
	v.YawVelocity = moveTowards(v.YawVelocity, targetYawVel, yawRateDelta)
	v.Yaw = v.Yaw.Add(v.YawVelocity.Mul(dt))

	planarForward, planarRight = v.arcadePlanarBasis(planarNormal)
	velocity := planarForward.Scale(forwardSpeed).Add(planarRight.Scale(sideSpeed))
	nextPos := v.Position.Add(geom.V3(velocity.X, fixed.Zero, velocity.Z).Scale(dt))

	if grounded {
		desiredY := currentGroundY.Add(v.ArcadeTuning.RideHeight)
		nextPos.Y, v.Velocity.Y = v.followGroundHeight(nextPos.Y, desiredY, dt)
	} else {
		v.Velocity.Y = v.Velocity.Y.Add(v.ArcadeTuning.Gravity.Mul(dt))
		nextPos.Y = nextPos.Y.Add(v.Velocity.Y.Mul(dt))
	}

	v.Velocity.X = velocity.X
	v.Velocity.Z = velocity.Z
	v.Position = nextPos

	finalNormal, finalGroundY := v.sampleGround(g)
	finalGroundContact := v.OnGround && v.GroundedWheels > 0
	finalGrounded := finalGroundContact
	if finalGrounded {
		desiredY := finalGroundY.Add(v.ArcadeTuning.RideHeight)
		v.Position.Y, v.Velocity.Y = v.followGroundHeight(v.Position.Y, desiredY, dt)
	}
	if finalGrounded {
		v.UpdateBasis(finalNormal)
	} else if finalGroundContact {
		v.UpdateBasis(finalNormal)
	} else {
		v.UpdateBasisFromYaw()
	}
}

func (v *ArcadeVehicle) sampleGround(g GroundQuery) (geom.Vec3, fixed.Fixed) {
	sumNormal := geom.Zero()
	sumY := fixed.Zero
	count := int64(0)

	for i := range v.Wheels {
		v.queryWheelGround(i, g)
		if v.Wheels[i].InContact {
			sumNormal = sumNormal.Add(v.Wheels[i].ContactNormal)
			sumY = sumY.Add(v.Wheels[i].ContactPoint.Y)
			count++
		}
	}
	v.finalizeSuspensionState()

	if count == 0 {
		return geom.V3(fixed.Zero, fixed.One, fixed.Zero), v.Position.Y.Sub(v.ArcadeTuning.RideHeight)
	}

	avgGroundY := sumY.Div(fixed.FromInt(count))
	avgNormal := sumNormal.Scale(fixed.FromFraction(1, count)).Normalize()
	if avgNormal.LengthSq().Cmp(fixed.Zero) == 0 {
		avgNormal = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
	}
	if avgNormal.Y.Cmp(fixed.FromFraction(39, 40)) >= 0 {
		return avgNormal, avgGroundY
	}

	blendedNormal := v.safeNormalize(
		v.UpWS.Scale(fixed.FromFraction(3, 5)).
			Add(avgNormal.Scale(fixed.FromFraction(2, 5))),
	)
	if blendedNormal.LengthSq().Cmp(fixed.Zero) == 0 {
		blendedNormal = avgNormal
	}
	if blendedNormal.LengthSq().Cmp(fixed.Zero) == 0 {
		blendedNormal = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
	}

	return blendedNormal.Normalize(), avgGroundY
}

func (v *ArcadeVehicle) arcadeStableGroundContact() bool {
	if v == nil || !v.OnGround || v.GroundedWheels < 2 {
		return false
	}
	if v.GroundedWheels >= 3 {
		return true
	}

	frontContact := false
	rearContact := false
	for i := range v.Wheels {
		if !v.Wheels[i].InContact {
			continue
		}
		if v.WheelDefs[i].IsFront {
			frontContact = true
		} else {
			rearContact = true
		}
	}

	return frontContact && rearContact
}

func (v *ArcadeVehicle) followGroundHeight(currentY, desiredY, dt fixed.Fixed) (fixed.Fixed, fixed.Fixed) {
	yError := desiredY.Sub(currentY)
	targetYVel := yError.Mul(v.ArcadeTuning.RideSpring)
	followAccel := v.ArcadeTuning.RideDamping
	maxStep := v.ArcadeTuning.GroundSnapSpeed.Mul(dt)

	if yError.Cmp(fixed.Zero) < 0 {
		followAccel = followAccel.Mul(fixed.FromInt(6))
		maxStep = maxStep.Mul(fixed.FromInt(4))
	} else if yError.Cmp(fixed.FromFraction(1, 2)) > 0 {
		maxStep = maxStep.Mul(fixed.FromFraction(3, 2))
	}

	nextVelY := moveTowards(v.Velocity.Y, targetYVel, followAccel.Mul(dt))
	nextY := currentY.Add(nextVelY.Mul(dt))
	nextY = moveTowards(nextY, desiredY, maxStep)
	nextVelY = nextY.Sub(currentY).Div(dt)

	return nextY, nextVelY
}

func (v *ArcadeVehicle) arcadePlanarBasis(normal geom.Vec3) (geom.Vec3, geom.Vec3) {
	normal = normal.Normalize()
	if normal.LengthSq().Cmp(fixed.Zero) == 0 {
		normal = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
	}

	bodyForward, _ := HeadingFromYaw(v.Yaw)
	forward := v.safeNormalize(v.projectOntoPlane(bodyForward, normal))
	if forward.LengthSq().Cmp(fixed.Zero) == 0 {
		forward = bodyForward
	}

	right := normal.Cross(forward)
	if right.LengthSq().Cmp(fixed.Zero) == 0 {
		right = geom.V3(fixed.One, fixed.Zero, fixed.Zero)
	} else {
		right = right.Normalize()
	}

	return forward, right
}

func (v *ArcadeVehicle) arcadeHorizontalDriveBasis(dir, fallback geom.Vec3) geom.Vec3 {
	horizontal := geom.V3(dir.X, fixed.Zero, dir.Z)
	if horizontal.LengthSq().Cmp(fixed.Zero) == 0 {
		if fallback.LengthSq().Cmp(fixed.Zero) == 0 {
			return geom.V3(fixed.Zero, fixed.Zero, fixed.One)
		}
		return fallback.Normalize()
	}

	horizontalLen := horizontal.Length()
	return horizontal.Scale(fixed.One.Div(horizontalLen))
}

func (v *ArcadeVehicle) arcadeTurnRate(speed fixed.Fixed) fixed.Fixed {
	if v.ArcadeTuning.TurnSpeedRange.Cmp(fixed.Zero) <= 0 {
		return v.ArcadeTuning.TurnRateHighSpeed
	}

	t := speed.Div(v.ArcadeTuning.TurnSpeedRange)
	t = fixed.Clamp(t, fixed.Zero, fixed.One)
	delta := v.ArcadeTuning.TurnRateHighSpeed.Sub(v.ArcadeTuning.TurnRateLowSpeed)
	return v.ArcadeTuning.TurnRateLowSpeed.Add(delta.Mul(t))
}
