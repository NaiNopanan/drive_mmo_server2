package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func TestNewArcadeCityWorld_UsesArcadeDriveTuning(t *testing.T) {
	world := NewArcadeCityWorld()

	if world.Vehicle.ArcadeTuning.MaxForwardSpeed != fixed.FromInt(28) {
		t.Fatalf("arcade max forward speed mismatch: got=%v want=28", world.Vehicle.ArcadeTuning.MaxForwardSpeed)
	}
	if world.Vehicle.Tuning.MaxSpeed != fixed.FromInt(28) {
		t.Fatalf("base vehicle tuning mismatch: got=%v want=28", world.Vehicle.Tuning.MaxSpeed)
	}
}

func TestArcadeVehicle_AcceleratesForwardOnFlatGround(t *testing.T) {
	trace := traceFlatForward()

	if trace.GroundedTicks != 150 {
		t.Fatalf("flat forward grounded ticks mismatch: got=%d want=150", trace.GroundedTicks)
	}
	if trace.TickA != 50 {
		t.Fatalf("flat forward speed tick mismatch: got=%d want=50", trace.TickA)
	}
	if trace.TickB != 84 {
		t.Fatalf("flat forward distance tick mismatch: got=%d want=84", trace.TickB)
	}
	assertFixedString(t, "flat forward final z", trace.FinalPosition.Z, "35.649999")
	assertFixedString(t, "flat forward final vz", trace.FinalVelocity.Z, "22.399999")
}

func TestArcadeVehicle_ReversesOnFlatGround(t *testing.T) {
	trace := traceFlatReverse()

	if trace.GroundedTicks != 120 {
		t.Fatalf("flat reverse grounded ticks mismatch: got=%d want=120", trace.GroundedTicks)
	}
	if trace.TickA != 16 {
		t.Fatalf("flat reverse speed tick mismatch: got=%d want=16", trace.TickA)
	}
	if trace.TickB != 55 {
		t.Fatalf("flat reverse distance tick mismatch: got=%d want=55", trace.TickB)
	}
	assertFixedString(t, "flat reverse final z", trace.FinalPosition.Z, "-10.549999")
	assertFixedString(t, "flat reverse final vz", trace.FinalVelocity.Z, "-5.999999")
}

func TestArcadeVehicle_BrakeDropsSpeedOnFlatGround(t *testing.T) {
	trace := traceFlatBrake()

	if trace.GroundedTicks != 45 {
		t.Fatalf("flat brake grounded ticks mismatch: got=%d want=45", trace.GroundedTicks)
	}
	if trace.TickA != 15 {
		t.Fatalf("flat brake half-speed tick mismatch: got=%d want=15", trace.TickA)
	}
	assertFixedString(t, "flat brake final z", trace.FinalPosition.Z, "29.489999")
	assertFixedString(t, "flat brake final vz", trace.FinalVelocity.Z, "0.000000")
}

func TestArcadeVehicle_SteersIntoArcOnFlatGround(t *testing.T) {
	trace := traceFlatTurn()

	if trace.GroundedTicks != 120 {
		t.Fatalf("flat turn grounded ticks mismatch: got=%d want=120", trace.GroundedTicks)
	}
	if trace.TickA != 23 {
		t.Fatalf("flat turn yaw tick mismatch: got=%d want=23", trace.TickA)
	}
	if trace.TickB != 47 {
		t.Fatalf("flat turn arc tick mismatch: got=%d want=47", trace.TickB)
	}
	assertFixedString(t, "flat turn final x", trace.FinalPosition.X, "19.612441")
	assertFixedString(t, "flat turn final z", trace.FinalPosition.Z, "23.363236")
}

func TestArcadeCityWorld_ClimbsRampAndMostlyStaysGrounded(t *testing.T) {
	trace := traceRampClimb()

	if trace.GroundedTicks != 186 {
		t.Fatalf("ramp climb grounded ticks mismatch: got=%d want=186", trace.GroundedTicks)
	}
	if trace.TickA != 104 {
		t.Fatalf("ramp climb height tick mismatch: got=%d want=104", trace.TickA)
	}
	if trace.PeakYTick != 120 {
		t.Fatalf("ramp climb peak tick mismatch: got=%d want=120", trace.PeakYTick)
	}
	assertFixedString(t, "ramp climb peak y", trace.PeakY, "6.849999")
	assertFixedString(t, "ramp climb final y", trace.FinalPosition.Y, "1.495547")
}

func TestArcadeCityWorld_DescendsRampAndLosesHeight(t *testing.T) {
	trace := traceRampDescend()

	if trace.GroundedTicks != 116 {
		t.Fatalf("ramp descend grounded ticks mismatch: got=%d want=116", trace.GroundedTicks)
	}
	if trace.TickA != 47 {
		t.Fatalf("ramp descend first drop tick mismatch: got=%d want=47", trace.TickA)
	}
	if trace.TickB != 113 {
		t.Fatalf("ramp descend flat-height tick mismatch: got=%d want=113", trace.TickB)
	}
	if trace.TickC != 1 {
		t.Fatalf("ramp descend up-recover tick mismatch: got=%d want=1", trace.TickC)
	}
	assertFixedString(t, "ramp descend final y", trace.FinalPosition.Y, "0.849999")
}

func TestArcadeCityWorld_SteersOnRampWithoutLeavingRoad(t *testing.T) {
	trace := traceRampSteer()

	if trace.GroundedTicks != 104 {
		t.Fatalf("ramp steer grounded ticks mismatch: got=%d want=104", trace.GroundedTicks)
	}
	if trace.TickA != 29 {
		t.Fatalf("ramp steer yaw tick mismatch: got=%d want=29", trace.TickA)
	}
	if trace.TickB != 39 {
		t.Fatalf("ramp steer lateral tick mismatch: got=%d want=39", trace.TickB)
	}
	assertFixedString(t, "ramp steer final y", trace.FinalPosition.Y, "5.031532")
	assertFixedString(t, "ramp steer final x", trace.FinalPosition.X, "9.454590")
}

func TestArcadeCityWorld_CoastingSteerBuildsYaw(t *testing.T) {
	world := NewArcadeCityWorld()
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 120; i++ {
		world.Vehicle.Input = VehicleInput{}
		world.Step(dt)
	}
	for i := 0; i < 90; i++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		world.Step(dt)
	}

	startYaw := world.Vehicle.Yaw
	startX := world.Vehicle.Position.X

	for i := 0; i < 120; i++ {
		world.Vehicle.Input = VehicleInput{Steer: fixed.One}
		world.Step(dt)
	}

	yawDelta := world.Vehicle.Yaw.Sub(startYaw).Abs()
	xDelta := world.Vehicle.Position.X.Sub(startX).Abs()

	if yawDelta.Cmp(fixed.FromFraction(3, 10)) <= 0 {
		t.Fatalf("arcade coasting steer yaw response too weak: got=%v need>0.3", yawDelta)
	}
	if xDelta.Cmp(fixed.FromInt(2)) <= 0 {
		t.Fatalf("arcade coasting steer arc too small: got=%v need>2", xDelta)
	}
}

func TestArcadeCityWorld_DrivingUnderBridgeStaysOnLowerRoad(t *testing.T) {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28))
	world.Vehicle.Yaw = fixed.Zero
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	maxYUnderBridge := fixed.Zero
	seenUnderBridge := false

	for tick := 0; tick < 300; tick++ {
		world.Vehicle.Input = VehicleInput{}
		if tick >= 30 {
			world.Vehicle.Input.Throttle = fixed.FromFraction(3, 5)
		}

		world.Step(dt)

		if world.Vehicle.Position.Z.Cmp(fixed.FromInt(-8)) >= 0 && world.Vehicle.Position.Z.Cmp(fixed.FromInt(8)) <= 0 {
			seenUnderBridge = true
			if world.Vehicle.Position.Y.Cmp(maxYUnderBridge) > 0 {
				maxYUnderBridge = world.Vehicle.Position.Y
			}
		}
	}

	if !seenUnderBridge {
		t.Fatalf("arcade vehicle never reached under-bridge section")
	}
	if maxYUnderBridge.Cmp(fixed.FromInt(2)) > 0 {
		t.Fatalf("arcade vehicle was lifted toward overpass while on lower road: maxY=%v", maxYUnderBridge)
	}
}

func TestArcadeCityWorld_DescendingOffRampReturnsToFlatGround(t *testing.T) {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(5), fixed.FromInt(6), fixed.FromInt(2))
	world.Vehicle.Yaw = fixed.FromFraction(11, 10)
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	reachedFlat := false

	for i := 0; i < 360; i++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		world.Step(dt)

		if world.Vehicle.Position.X.Cmp(fixed.FromInt(24)) >= 0 && world.Vehicle.Position.Z.Cmp(fixed.FromInt(14)) >= 0 {
			reachedFlat = true
			break
		}
	}

	if !reachedFlat {
		t.Fatalf("arcade vehicle never reached flat road after descending ramp: pos=%v", world.Vehicle.Position)
	}

	for i := 0; i < 75; i++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(1, 5)}
		world.Step(dt)
	}

	if world.Vehicle.Position.Y.Cmp(fixed.FromFraction(14, 10)) > 0 {
		t.Fatalf("arcade vehicle stayed too high after returning to flat road: y=%v", world.Vehicle.Position.Y)
	}
	if world.Vehicle.UpWS.Y.Cmp(fixed.FromFraction(49, 50)) < 0 {
		t.Fatalf("arcade vehicle stayed tilted after returning to flat road: up=%v", world.Vehicle.UpWS)
	}
}

func TestArcadeCityWorld_BuildingCollisionStopsPenetration(t *testing.T) {
	trace := traceBuildingCollision()

	if trace.GroundedTicks != 178 {
		t.Fatalf("building collision grounded ticks mismatch: got=%d want=178", trace.GroundedTicks)
	}
	if trace.TickA != 1 {
		t.Fatalf("building collision stop tick mismatch: got=%d want=1", trace.TickA)
	}
	assertFixedString(t, "building collision final x", trace.FinalPosition.X, "-73.301000")
	assertFixedString(t, "building collision final z", trace.FinalPosition.Z, "-61.999999")
	assertFixedString(t, "building collision final vx", trace.FinalVelocity.X, "0.000000")
}

func TestArcadeCityWorld_GuardRailCollisionStopsPenetration(t *testing.T) {
	trace := traceGuardRailCollision()

	if trace.GroundedTicks != 180 {
		t.Fatalf("guard rail grounded ticks mismatch: got=%d want=180", trace.GroundedTicks)
	}
	if trace.TickA != 2 {
		t.Fatalf("guard rail stop tick mismatch: got=%d want=2", trace.TickA)
	}
	assertFixedString(t, "guard rail final x", trace.FinalPosition.X, "-10.933529")
	assertFixedString(t, "guard rail final y", trace.FinalPosition.Y, "3.180799")
	assertFixedString(t, "guard rail final z", trace.FinalPosition.Z, "-1.005768")
}

func TestArcadeVehicle_WallBoundsContainVehicle(t *testing.T) {
	trace := traceWallCollision()

	if trace.GroundedTicks != 180 {
		t.Fatalf("wall collision grounded ticks mismatch: got=%d want=180", trace.GroundedTicks)
	}
	if trace.TickA != 52 {
		t.Fatalf("wall collision tick mismatch: got=%d want=52", trace.TickA)
	}
	assertFixedString(t, "wall collision final z", trace.FinalPosition.Z, "4.700000")
	assertFixedString(t, "wall collision final vz", trace.FinalVelocity.Z, "0.000000")
}

func TestArcadeCityWorld_SmokeDriveStaysGroundedAndOutsideObstacles(t *testing.T) {
	world := NewArcadeCityWorld()
	dt := fixed.FromFraction(1, 60)
	groundedTicks := 0

	for tick := 0; tick < 360; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		if tick > 240 {
			world.Vehicle.Input = VehicleInput{Brake: fixed.One}
		}

		world.Step(dt)

		if world.Vehicle.OnGround {
			groundedTicks++
		}
		if tick > 90 && !world.Vehicle.OnGround {
			t.Fatalf("arcade vehicle lost ground contact at tick=%d pos=%v vel=%v", tick, world.Vehicle.Position, world.Vehicle.Velocity)
		}
		if overlapsAnyObstacle(world.Vehicle.Vehicle, world.Map.Obstacles) {
			t.Fatalf("arcade vehicle overlapped city obstacle at tick=%d pos=%v", tick, world.Vehicle.Position)
		}
		assertVehicleInsideBounds(t, world.Vehicle.Vehicle, world.Map.Bounds)
	}

	if groundedTicks < 320 {
		t.Fatalf("arcade vehicle grounded too infrequently: groundedTicks=%d", groundedTicks)
	}
}

func arcadeSettleVehicle(v *ArcadeVehicle, ground GroundQuery, steps int) {
	arcadeStepVehicle(v, ground, steps, VehicleInput{})
}

func arcadeStepVehicle(v *ArcadeVehicle, ground GroundQuery, steps int, input VehicleInput) {
	dt := fixed.FromFraction(1, 60)
	for i := 0; i < steps; i++ {
		v.Input = input
		v.Step(dt, ground)
	}
}

func arcadeSettleVehicleWithWorld(v *ArcadeVehicle, ground GroundQuery, bounds WallBounds, obstacles []CityObstacle, steps int) {
	arcadeStepVehicleWithWorld(v, ground, bounds, obstacles, steps, VehicleInput{})
}

func arcadeStepVehicleWithWorld(v *ArcadeVehicle, ground GroundQuery, bounds WallBounds, obstacles []CityObstacle, steps int, input VehicleInput) {
	dt := fixed.FromFraction(1, 60)
	for i := 0; i < steps; i++ {
		v.Input = input
		v.Step(dt, ground)
		CollideVehicleWithObstacles(&v.Vehicle, obstacles)
		CollideVehicleWithWalls(&v.Vehicle, bounds)
	}
}

func arcadeStepCityWorld(world *ArcadeCityWorld, steps int, input VehicleInput) {
	dt := fixed.FromFraction(1, 60)
	for i := 0; i < steps; i++ {
		world.Vehicle.Input = input
		world.Step(dt)
	}
}

func arcadeStepCityWorldCountGrounded(world *ArcadeCityWorld, steps int, input VehicleInput) int {
	dt := fixed.FromFraction(1, 60)
	groundedTicks := 0
	for i := 0; i < steps; i++ {
		world.Vehicle.Input = input
		world.Step(dt)
		if world.Vehicle.OnGround {
			groundedTicks++
		}
	}
	return groundedTicks
}

func arcadePlanarSpeed(v ArcadeVehicle) fixed.Fixed {
	return geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length()
}

func arcadePlaceVehicleForObstacleApproach(v *ArcadeVehicle, obstacle CityObstacle, clearance fixed.Fixed) {
	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	rideY := obstacle.BaseY.Add(v.ArcadeTuning.RideHeight).Add(fixed.FromFraction(1, 5))

	spanX := obstacle.MaxX.Sub(obstacle.MinX)
	spanZ := obstacle.MaxZ.Sub(obstacle.MinZ)
	if spanX.Cmp(spanZ) >= 0 {
		v.Position = geom.V3(
			obstacle.Center().X,
			rideY,
			obstacle.MinZ.Sub(halfLen).Sub(clearance),
		)
		v.Yaw = fixed.Zero
	} else {
		v.Position = geom.V3(
			obstacle.MinX.Sub(halfWidth).Sub(clearance),
			rideY,
			obstacle.Center().Z,
		)
		v.Yaw = fixedFromFloat(1.5707963267948966)
	}

	v.Velocity = geom.Zero()
	v.YawVelocity = fixed.Zero
	v.UpdateBasisFromYaw()
}

func assertFixedString(t *testing.T, label string, got fixed.Fixed, want string) {
	t.Helper()
	if got.String() != want {
		t.Fatalf("%s mismatch: got=%s want=%s", label, got.String(), want)
	}
}
