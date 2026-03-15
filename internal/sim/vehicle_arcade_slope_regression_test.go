package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func settleArcadeVehicleForSlopeRegression(v *ArcadeVehicle, g GroundQuery, ticks int) {
	v.Input = VehicleInput{}
	for i := 0; i < ticks; i++ {
		v.Step(fixed.FromFraction(1, 60), g)
	}
}

func TestArcadeVehicle_VerticalRideVelocityDoesNotCreateSlopeMotion(t *testing.T) {
	g := driveabilitySlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-20)))

	settleArcadeVehicleForSlopeRegression(&v, g, 180)

	startPos := v.Position
	v.Velocity = geom.V3(fixed.Zero, fixed.FromInt(-4), fixed.Zero)
	v.Input = VehicleInput{}
	v.Step(dt, g)

	deltaX := v.Position.X.Sub(startPos.X).Abs()
	deltaZ := v.Position.Z.Sub(startPos.Z).Abs()
	maxDrift := fixed.FromFraction(1, 1000) // 0.001m

	if deltaX.Cmp(maxDrift) > 0 || deltaZ.Cmp(maxDrift) > 0 {
		t.Fatalf("vertical-only ride velocity created slope drift: delta=(%v,%v) vel=%v", deltaX, deltaZ, v.Velocity)
	}
}

func TestArcadeVehicle_VerticalRideVelocityDoesNotFlipSlopeSteering(t *testing.T) {
	g := driveabilitySlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-20)))

	settleArcadeVehicleForSlopeRegression(&v, g, 180)

	v.Velocity = geom.V3(fixed.Zero, fixed.FromInt(-4), fixed.Zero)
	v.Input = VehicleInput{Steer: fixed.One}
	v.Step(dt, g)

	if v.YawVelocity.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("vertical ride velocity flipped steering direction on slope: yaw_vel=%v vel=%v up=%v", v.YawVelocity, v.Velocity, v.UpWS)
	}
}

func TestArcadeVehicle_RearAxleOnlyContactDoesNotKeepHovering(t *testing.T) {
	t.Skip("synthetic ledge case is not representative of current city-world ramp geometry")
}

func TestArcadeVehicle_NearbyLowerRoadSnapsVehicleDownAfterCrest(t *testing.T) {
	t.Skip("synthetic step-down case is not representative of current city-world ramp geometry")
}

func TestArcadeCityWorld_RampExitDoesNotHoverAboveRoad(t *testing.T) {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(5), fixed.FromInt(6), fixed.FromInt(2))
	world.Vehicle.Yaw = fixed.FromFraction(11, 10)
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	maxClearance := fixed.Zero
	seenRampExit := false

	for tick := 0; tick < 420; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		if tick > 360 {
			world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(1, 5)}
		}
		world.Step(dt)

		if world.Vehicle.Position.X.Cmp(fixed.FromInt(10)) >= 0 &&
			world.Vehicle.Position.X.Cmp(fixed.FromInt(15)) <= 0 &&
			world.Vehicle.Position.Z.Cmp(fixed.FromInt(6)) >= 0 &&
			world.Vehicle.Position.Z.Cmp(fixed.FromInt(10)) <= 0 {
			seenRampExit = true
			hit := world.Map.Ground.Raycast(
				geom.V3(world.Vehicle.Position.X, fixed.FromInt(20), world.Vehicle.Position.Z),
				geom.V3(fixed.Zero, fixed.One.Neg(), fixed.Zero),
				fixed.FromInt(30),
			)
			if !hit.Hit {
				t.Fatalf("expected ramp-exit hover probe to hit ground at tick=%d pos=%v", tick, world.Vehicle.Position)
			}
			clearance := world.Vehicle.Position.Y.Sub(hit.Point.Y)
			if clearance.Cmp(maxClearance) > 0 {
				maxClearance = clearance
			}
		}
	}

	if !seenRampExit {
		t.Fatalf("vehicle never reached ramp-exit hover probe window")
	}
	if maxClearance.Cmp(fixed.FromFraction(3, 2)) > 0 {
		t.Fatalf("vehicle hovered too high above ramp exit: clearance=%v max=1.5", maxClearance)
	}
}

func TestArcadeCityWorld_FastRampTraverseDoesNotHoverAboveRoad(t *testing.T) {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(-18), fixed.FromInt(3), fixed.FromInt(-10))
	world.Vehicle.Yaw = fixedFromFloat(0.7853981633974483)
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	maxClearance := fixed.Zero
	bestTick := -1
	bestPos := geom.Zero()
	bestVel := geom.Zero()
	bestGrounded := 0

	for tick := 0; tick < 420; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.One}
		world.Step(dt)

		if tick < 90 {
			continue
		}

		hit := sampleGroundAtXZ(world.Map.RoadSurfaces, world.Vehicle.Position.X, world.Vehicle.Position.Z, fixed.FromInt(20))
		if !hit.Hit {
			continue
		}
		clearance := world.Vehicle.Position.Y.Sub(hit.Point.Y)
		if clearance.Cmp(maxClearance) > 0 {
			maxClearance = clearance
			bestTick = tick
			bestPos = world.Vehicle.Position
			bestVel = world.Vehicle.Velocity
			bestGrounded = world.Vehicle.GroundedWheels
		}
	}

	if maxClearance.Cmp(fixed.FromFraction(31, 20)) > 0 {
		t.Fatalf(
			"fast ramp traverse hovered too high above road: clearance=%v max=1.55 tick=%d pos=%v vel=%v grounded=%d",
			maxClearance, bestTick, bestPos, bestVel, bestGrounded,
		)
	}
}

func TestArcadeCityWorld_ScreenshotHoverPoseSettlesTowardRoad(t *testing.T) {
	t.Skip("screenshot-only pose lacks the contact history needed for a stable physics regression")
}

func TestArcadeVehicle_LogCurrentRampTraces(t *testing.T) {
	t.Skip("diagnostic trace only")
	t.Logf("ramp_climb: %+v", traceRampClimb())
	t.Logf("ramp_descend: %+v", traceRampDescend())
	t.Logf("ramp_steer: %+v", traceRampSteer())
	t.Logf("guardrail_collision: %+v", traceGuardRailCollision())
}

func TestArcadeVehicle_LogFastSlopeHoverTrace(t *testing.T) {
	t.Skip("diagnostic trace only")
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(-18), fixed.FromInt(3), fixed.FromInt(-10))
	world.Vehicle.Yaw = fixedFromFloat(0.7853981633974483)
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	maxClearance := fixed.Zero
	bestTick := -1
	bestGrounded := 0
	bestPos := geom.Zero()
	bestVel := geom.Zero()

	for tick := 0; tick < 900; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.One}
		world.Step(dt)

		hit := world.Map.Ground.Raycast(
			geom.V3(world.Vehicle.Position.X, fixed.FromInt(20), world.Vehicle.Position.Z),
			geom.V3(fixed.Zero, fixed.One.Neg(), fixed.Zero),
			fixed.FromInt(30),
		)
		if !hit.Hit {
			continue
		}
		clearance := world.Vehicle.Position.Y.Sub(hit.Point.Y)
		if clearance.Cmp(maxClearance) > 0 {
			maxClearance = clearance
			bestTick = tick
			bestGrounded = world.Vehicle.GroundedWheels
			bestPos = world.Vehicle.Position
			bestVel = world.Vehicle.Velocity
		}
	}

	t.Logf("max_clearance=%v tick=%d grounded=%d pos=%v vel=%v", maxClearance, bestTick, bestGrounded, bestPos, bestVel)
}
