package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

type arcadeTrace struct {
	GroundedTicks int
	TickA         int
	TickB         int
	TickC         int
	PeakYTick     int
	PeakY         fixed.Fixed
	FinalPosition geom.Vec3
	FinalVelocity geom.Vec3
	FinalUp       geom.Vec3
}

func TestArcadeVehicle_LogTickTraces(t *testing.T) {
	t.Skip("diagnostic trace only")

	t.Logf("flat_forward: %+v", traceFlatForward())
	t.Logf("flat_reverse: %+v", traceFlatReverse())
	t.Logf("flat_brake: %+v", traceFlatBrake())
	t.Logf("flat_turn: %+v", traceFlatTurn())
	t.Logf("ramp_climb: %+v", traceRampClimb())
	t.Logf("ramp_descend: %+v", traceRampDescend())
	t.Logf("ramp_steer: %+v", traceRampSteer())
	t.Logf("building_collision: %+v", traceBuildingCollision())
	t.Logf("guardrail_collision: %+v", traceGuardRailCollision())
	t.Logf("wall_collision: %+v", traceWallCollision())
}

func traceFlatForward() arcadeTrace {
	ground := FlatGround{Y: fixed.Zero}
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	arcadeSettleVehicle(&v, ground, 90)

	startZ := v.Position.Z
	trace := arcadeTrace{TickA: -1, TickB: -1, TickC: -1}
	for tick := 1; tick <= 150; tick++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(4, 5)}
		v.Step(fixed.FromFraction(1, 60), ground)
		if v.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && arcadePlanarSpeed(v).Cmp(fixed.FromInt(10)) >= 0 {
			trace.TickA = tick
		}
		if trace.TickB < 0 && v.Position.Z.Sub(startZ).Cmp(fixed.FromInt(12)) >= 0 {
			trace.TickB = tick
		}
	}
	trace.FinalPosition = v.Position
	trace.FinalVelocity = v.Velocity
	trace.FinalUp = v.UpWS
	return trace
}

func traceFlatReverse() arcadeTrace {
	ground := FlatGround{Y: fixed.Zero}
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	arcadeSettleVehicle(&v, ground, 90)

	startZ := v.Position.Z
	trace := arcadeTrace{TickA: -1, TickB: -1}
	for tick := 1; tick <= 120; tick++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5).Neg()}
		v.Step(fixed.FromFraction(1, 60), ground)
		if v.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && v.Velocity.Dot(v.ForwardWS).Cmp(fixed.FromInt(-3)) <= 0 {
			trace.TickA = tick
		}
		if trace.TickB < 0 && startZ.Sub(v.Position.Z).Cmp(fixed.FromInt(4)) >= 0 {
			trace.TickB = tick
		}
	}
	trace.FinalPosition = v.Position
	trace.FinalVelocity = v.Velocity
	trace.FinalUp = v.UpWS
	return trace
}

func traceFlatBrake() arcadeTrace {
	ground := FlatGround{Y: fixed.Zero}
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	arcadeSettleVehicle(&v, ground, 90)
	arcadeStepVehicle(&v, ground, 120, VehicleInput{Throttle: fixed.FromFraction(4, 5)})

	speedBefore := arcadePlanarSpeed(v)
	trace := arcadeTrace{TickA: -1}
	for tick := 1; tick <= 45; tick++ {
		v.Input = VehicleInput{Brake: fixed.One}
		v.Step(fixed.FromFraction(1, 60), ground)
		if v.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && arcadePlanarSpeed(v).Cmp(speedBefore.Div(fixed.FromInt(2))) < 0 {
			trace.TickA = tick
		}
	}
	trace.FinalPosition = v.Position
	trace.FinalVelocity = v.Velocity
	trace.FinalUp = v.UpWS
	return trace
}

func traceFlatTurn() arcadeTrace {
	ground := FlatGround{Y: fixed.Zero}
	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	arcadeSettleVehicle(&v, ground, 90)
	arcadeStepVehicle(&v, ground, 90, VehicleInput{Throttle: fixed.FromFraction(3, 5)})

	startYaw := v.Yaw
	startX := v.Position.X
	trace := arcadeTrace{TickA: -1, TickB: -1}
	for tick := 1; tick <= 120; tick++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(1, 2), Steer: fixed.One}
		v.Step(fixed.FromFraction(1, 60), ground)
		if v.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && v.Yaw.Sub(startYaw).Abs().Cmp(fixed.FromFraction(35, 100)) >= 0 {
			trace.TickA = tick
		}
		if trace.TickB < 0 && v.Position.X.Sub(startX).Abs().Cmp(fixed.FromInt(4)) >= 0 {
			trace.TickB = tick
		}
	}
	trace.FinalPosition = v.Position
	trace.FinalVelocity = v.Velocity
	trace.FinalUp = v.UpWS
	return trace
}

func traceRampClimb() arcadeTrace {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(-18), fixed.FromInt(3), fixed.FromInt(-10))
	world.Vehicle.Yaw = fixedFromFloat(0.7853981633974483)
	world.Vehicle.UpdateBasisFromYaw()

	startY := world.Vehicle.Position.Y
	trace := arcadeTrace{TickA: -1, PeakYTick: -1, PeakY: world.Vehicle.Position.Y}
	for tick := 1; tick <= 220; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(7, 10)}
		world.Step(fixed.FromFraction(1, 60))
		if world.Vehicle.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && world.Vehicle.Position.Y.Sub(startY).Cmp(fixed.FromFraction(3, 2)) >= 0 {
			trace.TickA = tick
		}
		if world.Vehicle.Position.Y.Cmp(trace.PeakY) > 0 {
			trace.PeakY = world.Vehicle.Position.Y
			trace.PeakYTick = tick
		}
	}
	trace.FinalPosition = world.Vehicle.Position
	trace.FinalVelocity = world.Vehicle.Velocity
	trace.FinalUp = world.Vehicle.UpWS
	return trace
}

func traceRampDescend() arcadeTrace {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(5), fixed.FromInt(6), fixed.FromInt(2))
	world.Vehicle.Yaw = fixed.FromFraction(11, 10)
	world.Vehicle.UpdateBasisFromYaw()

	startY := world.Vehicle.Position.Y
	trace := arcadeTrace{TickA: -1, TickB: -1, TickC: -1}
	for tick := 1; tick <= 180; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(3, 5)}
		world.Step(fixed.FromFraction(1, 60))
		if world.Vehicle.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && startY.Sub(world.Vehicle.Position.Y).Cmp(fixed.One) >= 0 {
			trace.TickA = tick
		}
		if trace.TickB < 0 && world.Vehicle.Position.Y.Cmp(fixed.FromFraction(14, 10)) <= 0 {
			trace.TickB = tick
		}
		if trace.TickC < 0 && world.Vehicle.UpWS.Y.Cmp(fixed.FromFraction(49, 50)) >= 0 {
			trace.TickC = tick
		}
	}
	trace.FinalPosition = world.Vehicle.Position
	trace.FinalVelocity = world.Vehicle.Velocity
	trace.FinalUp = world.Vehicle.UpWS
	return trace
}

func traceRampSteer() arcadeTrace {
	world := NewArcadeCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(-10), fixed.FromInt(4), fixed.FromInt(-5))
	world.Vehicle.Yaw = fixedFromFloat(0.95)
	world.Vehicle.UpdateBasisFromYaw()

	startYaw := world.Vehicle.Yaw
	startX := world.Vehicle.Position.X
	trace := arcadeTrace{TickA: -1, TickB: -1}
	for tick := 1; tick <= 150; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(1, 2), Steer: fixed.One}
		world.Step(fixed.FromFraction(1, 60))
		if world.Vehicle.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && world.Vehicle.Yaw.Sub(startYaw).Abs().Cmp(fixed.FromFraction(1, 4)) >= 0 {
			trace.TickA = tick
		}
		if trace.TickB < 0 && world.Vehicle.Position.X.Sub(startX).Abs().Cmp(fixed.FromInt(2)) >= 0 {
			trace.TickB = tick
		}
	}
	trace.FinalPosition = world.Vehicle.Position
	trace.FinalVelocity = world.Vehicle.Velocity
	trace.FinalUp = world.Vehicle.UpWS
	return trace
}

func traceBuildingCollision() arcadeTrace {
	world := NewArcadeCityWorld()
	building := world.Map.Buildings[0]
	arcadePlaceVehicleForObstacleApproach(&world.Vehicle, building, fixed.FromFraction(1, 2))

	trace := arcadeTrace{TickA: -1}
	prevPos := world.Vehicle.Position
	for tick := 1; tick <= 180; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(4, 5)}
		world.Step(fixed.FromFraction(1, 60))
		if world.Vehicle.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && world.Vehicle.Position.Sub(prevPos).LengthSq().Cmp(fixed.FromFraction(1, 1000)) <= 0 {
			trace.TickA = tick
		}
		prevPos = world.Vehicle.Position
	}
	trace.FinalPosition = world.Vehicle.Position
	trace.FinalVelocity = world.Vehicle.Velocity
	trace.FinalUp = world.Vehicle.UpWS
	return trace
}

func traceGuardRailCollision() arcadeTrace {
	world := NewArcadeCityWorld()
	rail := world.Map.GuardRails[0]
	arcadePlaceVehicleForObstacleApproach(&world.Vehicle, rail, fixed.FromFraction(1, 2))

	trace := arcadeTrace{TickA: -1}
	prevPos := world.Vehicle.Position
	for tick := 1; tick <= 180; tick++ {
		world.Vehicle.Input = VehicleInput{Throttle: fixed.FromFraction(4, 5)}
		world.Step(fixed.FromFraction(1, 60))
		if world.Vehicle.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && world.Vehicle.Position.Sub(prevPos).LengthSq().Cmp(fixed.FromFraction(1, 1000)) <= 0 {
			trace.TickA = tick
		}
		prevPos = world.Vehicle.Position
	}
	trace.FinalPosition = world.Vehicle.Position
	trace.FinalVelocity = world.Vehicle.Velocity
	trace.FinalUp = world.Vehicle.UpWS
	return trace
}

func traceWallCollision() arcadeTrace {
	ground := FlatGround{Y: fixed.Zero}
	bounds := WallBounds{
		MinX: fixed.FromInt(-6),
		MaxX: fixed.FromInt(6),
		MinZ: fixed.FromInt(-6),
		MaxZ: fixed.FromInt(6),
	}

	v := NewArcadeVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	arcadeSettleVehicleWithWorld(&v, ground, bounds, nil, 90)

	trace := arcadeTrace{TickA: -1}
	maxCenterZ := bounds.MaxZ.Sub(v.Tuning.WheelBase.Div(fixed.FromInt(2)))
	for tick := 1; tick <= 180; tick++ {
		v.Input = VehicleInput{Throttle: fixed.FromFraction(4, 5)}
		v.Step(fixed.FromFraction(1, 60), ground)
		CollideVehicleWithWalls(&v.Vehicle, bounds)
		if v.OnGround {
			trace.GroundedTicks++
		}
		if trace.TickA < 0 && v.Position.Z.Cmp(maxCenterZ) >= 0 {
			trace.TickA = tick
		}
	}
	trace.FinalPosition = v.Position
	trace.FinalVelocity = v.Velocity
	trace.FinalUp = v.UpWS
	return trace
}
