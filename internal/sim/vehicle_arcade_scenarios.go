package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type ArcadeDebugScenario struct {
	ID            string
	Name          string
	Description   string
	DurationTicks int
	Setup         func(*ArcadeCityWorld)
	InputAt       func(tick int) VehicleInput
}

func ArcadeDebugScenarios() []ArcadeDebugScenario {
	return []ArcadeDebugScenario{
		{
			ID:            "flat_forward",
			Name:          "Flat Forward",
			Description:   "Accelerate on the lower avenue",
			DurationTicks: 150,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28)), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(4, 5)}
			},
		},
		{
			ID:            "flat_reverse",
			Name:          "Flat Reverse",
			Description:   "Reverse on the lower avenue",
			DurationTicks: 120,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28)), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(3, 5).Neg()}
			},
		},
		{
			ID:            "flat_brake",
			Name:          "Flat Brake",
			Description:   "Accelerate then full-brake on the lower avenue",
			DurationTicks: 165,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28)), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				if tick <= 120 {
					return VehicleInput{Throttle: fixed.FromFraction(4, 5)}
				}
				return VehicleInput{Brake: fixed.One}
			},
		},
		{
			ID:            "flat_turn",
			Name:          "Flat Turn",
			Description:   "Build speed then steer on flat road",
			DurationTicks: 210,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28)), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				if tick <= 90 {
					return VehicleInput{Throttle: fixed.FromFraction(3, 5)}
				}
				return VehicleInput{
					Throttle: fixed.FromFraction(1, 2),
					Steer:    fixed.One,
				}
			},
		},
		{
			ID:            "ramp_climb",
			Name:          "Ramp Climb",
			Description:   "Climb onto the overpass",
			DurationTicks: 220,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(
					w,
					geom.V3(fixed.FromInt(-18), fixed.FromInt(3), fixed.FromInt(-10)),
					fixed.FromFraction(785398, 1000000),
				)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(7, 10)}
			},
		},
		{
			ID:            "ramp_descend",
			Name:          "Ramp Descend",
			Description:   "Drive down from the overpass toward flat road",
			DurationTicks: 180,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(
					w,
					geom.V3(fixed.FromInt(5), fixed.FromInt(6), fixed.FromInt(2)),
					fixed.FromFraction(11, 10),
				)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(3, 5)}
			},
		},
		{
			ID:            "ramp_steer",
			Name:          "Ramp Steer",
			Description:   "Steer while traversing the overpass slope",
			DurationTicks: 150,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(
					w,
					geom.V3(fixed.FromInt(-10), fixed.FromInt(4), fixed.FromInt(-5)),
					fixed.FromFraction(95, 100),
				)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{
					Throttle: fixed.FromFraction(1, 2),
					Steer:    fixed.One,
				}
			},
		},
		{
			ID:            "coast_turn",
			Name:          "Coast Turn",
			Description:   "Accelerate first, then steer while coasting",
			DurationTicks: 330,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, w.Map.SpawnPosition, w.Map.SpawnYaw)
			},
			InputAt: func(tick int) VehicleInput {
				switch {
				case tick <= 120:
					return VehicleInput{}
				case tick <= 210:
					return VehicleInput{Throttle: fixed.FromFraction(3, 5)}
				default:
					return VehicleInput{Steer: fixed.One}
				}
			},
		},
		{
			ID:            "under_bridge",
			Name:          "Under Bridge",
			Description:   "Drive through the underpass without snapping onto the deck",
			DurationTicks: 300,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28)), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				if tick < 30 {
					return VehicleInput{}
				}
				return VehicleInput{Throttle: fixed.FromFraction(3, 5)}
			},
		},
		{
			ID:            "off_ramp_recovery",
			Name:          "Off Ramp Recovery",
			Description:   "Descend from the overpass and watch recovery to flat ground",
			DurationTicks: 435,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioSetPose(
					w,
					geom.V3(fixed.FromInt(5), fixed.FromInt(6), fixed.FromInt(2)),
					fixed.FromFraction(11, 10),
				)
			},
			InputAt: func(tick int) VehicleInput {
				if tick <= 360 {
					return VehicleInput{Throttle: fixed.FromFraction(3, 5)}
				}
				return VehicleInput{Throttle: fixed.FromFraction(1, 5)}
			},
		},
		{
			ID:            "building_collision",
			Name:          "Building Collision",
			Description:   "Drive directly into the first building footprint",
			DurationTicks: 180,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioPlaceVehicleForObstacleApproach(&w.Vehicle, w.Map.Buildings[0], fixed.FromFraction(1, 2))
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(4, 5)}
			},
		},
		{
			ID:            "guardrail_collision",
			Name:          "Guard Rail Collision",
			Description:   "Drive directly into the first overpass guard rail",
			DurationTicks: 180,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				arcadeScenarioPlaceVehicleForObstacleApproach(&w.Vehicle, w.Map.GuardRails[0], fixed.FromFraction(1, 2))
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(4, 5)}
			},
		},
		{
			ID:            "wall_collision",
			Name:          "Wall Collision",
			Description:   "Run into a shrunken test wall bound",
			DurationTicks: 180,
			Setup: func(w *ArcadeCityWorld) {
				arcadeScenarioResetWorld(w)
				w.Map.Bounds = WallBounds{
					MinX: fixed.FromInt(-6),
					MaxX: fixed.FromInt(6),
					MinZ: fixed.FromInt(-6),
					MaxZ: fixed.FromInt(6),
				}
				arcadeScenarioSetPose(w, geom.V3(fixed.Zero, fixed.FromInt(2), fixed.Zero), fixed.Zero)
			},
			InputAt: func(tick int) VehicleInput {
				return VehicleInput{Throttle: fixed.FromFraction(4, 5)}
			},
		},
	}
}

func arcadeScenarioResetWorld(w *ArcadeCityWorld) {
	if w == nil {
		return
	}
	*w = NewArcadeCityWorld()
}

func arcadeScenarioSetPose(w *ArcadeCityWorld, pos geom.Vec3, yaw fixed.Fixed) {
	if w == nil {
		return
	}
	w.Vehicle.Position = pos
	w.Vehicle.Velocity = geom.Zero()
	w.Vehicle.Yaw = yaw
	w.Vehicle.YawVelocity = fixed.Zero
	w.Vehicle.Input = VehicleInput{}
	w.Vehicle.UpdateBasisFromYaw()
}

func arcadeScenarioPlaceVehicleForObstacleApproach(v *ArcadeVehicle, obstacle CityObstacle, clearance fixed.Fixed) {
	if v == nil {
		return
	}

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
		v.Yaw = fixed.FromFraction(1570796, 1000000)
	}

	v.Velocity = geom.Zero()
	v.YawVelocity = fixed.Zero
	v.Input = VehicleInput{}
	v.UpdateBasisFromYaw()
}
