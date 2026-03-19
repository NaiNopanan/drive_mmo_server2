package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func Vehicle3ScenarioDefinitions() []ScenarioDefinition {
	return []ScenarioDefinition{
		NewVehicle3RayCityManualScenario(),
	}
}

func NewVehicle3RayCityManualScenario() ScenarioDefinition {
	const scenarioTicks = 36000

	return ScenarioDefinition{
		Name:        "Vehicle3 RayCity Manual Tune",
		Description: "A realtime tuning scene for driving vehicle3 by hand and adjusting arcade parameters live.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle3RayCitySceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(29, 20), fixed.FromInt(-34)),
				physics.IdentityQuaternion(),
			)
			state.VehicleDriveForce = fixed.FromInt(56)
			state.GroundTriangles = makeVehicle3RayCityMapTriangles()
			state.GroundBoxes = makeVehicle3RayCityMapBoxes()
			return state
		},
		Step: StepVehicle3RayCityManualScene,
		Check: func(state SceneState) ScenarioResult {
			return ScenarioResult{
				Status:  Running,
				Message: "Manual tuning scene",
			}
		},
	}
}

func NewVehicle3RayCityGroundingBaselineScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle3 RayCity Grounding Baseline",
		Description: "A fresh vehicle3 support-first baseline scene tuned for MMO-style ride height, wheel grounding, and stable forward cruise on flat road.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicle3RayCitySceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(29, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
		},
		Step: StepVehicle3RayCityCruiseScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 baseline lost too many grounded wheels."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 baseline did not move far enough forward."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 baseline did not stay upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle3 baseline stayed grounded and cruised flat road with MMO-style support.",
			}
		},
	}
}

func NewVehicle3RayCityCurbSupportScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle3 RayCity Curb Support",
		Description: "A vehicle3 curb scene where wheel support keeps the chassis riding over a curb bump instead of sinking into it.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle3RayCitySceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(29, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicle2CurbBumpTriangles()
			return state
		},
		Step: StepVehicle3RayCityCurbScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 curb scene lost too many grounded wheels."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(10)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 curb scene did not clear the supported bump."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 10)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 curb scene rode too low over the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 curb scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle3 stayed supported over the curb bump without sinking the chassis.",
			}
		},
	}
}

func NewVehicle3RayCityRoadSupportAndWallScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Vehicle3 RayCity Road Support And Wall",
		Description: "A vehicle3 road scene where the wheels support the road profile while the chassis still collides against a hard wall ahead.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle3RayCitySceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(29, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = append(makeVehicle2CurbBumpTriangles(), makeVehicle2WallImpactTriangles()...)
			state.GroundBoxes = []geometry.AxisAlignedBoundingBox{makeVehicle2WallImpactBounds()}
			return state
		},
		Step: StepVehicle3RayCityCurbScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.VehicleEverHitWall {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 wall scene never hit the hard wall obstacle."}
			}
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 wall scene lost too many grounded wheels."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 wall scene did not recover upright enough."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 wall scene did not advance far enough through the supported road section."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle3 kept wheel support on the road and let the chassis handle the wall impact.",
			}
		},
	}
}

func NewVehicle3RayCityArcadeTurnScenario() ScenarioDefinition {
	const scenarioTicks = 260

	return ScenarioDefinition{
		Name:        "Vehicle3 RayCity Arcade Turn",
		Description: "A vehicle3 arcade turn scene that uses front-weighted steering, rear drive, and anti-roll support to carve a stable MMO-style arc.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicle3RayCitySceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(29, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
		},
		Step: StepVehicle3RayCityTurnScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleFrontGroundedProbeCount < 1 || state.VehicleRearGroundedProbeCount < 1 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 turn scene lost an entire axle worth of grounding."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(4)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 turn scene did not carry enough forward speed."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.FromFraction(3, 5)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 turn scene did not build enough sideways arc."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.FromInt(8)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 turn scene over-rotated sideways too aggressively."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle3 turn scene did not stay upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle3 carved a stable arcade turn while keeping its support envelope.",
			}
		},
	}
}
