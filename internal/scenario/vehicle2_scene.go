package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func Vehicle2ScenarioDefinitions() []ScenarioDefinition {
	return []ScenarioDefinition{
		NewVehicle2MMOGroundingBaselineScenario(),
		NewVehicle2MMOCurbSupportScenario(),
		NewVehicle2MMORoadSupportAndWallScenario(),
	}
}

func NewVehicle2MMOGroundingBaselineScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle2 MMO Grounding Baseline",
		Description: "A vehicle2 baseline scene for MMO-style support-driven grounding on flat road using sphere wheel probes, wheel collider support, and clamped correction.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle2MMOSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.VehicleUseWheelCollider = true
			state.VehicleWheelCorrectionClamp = fixed.FromFraction(1, 20)
			return state
		},
		Step: func(state *SceneState) {
			stepVehicle2SceneOnPlane(state, fixed.FromInt(24), fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
		},
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 baseline lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(6)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 baseline did not move far enough forward."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 baseline did not stay upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle2 baseline stayed grounded and stable on flat road.",
			}
		},
	}
}

func NewVehicle2MMOCurbSupportScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle2 MMO Curb Support",
		Description: "A vehicle2 MMO grounding scene that uses support-driven sphere wheel probes with wheel collider clamp across a curb bump.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle2MMOSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.VehicleUseWheelCollider = true
			state.VehicleWheelCorrectionClamp = fixed.FromFraction(1, 20)
			state.GroundTriangles = makeVehicle2CurbBumpTriangles()
			return state
		},
		Step: StepVehicle2MMOCurbSupportScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 curb support lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 curb support did not clear the curb bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 curb support did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle2 curb support stayed grounded through the curb bump.",
			}
		},
	}
}

func NewVehicle2MMORoadSupportAndWallScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Vehicle2 MMO Road Support And Wall",
		Description: "A vehicle2 MMO grounding scene where wheel support handles the road and curb bump while the chassis only resolves the hard wall obstacle ahead.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicle2MMOSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.VehicleUseWheelCollider = true
			state.VehicleWheelCorrectionClamp = fixed.FromFraction(1, 20)
			state.GroundTriangles = append(makeVehicle2CurbBumpTriangles(), makeVehicle2WallImpactTriangles()...)
			state.GroundBoxes = []geometry.AxisAlignedBoundingBox{makeVehicle2WallImpactBounds()}
			return state
		},
		Step: func(state *SceneState) {
			StepVehicle2MMOCurbSupportScene(state)
		},
		Check: func(state SceneState) ScenarioResult {
			if !state.VehicleEverHitWall {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 road-support-and-wall scene never hit the hard wall obstacle."}
			}
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 road-support-and-wall scene lost too many grounded probes."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 road-support-and-wall scene did not recover upright enough."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Vehicle2 road-support-and-wall scene did not drive far enough through the supported road section."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle2 road support handled the road while chassis collision handled the hard wall obstacle.",
			}
		},
	}
}
