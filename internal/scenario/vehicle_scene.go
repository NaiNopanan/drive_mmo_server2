package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

var vehicleSceneWheelLocalOffsets = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-3, 4), fixed.FromFraction(-7, 20), fixed.FromFraction(6, 5)),
	geometry.NewVector3(fixed.FromFraction(3, 4), fixed.FromFraction(-7, 20), fixed.FromFraction(6, 5)),
	geometry.NewVector3(fixed.FromFraction(-3, 4), fixed.FromFraction(-7, 20), fixed.FromFraction(-6, 5)),
	geometry.NewVector3(fixed.FromFraction(3, 4), fixed.FromFraction(-7, 20), fixed.FromFraction(-6, 5)),
}

var vehicleSceneWheelLocalOffsetsEdge = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-4, 5), fixed.FromFraction(-7, 20), fixed.FromFraction(7, 5)),
	geometry.NewVector3(fixed.FromFraction(4, 5), fixed.FromFraction(-7, 20), fixed.FromFraction(7, 5)),
	geometry.NewVector3(fixed.FromFraction(-4, 5), fixed.FromFraction(-7, 20), fixed.FromFraction(-7, 5)),
	geometry.NewVector3(fixed.FromFraction(4, 5), fixed.FromFraction(-7, 20), fixed.FromFraction(-7, 5)),
}

var vehicleSceneWheelLocalOffsetsCorner = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.FromFraction(-7, 20), fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.FromFraction(-7, 20), fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.FromFraction(-7, 20), fixed.FromFraction(-8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.FromFraction(-7, 20), fixed.FromFraction(-8, 5)),
}

var vehicleSceneWheelLocalOffsetsSphere = []geometry.Vector3{
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.Zero, fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.Zero, fixed.FromFraction(8, 5)),
	geometry.NewVector3(fixed.FromFraction(-9, 10), fixed.Zero, fixed.FromFraction(-8, 5)),
	geometry.NewVector3(fixed.FromFraction(9, 10), fixed.Zero, fixed.FromFraction(-8, 5)),
}

func VehicleScenarioDefinitions() []ScenarioDefinition {
	return []ScenarioDefinition{
		NewVehicleArcadeDropToGroundScenario(),
		NewVehicleArcadeIdleSuspensionScenario(),
		NewVehicleArcadeStraightDriveScenario(),
		NewVehicleArcadeTurnScenario(),
		NewVehicleArcadeWallImpactScenario(),
		NewVehicleArcadeUphillSlopeScenario(),
		NewVehicleArcadeSteerOnSlopeScenario(),
		NewVehicleArcadeDownhillSlopeScenario(),
		NewVehicleArcadeSteerDownhillSlopeScenario(),
		NewVehicleArcadeSlopeCrestTransitionScenario(),
		NewVehicleArcadeWallRecoverScenario(),
		NewVehicleArcadeCurbBumpScenario(),
		NewVehicleArcadeUphillSlopeEdgeProbesScenario(),
		NewVehicleArcadeCurbBumpEdgeProbesScenario(),
		NewVehicleArcadeCurbBumpWheelRadiusScenario(),
		NewVehicleArcadeCurbBumpSphereWheelProbesScenario(),
		NewVehicleArcadeCurbBumpWheelColliderScenario(),
		NewVehicleArcadeCurbBumpClampedWheelColliderScenario(),
	}
}

func NewVehicleArcadeDropToGroundScenario() ScenarioDefinition {
	const scenarioTicks = 300

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Drop To Ground",
		Description: "A single arcade-style rigid chassis with four wheel probes falls onto flat ground and should settle upright without tunneling.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromInt(4), fixed.Zero),
				physics.NewQuaternionFromEulerXYZ(
					fixed.FromFraction(52359, 100000),
					fixed.Zero,
					fixed.FromFraction(17453, 100000),
				),
			)
		},
		Step:  StepVehicleArcadeDropToGroundScene,
		Check: vehicleArcadeDropCheck,
	}
}

func NewVehicleArcadeIdleSuspensionScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Idle Suspension",
		Description: "A single arcade-style rigid chassis starts near ride height and should stay upright, grounded, and stable on flat ground.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
		},
		Step: StepVehicleArcadeDropToGroundScene,
		Check: func(state SceneState) ScenarioResult {
			if result := vehicleArcadeBaseCheck(state); result.Status != Passed {
				return result
			}
			if state.VehicleGroundedProbeCount < 3 {
				return ScenarioResult{Status: Failed, Message: "Idle suspension scene ended without at least three grounded probes."}
			}
			if state.VehicleAverageCompression.Cmp(fixed.FromFraction(1, 20)) < 0 || state.VehicleAverageCompression.Cmp(fixed.FromFraction(3, 5)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Idle suspension scene ended outside the expected compression window."}
			}
			if state.VehicleChassis.Motion.Velocity.Length().Cmp(fixed.FromFraction(1, 8)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Idle suspension scene still had too much linear motion at the end."}
			}
			if state.VehicleChassis.AngularVelocity.Length().Cmp(fixed.FromFraction(1, 6)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Idle suspension scene still had too much angular motion at the end."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle suspension stayed grounded and stable at idle ride height.",
			}
		},
	}
}

func NewVehicleArcadeStraightDriveScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Straight Drive",
		Description: "A single arcade-style rigid chassis starts near ride height and accelerates straight ahead on flat ground while staying upright and stable.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
		},
		Step: StepVehicleArcadeStraightDriveScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Straight-drive scene lost too many grounded probes."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Straight-drive scene did not stay upright enough."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(6)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Straight-drive scene did not move far enough forward."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.FromInt(-1)) < 0 || state.VehicleChassis.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Straight-drive scene drifted too far sideways."}
			}
			if state.VehicleChassis.AngularVelocity.Length().Cmp(fixed.FromFraction(1, 2)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Straight-drive scene accumulated too much angular motion."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle accelerated straight ahead while staying upright on flat ground.",
			}
		},
	}
}

func NewVehicleArcadeTurnScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Turn",
		Description: "A single arcade-style rigid chassis accelerates while steering on flat ground and should trace a stable turn without tipping over.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
		},
		Step: StepVehicleArcadeTurnScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Turn scene lost too many grounded probes."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Turn scene did not stay upright enough."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(4)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Turn scene did not move forward enough."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.One) < 0 {
				return ScenarioResult{Status: Failed, Message: "Turn scene did not arc sideways enough."}
			}
			if state.VehicleChassis.AngularVelocity.Y.Cmp(fixed.FromFraction(1, 10)) <= 0 {
				return ScenarioResult{Status: Failed, Message: "Turn scene did not build enough steering yaw."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle drove a stable turning arc on flat ground.",
			}
		},
	}
}

func NewVehicleArcadeWallImpactScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Wall Impact",
		Description: "A single arcade-style rigid chassis accelerates straight into a wall and should collide without tunneling through it.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = append(state.GroundTriangles, makeVehicleWallImpactTriangles()...)
			state.GroundBoxes = []geometry.AxisAlignedBoundingBox{makeVehicleWallImpactBounds()}
			return state
		},
		Step: StepVehicleArcadeWallImpactScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Wall-impact scene lost too many grounded probes."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-impact scene did not stay upright enough."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromFraction(17, 2)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-impact scene tunneled through the wall."}
			}
			if !state.LastContact.Hit || state.LastContact.Normal.Z.Cmp(fixed.FromFraction(-9, 10)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-impact scene never produced a forward wall contact."}
			}
			if state.VehicleChassis.Motion.Velocity.Z.Cmp(fixed.FromInt(1)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-impact scene kept too much forward speed after impact."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle hit the wall and stayed on the impact side without tunneling.",
			}
		},
	}
}

func NewVehicleArcadeUphillSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 260

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Uphill Slope",
		Description: "A single arcade-style rigid chassis accelerates up a gentle slope while staying grounded and aligned to the road surface.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-6)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleUphillSlopeTriangles()
			return state
		},
		Step: StepVehicleArcadeUphillSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.Zero) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope scene did not climb far enough forward."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope scene did not gain enough height while climbing."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(47, 50)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope scene did not stay aligned to the slope."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle climbed the slope while staying grounded and aligned to the surface.",
			}
		},
	}
}

func NewVehicleArcadeSteerOnSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 260

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Steer On Slope",
		Description: "A single arcade-style rigid chassis accelerates and steers while climbing a gentle slope, staying grounded and aligned to the road surface.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-6)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleUphillSlopeTriangles()
			return state
		},
		Step: StepVehicleArcadeSteerOnSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromFraction(-1, 2)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene did not climb far enough forward."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene did not gain enough height while climbing."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.FromFraction(3, 2)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene did not arc sideways enough."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(47, 50)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene did not stay aligned to the slope."}
			}
			if state.VehicleChassis.AngularVelocity.Y.Cmp(fixed.FromFraction(1, 10)) <= 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-on-slope scene did not build enough steering yaw."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle climbed and steered across the slope while staying grounded and aligned to the surface.",
			}
		},
	}
}

func NewVehicleArcadeDownhillSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Downhill Slope",
		Description: "A single arcade-style rigid chassis starts high on a gentle slope and drives downhill while staying grounded and aligned to the road surface.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(51, 20), fixed.FromInt(6)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleUphillSlopeTriangles()
			return state
		},
		Step: StepVehicleArcadeDownhillSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Downhill slope scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Downhill slope scene did not descend far enough forward along the slope."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(37, 20)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Downhill slope scene did not lose enough height while descending."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(47, 50)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Downhill slope scene did not stay aligned to the slope."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle descended the slope while staying grounded and aligned to the surface.",
			}
		},
	}
}

func NewVehicleArcadeSteerDownhillSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Steer Downhill",
		Description: "A single arcade-style rigid chassis drives and steers downhill while staying grounded and aligned to the road surface.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(51, 20), fixed.FromInt(6)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleUphillSlopeTriangles()
			return state
		},
		Step: StepVehicleArcadeSteerDownhillSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene did not descend far enough downhill."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(37, 20)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene did not lose enough height while descending."}
			}
			if state.VehicleChassis.Motion.Position.X.Cmp(fixed.One.Neg()) > 0 && state.VehicleChassis.Motion.Position.X.Cmp(fixed.One) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene did not arc sideways enough."}
			}
			if state.VehicleChassis.AngularVelocity.Y.Cmp(fixed.FromFraction(1, 20)) <= 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene did not build enough steering yaw."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(47, 50)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Steer-downhill scene did not stay aligned to the slope."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle descended and steered across the slope while staying grounded and aligned to the surface.",
			}
		},
	}
}

func NewVehicleArcadeSlopeCrestTransitionScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Slope Crest Transition",
		Description: "A single arcade-style rigid chassis climbs a slope and transitions onto a flat crest while staying grounded and stable through the surface change.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-6)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleSlopeCrestTriangles()
			return state
		},
		Step: StepVehicleArcadeSlopeCrestTransitionScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Slope-crest scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Slope-crest scene did not reach the flat crest."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 5)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Slope-crest scene did not stay high enough on the crest."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Slope-crest scene did not recover to a stable crest alignment."}
			}
			if state.VehicleChassis.AngularVelocity.Length().Cmp(fixed.FromFraction(3, 4)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Slope-crest scene accumulated too much angular motion across the transition."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle climbed onto the crest and stayed grounded through the slope-to-flat transition.",
			}
		},
	}
}

func NewVehicleArcadeWallRecoverScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Wall Recover",
		Description: "A single arcade-style rigid chassis hits a wall, backs away while steering, and should recover to a stable drivable posture on flat ground.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.Zero),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = append(state.GroundTriangles, makeVehicleWallImpactTriangles()...)
			state.GroundBoxes = []geometry.AxisAlignedBoundingBox{makeVehicleWallImpactBounds()}
			return state
		},
		Step: StepVehicleArcadeWallRecoverScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.VehicleEverHitWall {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene never actually reached the wall."}
			}
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene lost too many grounded probes."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene did not return to a stable upright posture."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(7)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene did not back away from the wall enough."}
			}
			if state.VehicleChassis.Motion.Velocity.Length().Cmp(fixed.FromInt(1)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene still had too much speed at the end."}
			}
			if state.VehicleChassis.AngularVelocity.Length().Cmp(fixed.FromFraction(1, 2)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Wall-recover scene still had too much angular motion at the end."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle hit the wall, backed away, and recovered to a stable drivable posture.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump",
		Description: "A single arcade-style rigid chassis drives over a short curb-like bump and should stay grounded, upright, and stable after the disturbance.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneState(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
			)
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump scene did not clear the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump scene did not recover to a stable upright posture."}
			}
			if state.VehicleChassis.AngularVelocity.Length().Cmp(fixed.FromFraction(3, 5)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump scene accumulated too much angular motion."}
			}
			if state.VehicleAverageCompression.Cmp(fixed.FromFraction(7, 10)) > 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump scene ended with too much suspension compression."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle drove over the curb bump and returned to a stable grounded posture.",
			}
		},
	}
}

func NewVehicleArcadeUphillSlopeEdgeProbesScenario() ScenarioDefinition {
	const scenarioTicks = 260

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Uphill Slope Edge Probes",
		Description: "A comparison scene for uphill slope handling using wheel probes moved closer to the chassis corners.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-6)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsEdge,
			)
			state.GroundTriangles = makeVehicleUphillSlopeTriangles()
			return state
		},
		Step: StepVehicleArcadeUphillSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope edge-probes scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.Zero) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope edge-probes scene did not climb far enough forward."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope edge-probes scene did not gain enough height while climbing."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Uphill slope edge-probes scene did not stay upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle climbed the slope with edge probes while staying grounded and aligned.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpEdgeProbesScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump Edge Probes",
		Description: "A comparison scene for curb-bump handling using wheel probes placed at the chassis lower corners.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsCorner,
			)
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump edge-probes scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump edge-probes scene did not clear the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump edge-probes scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle cleared the curb bump with edge probes and returned to a stable posture.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpWheelRadiusScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump Wheel Radius Offset",
		Description: "A comparison scene for curb-bump handling using corner probes plus a wheel-radius offset to keep the chassis off the bump.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsCorner,
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpWheelRadiusScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-radius scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-radius scene did not clear the bump."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(17, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-radius scene rode too low over the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-radius scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle cleared the curb bump with corner probes and wheel-radius offset while staying stable.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpSphereWheelProbesScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump Sphere Wheel Probes",
		Description: "A comparison scene for curb-bump handling using sphere-style wheel probes with the wheel center raised up from the chassis underside.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsSphere,
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpWheelRadiusScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump sphere-wheel scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump sphere-wheel scene did not clear the bump."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump sphere-wheel scene rode too low over the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump sphere-wheel scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle cleared the curb bump with sphere-style wheel probes and returned to a stable posture.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpWheelColliderScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump Wheel Collider",
		Description: "A comparison scene for curb-bump handling using sphere-style wheel probes plus per-wheel collider resolution against the road surface.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsSphere,
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.VehicleUseWheelCollider = true
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpWheelRadiusScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-collider scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-collider scene did not clear the bump."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(19, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-collider scene rode too low over the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump wheel-collider scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle cleared the curb bump with sphere wheel probes and wheel-collider support while staying stable.",
			}
		},
	}
}

func NewVehicleArcadeCurbBumpClampedWheelColliderScenario() ScenarioDefinition {
	const scenarioTicks = 320

	return ScenarioDefinition{
		Name:        "Vehicle Arcade Curb Bump Clamped Wheel Collider",
		Description: "A comparison scene for curb-bump handling using wheel collider support with clamped per-tick correction to reduce snap-pop artifacts.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			state := makeVehicleArcadeSceneStateWithProbeOffsets(
				geometry.NewVector3(fixed.Zero, fixed.FromFraction(21, 20), fixed.FromInt(-8)),
				physics.IdentityQuaternion(),
				vehicleSceneWheelLocalOffsetsSphere,
			)
			state.VehicleWheelRadius = fixed.FromFraction(1, 4)
			state.VehicleUseWheelCollider = true
			state.VehicleWheelCorrectionClamp = fixed.FromFraction(1, 20)
			state.GroundTriangles = makeVehicleCurbBumpTriangles()
			return state
		},
		Step: StepVehicleArcadeCurbBumpWheelRadiusScene,
		Check: func(state SceneState) ScenarioResult {
			if state.VehicleGroundedProbeCount < 2 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump clamped wheel-collider scene lost too many grounded probes."}
			}
			if state.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump clamped wheel-collider scene did not clear the bump."}
			}
			if state.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 20)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump clamped wheel-collider scene rode too low over the bump."}
			}
			if state.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
				return ScenarioResult{Status: Failed, Message: "Curb-bump clamped wheel-collider scene did not recover upright enough."}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Vehicle cleared the curb bump with clamped wheel-collider support and stayed stable.",
			}
		},
	}
}

func makeVehicleArcadeSceneState(position geometry.Vector3, orientation physics.Quaternion) SceneState {
	return makeVehicleArcadeSceneStateWithProbeOffsets(position, orientation, vehicleSceneWheelLocalOffsets)
}

func makeVehicleArcadeSceneStateWithProbeOffsets(position geometry.Vector3, orientation physics.Quaternion, probeOffsets []geometry.Vector3) SceneState {
	chassis := physics.NewRigidBoxBody3D(
		fixed.FromInt(8),
		geometry.NewVector3(fixed.FromFraction(9, 10), fixed.FromFraction(7, 20), fixed.FromFraction(8, 5)),
		position,
	)
	chassis.Restitution = fixed.Zero
	chassis.Orientation = orientation
	if len(probeOffsets) == 0 {
		probeOffsets = vehicleSceneWheelLocalOffsets
	}
	clonedOffsets := append([]geometry.Vector3(nil), probeOffsets...)

	return SceneState{
		VehicleChassis:              chassis,
		VehicleProbeLocalOffsets:    clonedOffsets,
		VehicleWheelRadius:          fixed.Zero,
		VehicleWheelProbes:          make([]VehicleWheelProbeState, len(clonedOffsets)),
		VehicleWheelCorrectionClamp: fixed.Zero,
		GroundTriangles:             makeFlatGroundTriangles(),
	}
}

func vehicleArcadeBaseCheck(state SceneState) ScenarioResult {
	if state.VehicleChassis.Motion.Position.Y.Cmp(state.VehicleChassis.HalfExtents.Y.Sub(fixed.FromFraction(1, 20))) < 0 {
		return ScenarioResult{Status: Failed, Message: "Vehicle chassis tunneled below the ground plane."}
	}
	if state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) < 0 {
		return ScenarioResult{Status: Failed, Message: "Vehicle chassis did not recover to an upright orientation."}
	}
	if !state.VehicleSettled {
		return ScenarioResult{Status: Failed, Message: "Vehicle never settled on flat ground."}
	}
	return ScenarioResult{Status: Passed}
}

func vehicleArcadeDropCheck(state SceneState) ScenarioResult {
	if result := vehicleArcadeBaseCheck(state); result.Status != Passed {
		return result
	}
	if state.VehicleGroundedProbeCount < 2 {
		return ScenarioResult{Status: Failed, Message: "Vehicle did not land on the ground with enough grounded wheel probes."}
	}
	return ScenarioResult{
		Status:  Passed,
		Message: "Vehicle chassis landed, compressed the probes, and settled upright on flat ground.",
	}
}

func StepVehicleArcadeDropToGroundScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.Zero, fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
}

func StepVehicleArcadeStraightDriveScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(20), fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
}

func StepVehicleArcadeTurnScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(18), fixed.FromInt(8), geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
}

func StepVehicleArcadeWallImpactScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(20), fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
}

func StepVehicleArcadeUphillSlopeScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(60), fixed.Zero, geometry.ZeroVector3(), vehicleUphillPlaneNormal())
}

func StepVehicleArcadeSteerOnSlopeScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(58), fixed.FromInt(10), geometry.ZeroVector3(), vehicleUphillPlaneNormal())
}

func StepVehicleArcadeDownhillSlopeScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(-18), fixed.Zero, geometry.ZeroVector3(), vehicleUphillPlaneNormal())
}

func StepVehicleArcadeSteerDownhillSlopeScene(state *SceneState) {
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(-12), fixed.FromInt(14), geometry.ZeroVector3(), vehicleUphillPlaneNormal())
}

func StepVehicleArcadeSlopeCrestTransitionScene(state *SceneState) {
	planePoint, planeNormal := vehicleSlopeCrestPlane(bodyPositionZ(state))
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(62), fixed.Zero, planePoint, planeNormal)
}

func StepVehicleArcadeWallRecoverScene(state *SceneState) {
	if state == nil {
		return
	}

	switch {
	case !state.VehicleEverHitWall && state.Tick < 220:
		stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(20), fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	case state.Tick < 280:
		stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(-16), fixed.FromInt(10), geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	default:
		stepVehicleArcadeSceneOnPlane(state, fixed.Zero, fixed.Zero, geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	}
}

func StepVehicleArcadeCurbBumpScene(state *SceneState) {
	planePoint, planeNormal := vehicleCurbBumpPlane(bodyPositionZ(state))
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(40), fixed.Zero, planePoint, planeNormal)
}

func StepVehicleArcadeCurbBumpWheelRadiusScene(state *SceneState) {
	planePoint, planeNormal := vehicleCurbBumpPlane(bodyPositionZ(state))
	stepVehicleArcadeSceneOnPlane(state, fixed.FromInt(40), fixed.Zero, planePoint, planeNormal)
}

func stepVehicleArcadeSceneOnPlane(state *SceneState, driveForce, steerTorque fixed.Fixed, planePoint, planeNormal geometry.Vector3) {
	if state == nil {
		return
	}
	if planeNormal.LengthSquared() == fixed.Zero {
		planeNormal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	planeNormal = planeNormal.Normalize()

	body := &state.VehicleChassis
	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.VehicleGroundedProbeCount = 0
	state.VehicleAverageCompression = fixed.Zero
	state.VehicleSettled = false
	if len(state.VehicleProbeLocalOffsets) == 0 {
		state.VehicleProbeLocalOffsets = append([]geometry.Vector3(nil), vehicleSceneWheelLocalOffsets...)
	}
	if len(state.VehicleWheelProbes) != len(state.VehicleProbeLocalOffsets) {
		state.VehicleWheelProbes = make([]VehicleWheelProbeState, len(state.VehicleProbeLocalOffsets))
	}

	const (
		restLengthNum    = 7
		restLengthDen    = 10
		springNum        = 42
		springDen        = 1
		damperNum        = 6
		damperDen        = 1
		wheelColliderNum = 90
		wheelColliderDen = 1
	)
	restLength := fixed.FromFraction(restLengthNum, restLengthDen)
	springStrength := fixed.FromFraction(springNum, springDen)
	damperStrength := fixed.FromFraction(damperNum, damperDen)
	wheelColliderStrength := fixed.FromFraction(wheelColliderNum, wheelColliderDen)

	for index, localOffset := range state.VehicleProbeLocalOffsets {
		probe := VehicleWheelProbeState{LocalOffset: localOffset}
		worldOffset := body.Orientation.RotateVector(localOffset)
		probe.WorldPosition = body.Motion.Position.Add(worldOffset)
		centerDistance := probe.WorldPosition.Sub(planePoint).Dot(planeNormal)
		if state.VehicleUseWheelCollider && state.VehicleWheelRadius.Cmp(fixed.Zero) > 0 && centerDistance.Cmp(state.VehicleWheelRadius) < 0 {
			penetration := state.VehicleWheelRadius.Sub(centerDistance)
			if state.VehicleWheelCorrectionClamp.Cmp(fixed.Zero) > 0 && penetration.Cmp(state.VehicleWheelCorrectionClamp) > 0 {
				penetration = state.VehicleWheelCorrectionClamp
			}
			applyForceAtRigidBoxPoint(body, probe.WorldPosition, planeNormal.Scale(penetration.Mul(wheelColliderStrength)), physics.DefaultTimeStep)
			body.Motion.Position = body.Motion.Position.Add(planeNormal.Scale(penetration))
			probe.WorldPosition = probe.WorldPosition.Add(planeNormal.Scale(penetration))
			centerDistance = state.VehicleWheelRadius
		}
		suspensionLength := centerDistance.Sub(state.VehicleWheelRadius)
		if suspensionLength.Cmp(restLength) <= 0 {
			compression := restLength.Sub(suspensionLength)
			if compression.Cmp(fixed.Zero) < 0 {
				compression = fixed.Zero
			}
			pointVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(worldOffset))
			dampingForce := pointVelocity.Dot(planeNormal).Mul(damperStrength)
			forceMagnitude := compression.Mul(springStrength).Sub(dampingForce)
			if forceMagnitude.Cmp(fixed.Zero) < 0 {
				forceMagnitude = fixed.Zero
			}
			applyForceAtRigidBoxPoint(body, probe.WorldPosition, planeNormal.Scale(forceMagnitude), physics.DefaultTimeStep)
			probe.Grounded = true
			probe.Compression = compression
			probe.ContactPoint = probe.WorldPosition.Sub(planeNormal.Scale(suspensionLength.Add(state.VehicleWheelRadius)))
			probe.ContactNormal = planeNormal
			state.VehicleGroundedProbeCount++
			state.VehicleAverageCompression = state.VehicleAverageCompression.Add(compression)
		}
		state.VehicleWheelProbes[index] = probe
	}

	if state.VehicleGroundedProbeCount > 0 {
		state.VehicleAverageCompression = state.VehicleAverageCompression.Div(fixed.FromInt(int64(state.VehicleGroundedProbeCount)))
	}
	if driveForce.Cmp(fixed.Zero) != 0 && state.VehicleGroundedProbeCount >= 2 {
		forward := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One))
		forward = forward.Sub(planeNormal.Scale(forward.Dot(planeNormal)))
		if forward.LengthSquared() != fixed.Zero {
			physics.ApplyForce(&body.Motion, forward.Normalize().Scale(driveForce))
		}
	}
	if steerTorque.Cmp(fixed.Zero) != 0 && state.VehicleGroundedProbeCount >= 2 {
		applyTorqueToRigidBox(body, geometry.NewVector3(fixed.Zero, steerTorque, fixed.Zero), physics.DefaultTimeStep)
	}

	applyUprightAssist(body, physics.DefaultTimeStep, planeNormal)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(99, 100))
	body.AngularVelocity = body.AngularVelocity.Scale(fixed.FromFraction(47, 50))

	result := physics.StepRigidBoxBody3DWithGravityAndPlaneOverride(
		body,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		planePoint,
		planeNormal,
		fixed.Zero,
	)
	body.Motion.Velocity = body.Motion.Velocity.Scale(fixed.FromFraction(99, 100))
	if result.HadContact {
		state.LastContact = result.LastContact
		state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
		state.EverTouchedGround = true
	}
	resolveVehicleWallContacts(state, body)

	bodyUp := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	state.VehicleUprightDot = bodyUp.Dot(planeNormal)
	if state.VehicleUprightDot.Cmp(fixed.One) > 0 {
		state.VehicleUprightDot = fixed.One
	}
	if state.VehicleGroundedProbeCount >= 2 &&
		body.Motion.Velocity.Length().Cmp(fixed.FromFraction(1, 5)) <= 0 &&
		body.AngularVelocity.Length().Cmp(fixed.FromFraction(1, 4)) <= 0 &&
		state.VehicleUprightDot.Cmp(fixed.FromFraction(19, 20)) >= 0 {
		state.VehicleSettled = true
	}

	state.RigidBox = *body
}

func applyForceAtRigidBoxPoint(body *physics.RigidBoxBody3D, worldPoint, force geometry.Vector3, dt fixed.Fixed) {
	if body == nil || force.LengthSquared() == fixed.Zero {
		return
	}

	physics.ApplyForce(&body.Motion, force)
	torque := worldPoint.Sub(body.Motion.Position).Cross(force)
	applyTorqueToRigidBox(body, torque, dt)
}

func applyTorqueToRigidBox(body *physics.RigidBoxBody3D, torque geometry.Vector3, dt fixed.Fixed) {
	if body == nil || torque.LengthSquared() == fixed.Zero {
		return
	}

	localTorque := body.Orientation.Conjugate().RotateVector(torque)
	localAngularAccel := geometry.NewVector3(
		localTorque.X.Mul(body.InverseInertiaBody.X),
		localTorque.Y.Mul(body.InverseInertiaBody.Y),
		localTorque.Z.Mul(body.InverseInertiaBody.Z),
	)
	worldAngularAccel := body.Orientation.RotateVector(localAngularAccel)
	body.AngularVelocity = body.AngularVelocity.Add(worldAngularAccel.Scale(dt))
}

func applyUprightAssist(body *physics.RigidBoxBody3D, dt fixed.Fixed, targetUp geometry.Vector3) {
	if body == nil {
		return
	}
	if targetUp.LengthSquared() == fixed.Zero {
		targetUp = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	targetUp = targetUp.Normalize()
	bodyUp := body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	alignAxis := bodyUp.Cross(targetUp)
	if alignAxis.LengthSquared() != fixed.Zero {
		alignTorque := alignAxis.Scale(fixed.FromInt(10))
		applyTorqueToRigidBox(body, alignTorque, dt)
	}
	body.AngularVelocity = geometry.NewVector3(
		body.AngularVelocity.X.Mul(fixed.FromFraction(9, 10)),
		body.AngularVelocity.Y.Mul(fixed.FromFraction(19, 20)),
		body.AngularVelocity.Z.Mul(fixed.FromFraction(9, 10)),
	)
}

func resolveVehicleWallContacts(state *SceneState, body *physics.RigidBoxBody3D) {
	if state == nil || body == nil || len(state.GroundBoxes) == 0 {
		return
	}

	for _, bounds := range state.GroundBoxes {
		sizeX := bounds.Max.X.Sub(bounds.Min.X)
		sizeZ := bounds.Max.Z.Sub(bounds.Min.Z)
		if sizeZ.Cmp(sizeX) <= 0 {
			planeZ := bounds.Min.Z
			planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
			planeNormal := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())
			if fixedAbs(body.Motion.Position.Z.Sub(bounds.Max.Z)).Cmp(fixedAbs(body.Motion.Position.Z.Sub(bounds.Min.Z))) < 0 {
				planeZ = bounds.Max.Z
				planePoint = geometry.NewVector3(fixed.Zero, fixed.Zero, planeZ)
				planeNormal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
			}
			if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
				state.LastContact = result.LastContact
				state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
				state.VehicleEverHitWall = true
			}
			continue
		}

		planeX := bounds.Min.X
		planePoint := geometry.NewVector3(planeX, fixed.Zero, fixed.Zero)
		planeNormal := geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero)
		if fixedAbs(body.Motion.Position.X.Sub(bounds.Max.X)).Cmp(fixedAbs(body.Motion.Position.X.Sub(bounds.Min.X))) < 0 {
			planeX = bounds.Max.X
			planePoint = geometry.NewVector3(planeX, fixed.Zero, fixed.Zero)
			planeNormal = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
		}
		if result := physics.ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, fixed.Zero); result.HadContact {
			state.LastContact = result.LastContact
			state.LastContacts = []physics.SphereTriangleContact{result.LastContact}
			state.VehicleEverHitWall = true
		}
	}
}

func makeVehicleWallImpactBounds() geometry.AxisAlignedBoundingBox {
	return geometry.NewAxisAlignedBoundingBox(
		geometry.NewVector3(fixed.FromInt(-4), fixed.Zero, fixed.FromInt(8)),
		geometry.NewVector3(fixed.FromInt(4), fixed.FromInt(4), fixed.FromFraction(17, 2)),
	)
}

func makeVehicleWallImpactTriangles() []geometry.Triangle {
	wall := makeVehicleWallImpactBounds()
	return makeVehicleWallTrianglesFromBounds(wall)
}

func makeVehicleWallRecoverBounds() geometry.AxisAlignedBoundingBox {
	return geometry.NewAxisAlignedBoundingBox(
		geometry.NewVector3(fixed.FromInt(-4), fixed.Zero, fixed.FromInt(4)),
		geometry.NewVector3(fixed.FromInt(4), fixed.FromInt(4), fixed.FromFraction(9, 2)),
	)
}

func makeVehicleWallRecoverTriangles() []geometry.Triangle {
	return makeVehicleWallTrianglesFromBounds(makeVehicleWallRecoverBounds())
}

func fixedAbs(value fixed.Fixed) fixed.Fixed {
	if value.Cmp(fixed.Zero) < 0 {
		return value.Neg()
	}
	return value
}

func makeVehicleWallTrianglesFromBounds(wall geometry.AxisAlignedBoundingBox) []geometry.Triangle {
	a := geometry.NewVector3(wall.Min.X, wall.Min.Y, wall.Min.Z)
	b := geometry.NewVector3(wall.Max.X, wall.Min.Y, wall.Min.Z)
	c := geometry.NewVector3(wall.Min.X, wall.Max.Y, wall.Min.Z)
	d := geometry.NewVector3(wall.Max.X, wall.Max.Y, wall.Min.Z)
	e := geometry.NewVector3(wall.Min.X, wall.Min.Y, wall.Max.Z)
	f := geometry.NewVector3(wall.Max.X, wall.Min.Y, wall.Max.Z)
	g := geometry.NewVector3(wall.Min.X, wall.Max.Y, wall.Max.Z)
	h := geometry.NewVector3(wall.Max.X, wall.Max.Y, wall.Max.Z)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(b, c, d),
		geometry.NewTriangle(e, f, g),
		geometry.NewTriangle(f, h, g),
		geometry.NewTriangle(a, e, c),
		geometry.NewTriangle(e, g, c),
		geometry.NewTriangle(b, d, f),
		geometry.NewTriangle(f, d, h),
		geometry.NewTriangle(c, g, d),
		geometry.NewTriangle(g, h, d),
		geometry.NewTriangle(a, b, e),
		geometry.NewTriangle(e, b, f),
	}
}

func vehicleUphillPlaneNormal() geometry.Vector3 {
	risePerRun := fixed.FromFraction(1, 4)
	return geometry.NewVector3(fixed.Zero, fixed.One, risePerRun.Neg()).Normalize()
}

func makeVehicleUphillSlopeTriangles() []geometry.Triangle {
	minX := fixed.FromInt(-8)
	maxX := fixed.FromInt(8)
	minZ := fixed.FromInt(-20)
	maxZ := fixed.FromInt(20)
	risePerRun := fixed.FromFraction(1, 4)

	a := geometry.NewVector3(minX, minZ.Mul(risePerRun), minZ)
	b := geometry.NewVector3(maxX, minZ.Mul(risePerRun), minZ)
	c := geometry.NewVector3(minX, maxZ.Mul(risePerRun), maxZ)
	d := geometry.NewVector3(maxX, maxZ.Mul(risePerRun), maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}

func vehicleSlopeCrestPlane(positionZ fixed.Fixed) (geometry.Vector3, geometry.Vector3) {
	crestZ := fixed.FromInt(8)
	crestY := crestZ.Mul(fixed.FromFraction(1, 4))
	if positionZ.Cmp(crestZ) >= 0 {
		return geometry.NewVector3(fixed.Zero, crestY, fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	return geometry.ZeroVector3(), vehicleUphillPlaneNormal()
}

func makeVehicleSlopeCrestTriangles() []geometry.Triangle {
	minX := fixed.FromInt(-8)
	maxX := fixed.FromInt(8)
	minZ := fixed.FromInt(-20)
	crestZ := fixed.FromInt(8)
	maxZ := fixed.FromInt(20)
	risePerRun := fixed.FromFraction(1, 4)
	crestY := crestZ.Mul(risePerRun)

	a := geometry.NewVector3(minX, minZ.Mul(risePerRun), minZ)
	b := geometry.NewVector3(maxX, minZ.Mul(risePerRun), minZ)
	c := geometry.NewVector3(minX, crestY, crestZ)
	d := geometry.NewVector3(maxX, crestY, crestZ)
	e := geometry.NewVector3(minX, crestY, maxZ)
	f := geometry.NewVector3(maxX, crestY, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
		geometry.NewTriangle(c, e, d),
		geometry.NewTriangle(e, f, d),
	}
}

func vehicleCurbBumpPlane(positionZ fixed.Fixed) (geometry.Vector3, geometry.Vector3) {
	rampStartZ := fixed.FromInt(-1)
	topStartZ := fixed.One
	topEndZ := fixed.FromInt(3)
	rampEndZ := fixed.FromInt(5)
	bumpHeight := fixed.FromFraction(3, 5)
	rampSlope := bumpHeight.Div(topStartZ.Sub(rampStartZ))

	switch {
	case positionZ.Cmp(rampStartZ) < 0:
		return geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	case positionZ.Cmp(topStartZ) < 0:
		return geometry.NewVector3(fixed.Zero, rampStartZ.Mul(rampSlope.Neg()), fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, rampSlope.Neg()).Normalize()
	case positionZ.Cmp(topEndZ) < 0:
		return geometry.NewVector3(fixed.Zero, bumpHeight, fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	case positionZ.Cmp(rampEndZ) < 0:
		return geometry.NewVector3(fixed.Zero, rampEndZ.Mul(rampSlope), fixed.Zero), geometry.NewVector3(fixed.Zero, fixed.One, rampSlope).Normalize()
	default:
		return geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
}

func makeVehicleCurbBumpTriangles() []geometry.Triangle {
	minX := fixed.FromInt(-8)
	maxX := fixed.FromInt(8)
	minZ := fixed.FromInt(-20)
	rampStartZ := fixed.FromInt(-1)
	topStartZ := fixed.One
	topEndZ := fixed.FromInt(3)
	rampEndZ := fixed.FromInt(5)
	maxZ := fixed.FromInt(20)
	bumpHeight := fixed.FromFraction(3, 5)

	a := geometry.NewVector3(minX, fixed.Zero, minZ)
	b := geometry.NewVector3(maxX, fixed.Zero, minZ)
	c := geometry.NewVector3(minX, fixed.Zero, rampStartZ)
	d := geometry.NewVector3(maxX, fixed.Zero, rampStartZ)
	e := geometry.NewVector3(minX, bumpHeight, topStartZ)
	f := geometry.NewVector3(maxX, bumpHeight, topStartZ)
	g := geometry.NewVector3(minX, bumpHeight, topEndZ)
	h := geometry.NewVector3(maxX, bumpHeight, topEndZ)
	i := geometry.NewVector3(minX, fixed.Zero, rampEndZ)
	j := geometry.NewVector3(maxX, fixed.Zero, rampEndZ)
	k := geometry.NewVector3(minX, fixed.Zero, maxZ)
	l := geometry.NewVector3(maxX, fixed.Zero, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
		geometry.NewTriangle(c, e, d),
		geometry.NewTriangle(e, f, d),
		geometry.NewTriangle(e, g, f),
		geometry.NewTriangle(g, h, f),
		geometry.NewTriangle(g, i, h),
		geometry.NewTriangle(i, j, h),
		geometry.NewTriangle(i, k, j),
		geometry.NewTriangle(k, l, j),
	}
}

func bodyPositionZ(state *SceneState) fixed.Fixed {
	if state == nil {
		return fixed.Zero
	}
	return state.VehicleChassis.Motion.Position.Z
}
