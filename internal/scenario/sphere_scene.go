package scenario

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func DefaultScenarioDefinitions() []ScenarioDefinition {
	return []ScenarioDefinition{
		NewSphereDropOnFlatGroundScenario(),
		NewSphereBounceOnFlatGroundScenario(),
		NewThreeSphereDropScenario(),
		NewSphereDropOnThirtyDegreeSlopeScenario(),
		NewSphereBounceOnThirtyDegreeSlopeScenario(),
		NewThreeSphereDifferentFloorBounceScenario(),
		NewThreeBoxDifferentFloorBounceScenario(),
		NewThreeBoxSameSlopeBounceScenario(),
		NewThreeBoxSameSlopeAngleComparisonScenario(),
		NewNineBoxFlatAngleComparisonScenario(),
		NewRigidBox3DFlatBounceScenario(),
		NewNineRigidBox3DFlatAngleComparisonScenario(),
		NewNineRigidSphere3DSmallSlopeScenario(),
		NewTwoSphereCollisionScenario(),
		NewTwoSphereDifferentMassCollisionScenario(),
		NewSphereBoxCollisionScenario(),
		NewTwoSphereBoxOpposingMassScenario(),
		NewBoxFrictionFlatSlideScenario(),
		NewSphereBoxFrictionCollisionScenario(),
		NewSphereBoxFrictionBounceCollisionScenario(),
		NewHundredRigidSpheresInBoxScenario(),
		NewHundredBoxesInBoxScenario(),
		NewHundredBoxesInBoxAngleScenario(),
		NewFiftyRigidSpheresAndFiftyRigidBoxesInBoxScenario(),
		NewHundredRigidSpheresAndHundredRigidBoxesInBoxScenario(),
		NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenario(),
		NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedHighSpeedScenario(),
		NewRigidSphereHighSpeedThinWallProjectileScenario(),
		NewRigidSphereHighSpeedThinWallProjectileCCDScenario(),
		NewRigidSphereHighSpeedThinWallProjectileMeshCCDScenario(),
	}
}

func NewSphereDropOnFlatGroundScenario() ScenarioDefinition {
	const scenarioTicks = 180

	return ScenarioDefinition{
		Name:        "Sphere Drop On Flat Ground",
		Description: "A sphere falls under gravity onto a floor built from two triangles and should settle at rest.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return SceneState{
				Sphere: physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
				),
				GroundTriangles: makeFlatGroundTriangles(),
			}
		},
		Step: StepSphereScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere never touched the ground.",
				}
			}

			if !state.Sphere.Grounded {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere was not grounded at the end of the scenario.",
				}
			}

			heightError := state.Sphere.Motion.Position.Y.Sub(state.Sphere.Radius).Abs()
			if heightError.Cmp(fixed.FromRaw(1024)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere did not settle at the expected rest height.",
				}
			}

			verticalSpeed := state.Sphere.Motion.Velocity.Y.Abs()
			if verticalSpeed.Cmp(fixed.FromRaw(1024)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere still had too much vertical speed at the end.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Sphere fell, touched the floor, and settled at rest.",
			}
		},
	}
}

func NewSphereBounceOnFlatGroundScenario() ScenarioDefinition {
	const scenarioTicks = 1200

	return ScenarioDefinition{
		Name:        "Sphere Bounce On Flat Ground",
		Description: "A sphere falls under gravity onto a floor built from two triangles and should bounce back upward.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewDynamicSphereBody(
				fixed.One,
				fixed.One,
				geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(4, 5)

			return SceneState{
				Sphere:          sphere,
				GroundTriangles: makeFlatGroundTriangles(),
			}
		},
		Step: StepSphereScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere never touched the ground.",
				}
			}

			if !state.BounceDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere touched the ground but never bounced upward.",
				}
			}

			if state.PeakBounceHeight.Cmp(fixed.FromInt(4)) < 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere bounced, but the rebound height was lower than expected.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Sphere touched the floor and rebounded upward.",
			}
		},
	}
}

func NewThreeSphereDropScenario() ScenarioDefinition {
	const scenarioTicks = 1200

	return ScenarioDefinition{
		Name:        "Three Sphere Bounce Drop",
		Description: "Three spheres fall together from different heights onto the same flat floor and should all bounce before settling.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := []physics.SphereBody{
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(-4), fixed.FromInt(6), fixed.Zero),
				),
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
				),
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(4), fixed.FromInt(14), fixed.Zero),
				),
			}
			for index := range spheres {
				spheres[index].Restitution = fixed.FromFraction(1, 4)
			}

			return SceneState{
				Spheres:           spheres,
				BounceDetectedSet: make([]bool, len(spheres)),
				GroundTriangles:   makeFlatGroundTriangles(),
			}
		},
		Step: StepMultiSphereScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Spheres) != 3 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly three spheres in the scene.",
				}
			}

			for index, sphere := range state.Spheres {
				if index >= len(state.BounceDetectedSet) || !state.BounceDetectedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "Not all spheres bounced upward after touching the floor.",
					}
				}

				if !sphere.Grounded {
					return ScenarioResult{
						Status:  Failed,
						Message: "Not all spheres were grounded at the end of the scenario.",
					}
				}

				heightError := sphere.Motion.Position.Y.Sub(sphere.Radius).Abs()
				if heightError.Cmp(fixed.FromRaw(1024)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one sphere did not settle at the expected rest height.",
					}
				}

				if state.LastContacts != nil && index >= len(state.LastContacts) {
					return ScenarioResult{
						Status:  Failed,
						Message: "Multi-sphere contact bookkeeping became inconsistent.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "All three spheres bounced and then settled on the floor.",
			}
		},
	}
}

func NewSphereDropOnThirtyDegreeSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 180

	return ScenarioDefinition{
		Name:        "Sphere Drop On 30 Degree Slope",
		Description: "A sphere falls onto a 30 degree side slope with no bounce and should stay in contact while sliding sideways downhill.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return SceneState{
				Sphere: physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero),
				),
				GroundTriangles: makeThirtyDegreeSlopeTriangles(),
			}
		},
		Step: StepSphereScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere never touched the slope.",
				}
			}

			if state.BounceDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere should not bounce in the slope scene.",
				}
			}

			if !state.Sphere.Grounded {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere was not grounded on the slope at the end of the scenario.",
				}
			}

			if state.Sphere.Motion.Position.X.Cmp(fixed.FromInt(-2)) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere did not slide sideways downhill along the slope.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Sphere touched the slope and slid sideways without bouncing.",
			}
		},
	}
}

func NewSphereBounceOnThirtyDegreeSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Sphere Bounce On 30 Degree Slope",
		Description: "A sphere falls onto a 30 degree side slope, bounces on contact, and keeps sliding sideways downhill.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewDynamicSphereBody(
				fixed.One,
				fixed.One,
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(1, 3)

			return SceneState{
				Sphere:          sphere,
				GroundTriangles: makeThirtyDegreeSlopeTriangles(),
			}
		},
		Step: StepSphereScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere never touched the slope.",
				}
			}

			if !state.BounceDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere touched the slope but never bounced.",
				}
			}

			if state.PeakBounceHeight.Cmp(state.Sphere.Radius) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere bounced on the slope, but the rebound height was too low.",
				}
			}

			if state.Sphere.Motion.Position.X.Cmp(fixed.FromInt(-2)) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere did not slide sideways downhill along the slope.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Sphere bounced on the slope and slid sideways downhill.",
			}
		},
	}
}

func NewThreeSphereDifferentFloorBounceScenario() ScenarioDefinition {
	const scenarioTicks = 600

	return ScenarioDefinition{
		Name:        "Three Sphere Floor Bounce Comparison",
		Description: "Three equal spheres drop onto three flat floor strips with low, medium, and high floor bounce.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := []physics.SphereBody{
				physics.NewDynamicSphereBody(fixed.One, fixed.One, geometry.NewVector3(fixed.FromInt(-14), fixed.FromInt(12), fixed.Zero)),
				physics.NewDynamicSphereBody(fixed.One, fixed.One, geometry.NewVector3(fixed.Zero, fixed.FromInt(12), fixed.Zero)),
				physics.NewDynamicSphereBody(fixed.One, fixed.One, geometry.NewVector3(fixed.FromInt(14), fixed.FromInt(12), fixed.Zero)),
			}
			for index := range spheres {
				spheres[index].Restitution = fixed.FromFraction(1, 5)
			}

			return SceneState{
				Spheres:           spheres,
				BounceDetectedSet: make([]bool, len(spheres)),
				PeakBounceHeights: make([]fixed.Fixed, len(spheres)),
				GroundTriangles:   makeThreeFloorBounceTriangles(),
			}
		},
		Step: StepThreeSphereDifferentFloorBounceScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Spheres) != 3 || len(state.PeakBounceHeights) != 3 || len(state.BounceDetectedSet) != 3 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Three-floor bounce scene bookkeeping was incomplete.",
				}
			}

			for index, sphere := range state.Spheres {
				if !state.BounceDetectedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one sphere never bounced on its floor strip.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one sphere fell through the floor strip.",
					}
				}
			}

			if state.PeakBounceHeights[0].Cmp(state.PeakBounceHeights[1]) >= 0 ||
				state.PeakBounceHeights[1].Cmp(state.PeakBounceHeights[2]) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Bounce heights did not increase from low to high floor restitution.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Equal spheres bounced to different heights based on floor restitution.",
			}
		},
	}
}

func NewThreeBoxDifferentFloorBounceScenario() ScenarioDefinition {
	const scenarioTicks = 600

	return ScenarioDefinition{
		Name:        "Three Box Floor Bounce Comparison",
		Description: "Three equal boxes drop onto three flat floor strips with low, medium, and high floor bounce.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			halfExtents := geometry.NewVector3(fixed.One, fixed.One, fixed.One)
			boxes := []physics.BoxBody{
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.FromInt(-14), fixed.FromInt(12), fixed.Zero)),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(12), fixed.Zero)),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.FromInt(14), fixed.FromInt(12), fixed.Zero)),
			}
			for index := range boxes {
				boxes[index].Restitution = fixed.FromFraction(1, 5)
			}

			return SceneState{
				Boxes:                boxes,
				GroundBoxes:          makeThreeFloorBounceBoxes(),
				BoxBounceDetectedSet: make([]bool, len(boxes)),
				BoxPeakBounceHeights: make([]fixed.Fixed, len(boxes)),
			}
		},
		Step: StepThreeBoxDifferentFloorBounceScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Boxes) != 3 || len(state.BoxPeakBounceHeights) != 3 || len(state.BoxBounceDetectedSet) != 3 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Three-box bounce scene bookkeeping was incomplete.",
				}
			}

			for index, box := range state.Boxes {
				if !state.BoxBounceDetectedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box never bounced on its floor strip.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box fell through the floor strip.",
					}
				}
			}

			if state.BoxPeakBounceHeights[0].Cmp(state.BoxPeakBounceHeights[1]) >= 0 ||
				state.BoxPeakBounceHeights[1].Cmp(state.BoxPeakBounceHeights[2]) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Box bounce heights did not increase from low to high floor restitution.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Equal boxes bounced to different heights based on floor restitution.",
			}
		},
	}
}

func NewThreeBoxSameSlopeBounceScenario() ScenarioDefinition {
	const scenarioTicks = 420

	return ScenarioDefinition{
		Name:        "Three Box Slope Bounce Comparison",
		Description: "Three equal boxes drop from the same height onto one side slope while each box uses a different bounce value.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			halfExtents := geometry.NewVector3(fixed.One, fixed.One, fixed.One)
			boxes := []physics.BoxBody{
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(-20))),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero)),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(20))),
			}
			boxes[0].Restitution = fixed.Zero
			boxes[1].Restitution = fixed.FromFraction(1, 2)
			boxes[2].Restitution = fixed.FromFraction(4, 5)

			return SceneState{
				Boxes:                boxes,
				GroundTriangles:      makeThirtyDegreeSlopeTriangles(),
				BoxBounceDetectedSet: make([]bool, len(boxes)),
				BoxPeakBounceHeights: make([]fixed.Fixed, len(boxes)),
			}
		},
		Step: StepThreeBoxSameSlopeBounceScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Boxes) != 3 || len(state.BoxPeakBounceHeights) != 3 || len(state.BoxBounceDetectedSet) != 3 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Three-box slope bookkeeping was incomplete.",
				}
			}

			if state.BoxBounceDetectedSet[0] {
				return ScenarioResult{
					Status:  Failed,
					Message: "The low-bounce box should not rebound on the slope.",
				}
			}

			if !state.BoxBounceDetectedSet[1] || !state.BoxBounceDetectedSet[2] {
				return ScenarioResult{
					Status:  Failed,
					Message: "The medium and high bounce boxes should rebound on the slope.",
				}
			}

			for _, box := range state.Boxes {
				if box.Motion.Position.X.Cmp(fixed.FromInt(-2)) >= 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box did not slide downhill along the slope.",
					}
				}
			}

			if state.BoxPeakBounceHeights[1].Cmp(state.BoxPeakBounceHeights[2]) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The higher-bounce box did not rebound higher than the medium-bounce box.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Three boxes used different bounce values on the same slope.",
			}
		},
	}
}

func NewThreeBoxSameSlopeAngleComparisonScenario() ScenarioDefinition {
	const scenarioTicks = 420

	return ScenarioDefinition{
		Name:        "Three Box Slope Angle Comparison",
		Description: "Three equal boxes drop from the same height onto one slope with equal bounce but different starting angles.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			halfExtents := geometry.NewVector3(fixed.One, fixed.One, fixed.One)
			initialRotations := []fixed.Fixed{
				fixed.FromFraction(-174533, 1000000),
				fixed.Zero,
				fixed.FromFraction(261799, 1000000),
			}
			boxes := []physics.BoxBody{
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(-20))),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero)),
				physics.NewDynamicBoxBody(fixed.One, halfExtents, geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(20))),
			}
			for index := range boxes {
				boxes[index].Restitution = fixed.FromFraction(1, 2)
				boxes[index].RotationZ = initialRotations[index]
			}

			return SceneState{
				Boxes:                 boxes,
				GroundTriangles:       makeThirtyDegreeSlopeTriangles(),
				BoxBounceDetectedSet:  make([]bool, len(boxes)),
				BoxPeakBounceHeights:  make([]fixed.Fixed, len(boxes)),
				BoxInitialRotations:   initialRotations,
				BoxRotationChangedSet: make([]bool, len(boxes)),
			}
		},
		Step: StepThreeBoxSameSlopeAngleComparisonScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Boxes) != 3 || len(state.BoxInitialRotations) != 3 || len(state.BoxRotationChangedSet) != 3 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Three-box angle scene bookkeeping was incomplete.",
				}
			}

			for index, box := range state.Boxes {
				if !state.BoxRotationChangedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box never changed rotation after touching the slope.",
					}
				}
				if box.Motion.Position.X.Cmp(fixed.FromInt(-2)) >= 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box did not slide downhill along the slope.",
					}
				}
			}

			reboundCount := 0
			for _, bounced := range state.BoxBounceDetectedSet {
				if bounced {
					reboundCount++
				}
			}
			if reboundCount == 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "No box produced an upward rebound on the slope.",
				}
			}

			if state.BoxPeakBounceHeights[0] == state.BoxPeakBounceHeights[1] &&
				state.BoxPeakBounceHeights[1] == state.BoxPeakBounceHeights[2] {
				return ScenarioResult{
					Status:  Failed,
					Message: "Different starting angles did not produce different rebound heights.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Three equal boxes reacted differently because they started with different angles.",
			}
		},
	}
}

func NewNineBoxFlatAngleComparisonScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Nine Box Flat Angle Comparison",
		Description: "Nine equal boxes drop onto flat ground from the same height while each box starts with a different angle.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			halfExtents := geometry.NewVector3(fixed.One, fixed.One, fixed.One)
			initialRotations := []fixed.Fixed{
				fixed.FromFraction(-349066, 1000000),
				fixed.FromFraction(-261799, 1000000),
				fixed.FromFraction(-174533, 1000000),
				fixed.FromFraction(-87266, 1000000),
				fixed.Zero,
				fixed.FromFraction(87266, 1000000),
				fixed.FromFraction(174533, 1000000),
				fixed.FromFraction(261799, 1000000),
				fixed.FromFraction(349066, 1000000),
			}
			positions := []geometry.Vector3{
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.FromInt(18)),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(18)),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.FromInt(18)),
			}

			boxes := make([]physics.BoxBody, 0, len(initialRotations))
			for index, rotation := range initialRotations {
				box := physics.NewDynamicBoxBody(fixed.One, halfExtents, positions[index])
				box.Restitution = fixed.Zero
				box.RotationZ = rotation
				boxes = append(boxes, box)
			}

			return SceneState{
				Boxes:                 boxes,
				GroundTriangles:       makeFlatGroundTriangles(),
				BoxBounceDetectedSet:  make([]bool, len(boxes)),
				BoxPeakBounceHeights:  make([]fixed.Fixed, len(boxes)),
				BoxInitialRotations:   initialRotations,
				BoxRotationChangedSet: make([]bool, len(boxes)),
			}
		},
		Step: StepNineBoxFlatAngleComparisonScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Boxes) != 9 || len(state.BoxInitialRotations) != 9 || len(state.BoxRotationChangedSet) != 9 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Nine-box flat angle scene bookkeeping was incomplete.",
				}
			}

			rotationChangedCount := 0
			for index, box := range state.Boxes {
				if !box.Grounded {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box was not grounded on the flat floor at the end of the scene.",
					}
				}
				if !state.BoxRotationChangedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box never changed rotation after hitting the flat floor.",
					}
				}
				rotationChangedCount++
			}

			if rotationChangedCount != len(state.Boxes) {
				return ScenarioResult{
					Status:  Failed,
					Message: "Not all boxes changed rotation on the flat floor.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Nine boxes with different starting angles hit the same flat floor and rotated under physics.",
			}
		},
	}
}

func NewRigidBox3DFlatBounceScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Rigid Box 3D Flat Bounce",
		Description: "One rigid box falls onto flat ground with full 3-axis orientation and angular velocity.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			box := physics.NewRigidBoxBody3D(
				fixed.One,
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero),
			)
			box.Restitution = fixed.FromFraction(1, 3)
			box.Orientation = physics.NewQuaternionFromEulerXYZ(
				fixed.FromFraction(174533, 1000000),
				fixed.FromFraction(261799, 1000000),
				fixed.FromFraction(87266, 1000000),
			)
			box.AngularVelocity = geometry.NewVector3(
				fixed.FromFraction(3, 2),
				fixed.FromFraction(5, 4),
				fixed.FromFraction(7, 4),
			)

			return SceneState{
				RigidBox: box,
			}
		},
		Step: StepRigidBox3DFlatBounceScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Rigid box never touched the flat ground.",
				}
			}
			if !state.RigidBoxBounceDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "Rigid box never produced an upward bounce.",
				}
			}
			if !state.RigidBoxRotationChanged {
				return ScenarioResult{
					Status:  Failed,
					Message: "Rigid box orientation never changed under 3D angular motion.",
				}
			}
			if state.RigidBoxPeakBounceHeight.Cmp(state.RigidBox.HalfExtents.Y) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Rigid box bounce height was too low.",
				}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Rigid box bounced and rotated on all axes against flat ground.",
			}
		},
	}
}

func NewNineRigidBox3DFlatAngleComparisonScenario() ScenarioDefinition {
	const scenarioTicks = 480

	return ScenarioDefinition{
		Name:        "Nine Rigid Box 3D Flat Angle Comparison",
		Description: "Nine rigid boxes fall onto flat ground from the same height while each box starts with a different 3-axis orientation.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			halfExtents := geometry.NewVector3(fixed.One, fixed.One, fixed.One)
			positions := []geometry.Vector3{
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.FromInt(-18)),
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.Zero),
				geometry.NewVector3(fixed.FromInt(-18), fixed.FromInt(14), fixed.FromInt(18)),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(14), fixed.FromInt(18)),
				geometry.NewVector3(fixed.FromInt(18), fixed.FromInt(14), fixed.FromInt(18)),
			}
			orientations := []physics.Quaternion{
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(-349066, 1000000), fixed.FromFraction(-174533, 1000000), fixed.FromFraction(-87266, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(-261799, 1000000), fixed.FromFraction(-87266, 1000000), fixed.Zero),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(-174533, 1000000), fixed.Zero, fixed.FromFraction(87266, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(-87266, 1000000), fixed.FromFraction(174533, 1000000), fixed.FromFraction(174533, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.Zero, fixed.Zero, fixed.Zero),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(87266, 1000000), fixed.FromFraction(-174533, 1000000), fixed.FromFraction(261799, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(174533, 1000000), fixed.FromFraction(87266, 1000000), fixed.FromFraction(-261799, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(261799, 1000000), fixed.FromFraction(261799, 1000000), fixed.FromFraction(87266, 1000000)),
				physics.NewQuaternionFromEulerXYZ(fixed.FromFraction(349066, 1000000), fixed.FromFraction(-261799, 1000000), fixed.FromFraction(174533, 1000000)),
			}

			boxes := make([]physics.RigidBoxBody3D, 0, len(positions))
			for index, position := range positions {
				box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
				box.Restitution = fixed.Zero
				box.Orientation = orientations[index]
				boxes = append(boxes, box)
			}

			return SceneState{
				RigidBoxes:                 boxes,
				RigidBoxBounceDetectedSet:  make([]bool, len(boxes)),
				RigidBoxPeakBounceHeights:  make([]fixed.Fixed, len(boxes)),
				RigidBoxRotationChangedSet: make([]bool, len(boxes)),
			}
		},
		Step: StepNineRigidBox3DFlatAngleComparisonScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidBoxes) != 9 || len(state.RigidBoxRotationChangedSet) != 9 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Nine rigid-box flat angle scene bookkeeping was incomplete.",
				}
			}

			for index, box := range state.RigidBoxes {
				if !box.Grounded {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box was not grounded on the flat floor at the end of the scene.",
					}
				}
				if !state.RigidBoxRotationChangedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box never changed orientation after hitting the flat floor.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Nine rigid boxes with different starting orientations hit the same flat floor and rotated under 3D physics.",
			}
		},
	}
}

func NewNineRigidSphere3DSmallSlopeScenario() ScenarioDefinition {
	const scenarioTicks = 300

	slopePatches := makeNineSmallSlopePatchSpecs()
	startPositions := make([]geometry.Vector3, 0, len(slopePatches))
	for _, patch := range slopePatches {
		startPositions = append(startPositions, geometry.NewVector3(patch.Center.X, fixed.FromInt(8), patch.Center.Z))
	}

	return ScenarioDefinition{
		Name:        "Nine Rigid Sphere 3D Small Slope",
		Description: "Nine rigid spheres with full 3-axis spin fall onto nine small side slopes and roll downhill under contact friction.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, len(startPositions))
			for index, position := range startPositions {
				sphere := physics.NewRigidSphereBody3D(fixed.One, fixed.One, position)
				sphere.Restitution = fixed.Zero
				sphere.Friction = fixed.FromFraction(3, 5)
				sphere.Orientation = physics.NewQuaternionFromEulerXYZ(
					fixed.FromFraction(int64((index%3)-1)*87266, 1000000),
					fixed.FromFraction(int64((index/3)-1)*174533, 1000000),
					fixed.FromFraction(int64(index-4)*43633, 1000000),
				)
				sphere.AngularVelocity = geometry.NewVector3(
					fixed.FromFraction(int64(index+1), 2),
					fixed.FromFraction(int64(index+2), 3),
					fixed.FromFraction(int64(index+3), 4),
				)
				spheres = append(spheres, sphere)
			}

			return SceneState{
				RigidSpheres:                  spheres,
				RigidSphereTouchedGroundSet:   make([]bool, len(spheres)),
				RigidSphereBounceDetectedSet:  make([]bool, len(spheres)),
				RigidSpherePeakBounceHeights:  make([]fixed.Fixed, len(spheres)),
				RigidSphereRotationChangedSet: make([]bool, len(spheres)),
				GroundTriangles:               makeSlopePatchTriangles(slopePatches),
			}
		},
		Step: StepNineRigidSphere3DSmallSlopeScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 9 || len(state.RigidSphereRotationChangedSet) != 9 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Nine rigid-sphere slope scene bookkeeping was incomplete.",
				}
			}

			for index, sphere := range state.RigidSpheres {
				if index >= len(state.RigidSphereTouchedGroundSet) || !state.RigidSphereTouchedGroundSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere never produced a slope contact.",
					}
				}
				if !state.RigidSphereRotationChangedSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere never changed orientation on its slope.",
					}
				}
				displacement := sphere.Motion.Position.Sub(startPositions[index])
				if displacement.Dot(slopePatches[index].DownhillDirection).Cmp(fixed.Zero) <= 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere did not roll downhill along its small slope.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Nine rigid spheres rolled downhill on small slopes while rotating under full 3-axis state.",
			}
		},
	}
}

func NewTwoSphereCollisionScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Two Sphere Collision",
		Description: "Two equal spheres move toward each other in midair and should collide, separate, and exchange horizontal motion.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := []physics.SphereBody{
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(-6), fixed.FromInt(6), fixed.Zero),
				),
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(6), fixed.FromInt(6), fixed.Zero),
				),
			}
			spheres[0].Motion.Velocity = geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero)
			spheres[1].Motion.Velocity = geometry.NewVector3(fixed.FromInt(-4), fixed.Zero, fixed.Zero)

			return SceneState{
				Spheres: spheres,
			}
		},
		Step: StepTwoSphereCollisionScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Spheres) != 2 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly two spheres in the scene.",
				}
			}
			if !state.SphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The two spheres never collided.",
				}
			}
			if state.Spheres[0].Motion.Velocity.X.Cmp(fixed.Zero) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere A did not reverse horizontal direction after collision.",
				}
			}
			if state.Spheres[1].Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Sphere B did not reverse horizontal direction after collision.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Two spheres collided and exchanged horizontal motion.",
			}
		},
	}
}

func NewTwoSphereDifferentMassCollisionScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Two Sphere Collision Different Mass",
		Description: "Two same-size spheres collide in midair with different masses, so the lighter sphere should leave faster after impact.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := []physics.SphereBody{
				physics.NewDynamicSphereBody(
					fixed.FromInt(3),
					fixed.One,
					geometry.NewVector3(fixed.FromInt(-6), fixed.FromInt(6), fixed.Zero),
				),
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(2), fixed.FromInt(6), fixed.Zero),
				),
			}
			spheres[0].Motion.Velocity = geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero)

			return SceneState{
				Spheres: spheres,
			}
		},
		Step: StepTwoSphereCollisionScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Spheres) != 2 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly two spheres in the scene.",
				}
			}
			if !state.SphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The two different-mass spheres never collided.",
				}
			}
			if state.Spheres[0].Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The heavier sphere should still move forward after collision.",
				}
			}
			if state.Spheres[1].Motion.Velocity.X.Cmp(state.Spheres[0].Motion.Velocity.X) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The lighter sphere should leave the collision faster than the heavier sphere.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Different masses changed the post-collision speeds while the spheres stayed the same size.",
			}
		},
	}
}

func NewSphereBoxCollisionScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Sphere Box Collision",
		Description: "A sphere collides with a same-size box in midair, transferring momentum between different object types.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewDynamicSphereBody(
				fixed.One,
				fixed.One,
				geometry.NewVector3(fixed.FromInt(-6), fixed.FromInt(6), fixed.Zero),
			)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero)

			box := physics.NewDynamicBoxBody(
				fixed.FromInt(2),
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.FromInt(2), fixed.FromInt(6), fixed.Zero),
			)

			return SceneState{
				Sphere: sphere,
				Box:    box,
			}
		},
		Step: StepSphereBoxCollisionScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The sphere and box never collided.",
				}
			}
			if state.Box.Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not gain rightward velocity after the collision.",
				}
			}
			if state.Sphere.Motion.Velocity.X.Cmp(fixed.Zero) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The sphere did not lose enough rightward velocity after hitting the box.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "A sphere hit the box and transferred momentum across object types.",
			}
		},
	}
}

func NewTwoSphereBoxOpposingMassScenario() ScenarioDefinition {
	const scenarioTicks = 180

	return ScenarioDefinition{
		Name:        "Two Sphere Box Opposing Mass",
		Description: "Two same-size spheres hit one box from opposite sides with different masses, so the box should drift toward the heavier sphere's push.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := []physics.SphereBody{
				physics.NewDynamicSphereBody(
					fixed.FromInt(3),
					fixed.One,
					geometry.NewVector3(fixed.FromInt(-6), fixed.FromInt(6), fixed.Zero),
				),
				physics.NewDynamicSphereBody(
					fixed.One,
					fixed.One,
					geometry.NewVector3(fixed.FromInt(6), fixed.FromInt(6), fixed.Zero),
				),
			}
			spheres[0].Motion.Velocity = geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero)
			spheres[1].Motion.Velocity = geometry.NewVector3(fixed.FromInt(-4), fixed.Zero, fixed.Zero)

			box := physics.NewDynamicBoxBody(
				fixed.FromInt(2),
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(6), fixed.Zero),
			)

			return SceneState{
				Spheres: spheres,
				Box:     box,
			}
		},
		Step: StepTwoSphereBoxOpposingMassScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Spheres) != 2 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly two spheres in the scene.",
				}
			}
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The spheres never reached the box.",
				}
			}
			if state.Box.Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not drift toward the heavier sphere's push direction.",
				}
			}
			if state.Box.Motion.Position.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not end up displaced toward the heavier sphere.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Two different-mass spheres pushed the box from opposite sides and the heavier side won.",
			}
		},
	}
}

func NewBoxFrictionFlatSlideScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Box Friction Flat Slide",
		Description: "A box lands on a flat floor with horizontal speed and should slow down under contact friction until nearly at rest.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			box := physics.NewDynamicBoxBody(
				fixed.One,
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.Zero, fixed.FromInt(4), fixed.Zero),
			)
			box.Friction = fixed.FromFraction(1, 8)
			box.Motion.Velocity = geometry.NewVector3(fixed.FromInt(6), fixed.Zero, fixed.Zero)

			return SceneState{
				Box: box,
				GroundBoxes: []geometry.AxisAlignedBoundingBox{
					makeFlatStripBox(fixed.FromInt(-40), fixed.FromInt(40)),
				},
			}
		},
		Step: StepBoxFrictionFlatSlideScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.Box.Grounded {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box was not grounded on the floor at the end of the friction scene.",
				}
			}
			if state.Box.Motion.Position.X.Cmp(fixed.FromInt(2)) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not slide forward before friction slowed it down.",
				}
			}
			if state.Box.Motion.Velocity.X.Abs().Cmp(fixed.FromFraction(1, 10)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box still had too much horizontal speed after friction.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "The box slid forward and slowed to near rest because of floor friction.",
			}
		},
	}
}

func NewSphereBoxFrictionCollisionScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Sphere Box Friction Collision",
		Description: "A sphere hits a grounded box on a flat floor, and the box should slide forward before floor friction slows it to near rest.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewDynamicSphereBody(
				fixed.FromInt(2),
				fixed.One,
				geometry.NewVector3(fixed.FromInt(-8), fixed.One, fixed.Zero),
			)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.Zero)

			box := physics.NewDynamicBoxBody(
				fixed.FromInt(2),
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
			)
			box.Friction = fixed.FromFraction(1, 8)

			return SceneState{
				Sphere: sphere,
				Box:    box,
				GroundBoxes: []geometry.AxisAlignedBoundingBox{
					makeFlatStripBox(fixed.FromInt(-40), fixed.FromInt(40)),
				},
			}
		},
		Step: StepSphereBoxFrictionCollisionScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The sphere never collided with the friction box.",
				}
			}
			if !state.Box.Grounded {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box was not grounded on the floor at the end of the friction collision scene.",
				}
			}
			if state.Box.Motion.Position.X.Cmp(fixed.FromFraction(1, 2)) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not slide forward after the sphere hit it.",
				}
			}
			if state.Box.Motion.Velocity.X.Abs().Cmp(fixed.FromFraction(1, 10)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box still had too much horizontal speed after friction acted on the collision.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "The sphere pushed the box forward and floor friction slowed the box to near rest.",
			}
		},
	}
}

func NewSphereBoxFrictionBounceCollisionScenario() ScenarioDefinition {
	const scenarioTicks = 240

	return ScenarioDefinition{
		Name:        "Sphere Box Friction Bounce Collision",
		Description: "A bouncing sphere hits a bouncing box on a flat floor, and the box should slide forward while both objects keep some rebound.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewRigidSphereBody3D(
				fixed.FromInt(3),
				fixed.One,
				geometry.NewVector3(fixed.FromInt(-8), fixed.One, fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(3, 5)
			sphere.Friction = fixed.FromFraction(1, 5)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.FromFraction(3, 2))

			box := physics.NewDynamicBoxBody(
				fixed.FromInt(10),
				geometry.NewVector3(fixed.One, fixed.One, fixed.One),
				geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
			)
			box.Restitution = fixed.FromFraction(2, 5)
			box.Friction = fixed.FromFraction(1, 8)

			return SceneState{
				RigidSphere: sphere,
				Box:         box,
				GroundBoxes: []geometry.AxisAlignedBoundingBox{
					makeFlatStripBox(fixed.FromInt(-40), fixed.FromInt(40)),
				},
			}
		},
		Step: StepSphereBoxFrictionBounceCollisionScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The bouncing sphere never collided with the bouncing box.",
				}
			}
			if !state.Box.Grounded {
				return ScenarioResult{
					Status:  Failed,
					Message: "The bouncing box was not grounded at the end of the scene.",
				}
			}
			if state.Box.Motion.Position.X.Cmp(fixed.FromFraction(1, 4)) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not slide forward after the collision.",
				}
			}
			if state.Box.Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The box did not keep any forward rebound after impact.",
				}
			}
			if state.RigidSphere.Motion.Velocity.X.Cmp(fixed.Zero) >= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The sphere did not rebound backward after hitting the box.",
				}
			}
			if state.RigidSphere.AngularVelocity.LengthSquared().Cmp(fixed.Zero) <= 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid sphere never gained visible spin from the collision.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "The rigid sphere and box both bounced on impact, and the rigid sphere picked up spin while floor friction still slowed the box.",
			}
		},
	}
}

func NewHundredRigidSpheresInBoxScenario() ScenarioDefinition {
	const scenarioTicks = 420

	return ScenarioDefinition{
		Name:        "Hundred Rigid Spheres In Box",
		Description: "One hundred rigid spheres fall into an open box, collide with the walls and each other, and settle inside the container.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, 100)
			radius := fixed.FromFraction(2, 5)
			spacing := fixed.FromFraction(19, 20)
			startX := fixed.FromFraction(-171, 50)
			startZ := fixed.FromFraction(-171, 50)
			startY := fixed.FromInt(10)

			for layer := 0; layer < 4; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						if len(spheres) == 100 {
							break
						}
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 3)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 3)),
						)
						sphere := physics.NewRigidSphereBody3D(fixed.One, radius, position)
						sphere.Restitution = fixed.FromFraction(1, 10)
						sphere.Friction = fixed.FromFraction(1, 5)
						spheres = append(spheres, sphere)
					}
				}
			}

			return SceneState{
				RigidSpheres:                  spheres,
				RigidSphereTouchedGroundSet:   make([]bool, len(spheres)),
				RigidSphereRotationChangedSet: make([]bool, len(spheres)),
				GroundTriangles:               makeOpenBoxContainerTriangles(),
			}
		},
		Step: StepHundredRigidSpheresInBoxScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly one hundred rigid spheres in the box scene.",
				}
			}
			if !state.RigidSphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with each other.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for index, sphere := range state.RigidSpheres {
				if sphere.Motion.Position.X.Cmp(minX.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.X.Cmp(maxX.Add(sphere.Radius)) > 0 ||
					sphere.Motion.Position.Z.Cmp(minZ.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.Z.Cmp(maxZ.Add(sphere.Radius)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere escaped the container bounds.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere fell below the box floor.",
					}
				}
				if index < len(state.RigidSphereTouchedGroundSet) && !state.RigidSphereTouchedGroundSet[index] {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere never touched the container.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "One hundred rigid spheres stayed inside the open box and collided under 3D sphere physics.",
			}
		},
	}
}

func NewHundredBoxesInBoxScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Hundred Boxes In Box",
		Description: "One hundred boxes fall into an open box, collide with the floor, walls, and each other, and stay inside the container.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			boxes := make([]physics.BoxBody, 0, 100)
			halfExtents := geometry.NewVector3(fixed.FromFraction(7, 20), fixed.FromFraction(7, 20), fixed.FromFraction(7, 20))
			spacing := fixed.FromFraction(17, 20)
			startX := fixed.FromFraction(-153, 50)
			startZ := fixed.FromFraction(-153, 50)
			startY := fixed.FromInt(10)

			for layer := 0; layer < 4; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						if len(boxes) == 100 {
							break
						}
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 4)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 4)),
						)
						box := physics.NewDynamicBoxBody(fixed.One, halfExtents, position)
						box.Restitution = fixed.FromFraction(1, 10)
						box.Friction = fixed.FromFraction(1, 6)
						boxes = append(boxes, box)
					}
				}
			}

			return SceneState{
				Boxes: boxes,
			}
		},
		Step: StepHundredBoxesInBoxScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.Boxes) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly one hundred boxes in the box scene.",
				}
			}
			if !state.BoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The boxes never collided with each other.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for _, box := range state.Boxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box escaped the container bounds.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one box fell below the box floor.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "One hundred boxes stayed inside the open box and collided under box physics.",
			}
		},
	}
}

func NewHundredBoxesInBoxAngleScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Hundred Rigid Boxes In Box Angle",
		Description: "One hundred rigid boxes fall into an open box while starting from different 3D orientations.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			boxes := make([]physics.RigidBoxBody3D, 0, 100)
			halfExtents := geometry.NewVector3(fixed.FromFraction(7, 20), fixed.FromFraction(7, 20), fixed.FromFraction(7, 20))
			spacing := fixed.FromFraction(17, 20)
			startX := fixed.FromFraction(-153, 50)
			startZ := fixed.FromFraction(-153, 50)
			startY := fixed.FromInt(10)

			for layer := 0; layer < 4; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						if len(boxes) == 100 {
							break
						}
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 4)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 4)),
						)
						box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
						box.Restitution = fixed.FromFraction(1, 10)
						index := len(boxes)
						box.Orientation = physics.NewQuaternionFromEulerXYZ(
							fixed.FromFraction(int64((index%9)-4)*43633, 1000000),
							fixed.FromFraction(int64((index%7)-3)*62333, 1000000),
							fixed.FromFraction(int64((index%11)-5)*87266, 1000000),
						)
						box.AngularVelocity = geometry.NewVector3(
							fixed.FromFraction(int64((index%5)-2), 3),
							fixed.FromFraction(int64((index%7)-3), 4),
							fixed.FromFraction(int64((index%9)-4), 5),
						)
						boxes = append(boxes, box)
					}
				}
			}

			return SceneState{
				RigidBoxes:                 boxes,
				RigidBoxRotationChangedSet: make([]bool, len(boxes)),
			}
		},
		Step: StepHundredRigidBoxesInBoxAngleScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidBoxes) != 100 || len(state.RigidBoxRotationChangedSet) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Hundred rigid-box angle scene bookkeeping was incomplete.",
				}
			}
			if !state.RigidBoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The angled rigid boxes never collided with each other.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			rotationChangedCount := 0
			for index, box := range state.RigidBoxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one angled rigid box escaped the container bounds.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one angled rigid box fell below the box floor.",
					}
				}
				if state.RigidBoxRotationChangedSet[index] {
					rotationChangedCount++
				}
			}
			if rotationChangedCount == 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "No angled rigid box changed orientation while falling in the container.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "One hundred rigid boxes with different starting orientations stayed inside the open box and continued rotating.",
			}
		},
	}
}

func NewFiftyRigidSpheresAndFiftyRigidBoxesInBoxScenario() ScenarioDefinition {
	const scenarioTicks = 300

	return ScenarioDefinition{
		Name:        "Fifty Rigid Spheres And Fifty Rigid Boxes In Box",
		Description: "Fifty rigid spheres and fifty rigid boxes fall together into an open box, collide across both shape types, and stay inside the container.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, 50)
			boxes := make([]physics.RigidBoxBody3D, 0, 50)
			radius := fixed.FromFraction(3, 10)
			halfExtents := geometry.NewVector3(fixed.FromFraction(3, 10), fixed.FromFraction(3, 10), fixed.FromFraction(3, 10))
			spacing := fixed.FromFraction(21, 20)
			startX := fixed.FromFraction(-21, 10)
			startZ := fixed.FromFraction(-21, 10)
			startY := fixed.FromInt(8)
			index := 0

			for layer := 0; layer < 4; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 5)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 5)),
						)
						if index%2 == 0 {
							sphere := physics.NewRigidSphereBody3D(fixed.One, radius, position)
							sphere.Restitution = fixed.FromFraction(1, 5)
							sphere.Friction = fixed.FromFraction(1, 10)
							spheres = append(spheres, sphere)
						} else {
							box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
							box.Restitution = fixed.FromFraction(1, 10)
							box.Orientation = physics.NewQuaternionFromEulerXYZ(
								fixed.FromFraction(int64((index%7)-3), 10),
								fixed.FromFraction(int64((index%9)-4), 12),
								fixed.FromFraction(int64((index%11)-5), 14),
							)
							box.AngularVelocity = geometry.NewVector3(
								fixed.FromFraction(int64((index%5)-2), 4),
								fixed.FromFraction(int64((index%7)-3), 5),
								fixed.FromFraction(int64((index%9)-4), 6),
							)
							boxes = append(boxes, box)
						}
						index++
					}
				}
			}

			return SceneState{
				RigidSpheres: spheres,
				RigidBoxes:   boxes,
				GroundTriangles: makeOpenBoxContainerTriangles(),
			}
		},
		Step: StepFiftyRigidSpheresAndFiftyRigidBoxesInBoxScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 50 || len(state.RigidBoxes) != 50 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly fifty rigid spheres and fifty rigid boxes.",
				}
			}
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with the rigid boxes.",
				}
			}
			if !state.RigidSphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with each other.",
				}
			}
			if !state.RigidBoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid boxes never collided with each other.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for _, sphere := range state.RigidSpheres {
				if sphere.Motion.Position.X.Cmp(minX.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.X.Cmp(maxX.Add(sphere.Radius)) > 0 ||
					sphere.Motion.Position.Z.Cmp(minZ.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.Z.Cmp(maxZ.Add(sphere.Radius)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere escaped the container bounds.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere fell below the box floor.",
					}
				}
			}
			for _, box := range state.RigidBoxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box escaped the container bounds.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box fell below the box floor.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Fifty rigid spheres and fifty rigid boxes stayed inside the open box and collided across both shape types.",
			}
		},
	}
}

func NewHundredRigidSpheresAndHundredRigidBoxesInBoxScenario() ScenarioDefinition {
	const scenarioTicks = 420

	return ScenarioDefinition{
		Name:        "Hundred Rigid Spheres And Hundred Rigid Boxes In Box",
		Description: "One hundred rigid spheres and one hundred rigid boxes fall together into an open box, collide across both shape types, and stay inside the container.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, 100)
			boxes := make([]physics.RigidBoxBody3D, 0, 100)
			radius := fixed.FromFraction(1, 4)
			halfExtents := geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromFraction(1, 4), fixed.FromFraction(1, 4))
			spacing := fixed.FromFraction(9, 10)
			startX := fixed.FromFraction(-18, 5)
			startZ := fixed.FromFraction(-18, 5)
			startY := fixed.FromInt(8)
			index := 0

			for layer := 0; layer < 8; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 6)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 6)),
						)
						if index%2 == 0 {
							sphere := physics.NewRigidSphereBody3D(fixed.One, radius, position)
							sphere.Restitution = fixed.FromFraction(1, 5)
							sphere.Friction = fixed.FromFraction(1, 10)
							spheres = append(spheres, sphere)
						} else {
							box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
							box.Restitution = fixed.FromFraction(1, 10)
							box.Orientation = physics.NewQuaternionFromEulerXYZ(
								fixed.FromFraction(int64((index%7)-3), 10),
								fixed.FromFraction(int64((index%9)-4), 12),
								fixed.FromFraction(int64((index%11)-5), 14),
							)
							box.AngularVelocity = geometry.NewVector3(
								fixed.FromFraction(int64((index%5)-2), 4),
								fixed.FromFraction(int64((index%7)-3), 5),
								fixed.FromFraction(int64((index%9)-4), 6),
							)
							boxes = append(boxes, box)
						}
						index++
					}
				}
			}

			return SceneState{
				RigidSpheres:    spheres,
				RigidBoxes:      boxes,
				GroundTriangles: makeOpenBoxContainerTriangles(),
			}
		},
		Step: StepFiftyRigidSpheresAndFiftyRigidBoxesInBoxScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 100 || len(state.RigidBoxes) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly one hundred rigid spheres and one hundred rigid boxes.",
				}
			}
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with the rigid boxes.",
				}
			}
			if !state.RigidSphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with each other.",
				}
			}
			if !state.RigidBoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid boxes never collided with each other.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for _, sphere := range state.RigidSpheres {
				if sphere.Motion.Position.X.Cmp(minX.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.X.Cmp(maxX.Add(sphere.Radius)) > 0 ||
					sphere.Motion.Position.Z.Cmp(minZ.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.Z.Cmp(maxZ.Add(sphere.Radius)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere escaped the container bounds.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere fell below the box floor.",
					}
				}
			}
			for _, box := range state.RigidBoxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box escaped the container bounds.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box fell below the box floor.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "One hundred rigid spheres and one hundred rigid boxes stayed inside the open box and collided across both shape types.",
			}
		},
	}
}

func NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenario() ScenarioDefinition {
	const scenarioTicks = 420

	return ScenarioDefinition{
		Name:        "Hundred Rigid Spheres And Hundred Rigid Boxes In Box Optimized",
		Description: "One hundred rigid spheres and one hundred rigid boxes fall together into an open box using a uniform-grid broadphase to reduce pair checks.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, 100)
			boxes := make([]physics.RigidBoxBody3D, 0, 100)
			radius := fixed.FromFraction(1, 4)
			halfExtents := geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromFraction(1, 4), fixed.FromFraction(1, 4))
			spacing := fixed.FromFraction(9, 10)
			startX := fixed.FromFraction(-18, 5)
			startZ := fixed.FromFraction(-18, 5)
			startY := fixed.FromInt(8)
			index := 0

			for layer := 0; layer < 8; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 6)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 6)),
						)
						if index%2 == 0 {
							sphere := physics.NewRigidSphereBody3D(fixed.One, radius, position)
							sphere.Restitution = fixed.FromFraction(1, 5)
							sphere.Friction = fixed.FromFraction(1, 10)
							spheres = append(spheres, sphere)
						} else {
							box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
							box.Restitution = fixed.FromFraction(1, 10)
							box.Orientation = physics.NewQuaternionFromEulerXYZ(
								fixed.FromFraction(int64((index%7)-3), 10),
								fixed.FromFraction(int64((index%9)-4), 12),
								fixed.FromFraction(int64((index%11)-5), 14),
							)
							box.AngularVelocity = geometry.NewVector3(
								fixed.FromFraction(int64((index%5)-2), 4),
								fixed.FromFraction(int64((index%7)-3), 5),
								fixed.FromFraction(int64((index%9)-4), 6),
							)
							boxes = append(boxes, box)
						}
						index++
					}
				}
			}

			return SceneState{
				RigidSpheres:    spheres,
				RigidBoxes:      boxes,
				GroundTriangles: makeOpenBoxContainerTriangles(),
			}
		},
		Step: StepHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 100 || len(state.RigidBoxes) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly one hundred rigid spheres and one hundred rigid boxes.",
				}
			}
			if !state.SphereBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with the rigid boxes.",
				}
			}
			if !state.RigidSphereSphereCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid spheres never collided with each other.",
				}
			}
			if !state.RigidBoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "The rigid boxes never collided with each other.",
				}
			}
			if state.SphereSphereCandidateCount >= 4950 || state.BoxBoxCandidateCount >= 4950 || state.SphereBoxCandidateCount >= 10000 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Broadphase did not reduce candidate pairs below the naive all-pairs counts.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for _, sphere := range state.RigidSpheres {
				if sphere.Motion.Position.X.Cmp(minX.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.X.Cmp(maxX.Add(sphere.Radius)) > 0 ||
					sphere.Motion.Position.Z.Cmp(minZ.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.Z.Cmp(maxZ.Add(sphere.Radius)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere escaped the container bounds.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere fell below the box floor.",
					}
				}
			}
			for _, box := range state.RigidBoxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box escaped the container bounds.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box fell below the box floor.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "Broadphase reduced candidate pairs while one hundred rigid spheres and one hundred rigid boxes stayed inside the open box.",
			}
		},
	}
}

func NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedHighSpeedScenario() ScenarioDefinition {
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Hundred Rigid Spheres And Hundred Rigid Boxes In Box Optimized High Speed",
		Description: "One hundred rigid spheres and one hundred rigid boxes start with high downward speed to stress-test tunneling while using the optimized broadphase path.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			spheres := make([]physics.RigidSphereBody3D, 0, 100)
			boxes := make([]physics.RigidBoxBody3D, 0, 100)
			radius := fixed.FromFraction(1, 4)
			halfExtents := geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromFraction(1, 4), fixed.FromFraction(1, 4))
			spacing := fixed.FromFraction(9, 10)
			startX := fixed.FromFraction(-18, 5)
			startZ := fixed.FromFraction(-18, 5)
			startY := fixed.FromInt(18)
			index := 0

			for layer := 0; layer < 8; layer++ {
				for row := 0; row < 5; row++ {
					for column := 0; column < 5; column++ {
						position := geometry.NewVector3(
							startX.Add(spacing.Mul(fixed.FromInt(int64(column)))),
							startY.Add(spacing.Mul(fixed.FromInt(int64(layer*2+row/3)))).Add(fixed.FromFraction(int64(row%3), 6)),
							startZ.Add(spacing.Mul(fixed.FromInt(int64(row*2+column/3)))).Add(fixed.FromFraction(int64(column%3), 6)),
						)
						downwardSpeed := fixed.FromInt(18).Add(fixed.FromFraction(int64(index%7), 1))
						sideSpeedX := fixed.FromFraction(int64((index%5)-2), 2)
						sideSpeedZ := fixed.FromFraction(int64((index%7)-3), 2)
						if index%2 == 0 {
							sphere := physics.NewRigidSphereBody3D(fixed.One, radius, position)
							sphere.Restitution = fixed.FromFraction(1, 5)
							sphere.Friction = fixed.FromFraction(1, 10)
							sphere.Motion.Velocity = geometry.NewVector3(sideSpeedX, downwardSpeed.Neg(), sideSpeedZ)
							spheres = append(spheres, sphere)
						} else {
							box := physics.NewRigidBoxBody3D(fixed.One, halfExtents, position)
							box.Restitution = fixed.FromFraction(1, 10)
							box.Motion.Velocity = geometry.NewVector3(sideSpeedZ, downwardSpeed.Neg(), sideSpeedX)
							box.Orientation = physics.NewQuaternionFromEulerXYZ(
								fixed.FromFraction(int64((index%7)-3), 10),
								fixed.FromFraction(int64((index%9)-4), 12),
								fixed.FromFraction(int64((index%11)-5), 14),
							)
							box.AngularVelocity = geometry.NewVector3(
								fixed.FromFraction(int64((index%5)-2), 2),
								fixed.FromFraction(int64((index%7)-3), 3),
								fixed.FromFraction(int64((index%9)-4), 4),
							)
							boxes = append(boxes, box)
						}
						index++
					}
				}
			}

			return SceneState{
				RigidSpheres:    spheres,
				RigidBoxes:      boxes,
				GroundTriangles: makeOpenBoxContainerTriangles(),
			}
		},
		Step: StepHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScene,
		Check: func(state SceneState) ScenarioResult {
			if len(state.RigidSpheres) != 100 || len(state.RigidBoxes) != 100 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Expected exactly one hundred rigid spheres and one hundred rigid boxes.",
				}
			}
			if !state.SphereBoxCollisionDetected || !state.RigidSphereSphereCollisionDetected || !state.RigidBoxBoxCollisionDetected {
				return ScenarioResult{
					Status:  Failed,
					Message: "High-speed scene did not exercise all collision paths.",
				}
			}
			if state.SphereSphereCandidateCount >= 4950 || state.BoxBoxCandidateCount >= 4950 || state.SphereBoxCandidateCount >= 10000 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Broadphase did not reduce candidate pairs below the naive all-pairs counts.",
				}
			}

			minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
			for _, sphere := range state.RigidSpheres {
				if sphere.Motion.Position.X.Cmp(minX.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.X.Cmp(maxX.Add(sphere.Radius)) > 0 ||
					sphere.Motion.Position.Z.Cmp(minZ.Add(sphere.Radius.Neg())) < 0 ||
					sphere.Motion.Position.Z.Cmp(maxZ.Add(sphere.Radius)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere escaped the container bounds during the high-speed test.",
					}
				}
				if sphere.Motion.Position.Y.Cmp(sphere.Radius.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid sphere tunneled below the box floor during the high-speed test.",
					}
				}
			}
			for _, box := range state.RigidBoxes {
				if box.Motion.Position.X.Cmp(minX.Add(box.HalfExtents.X.Neg())) < 0 ||
					box.Motion.Position.X.Cmp(maxX.Add(box.HalfExtents.X)) > 0 ||
					box.Motion.Position.Z.Cmp(minZ.Add(box.HalfExtents.Z.Neg())) < 0 ||
					box.Motion.Position.Z.Cmp(maxZ.Add(box.HalfExtents.Z)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box escaped the container bounds during the high-speed test.",
					}
				}
				if box.Motion.Position.Y.Cmp(box.HalfExtents.Y.Sub(fixed.FromFraction(1, 5))) < 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one rigid box tunneled below the box floor during the high-speed test.",
					}
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "High-speed rigid spheres and rigid boxes stayed inside the open box without tunneling through the floor.",
			}
		},
	}
}

func NewRigidSphereHighSpeedThinWallProjectileScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Rigid Sphere High Speed Thin Wall Projectile",
		Description: "A single rigid sphere launches at very high speed into a thin wall slab to stress-test tunneling through narrow geometry.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewRigidSphereBody3D(
				fixed.One,
				fixed.FromFraction(1, 2),
				geometry.NewVector3(fixed.FromInt(-12), fixed.FromInt(2), fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(1, 5)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero)

			return SceneState{
				RigidSphere:    sphere,
				GroundTriangles: makeThinWallSlabTriangles(),
			}
		},
		Step: StepRigidSphereHighSpeedThinWallProjectileScene,
		Check: func(state SceneState) ScenarioResult {
			const wallMaxX = 1
			const velocityLimit = 1

			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Projectile never contacted the thin wall.",
				}
			}
			if state.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(wallMaxX)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Projectile tunneled through the thin wall.",
				}
			}
			if state.RigidSphere.Motion.Velocity.X.Cmp(fixed.FromInt(velocityLimit)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Projectile kept too much forward speed after wall impact.",
				}
			}

			return ScenarioResult{
				Status:  Passed,
				Message: "High-speed projectile stayed on the impact side of the thin wall.",
			}
		},
	}
}

func NewRigidSphereHighSpeedThinWallProjectileCCDScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Rigid Sphere High Speed Thin Wall Projectile CCD",
		Description: "The same high-speed thin-wall projectile setup as scene 28, but resolved with swept sphere CCD to prevent tunneling.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewRigidSphereBody3D(
				fixed.One,
				fixed.FromFraction(1, 2),
				geometry.NewVector3(fixed.FromInt(-12), fixed.FromInt(2), fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(1, 5)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero)
			wall := geometry.NewAxisAlignedBoundingBox(
				geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(-4)),
				geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromInt(5), fixed.FromInt(4)),
			)

			return SceneState{
				RigidSphere:    sphere,
				GroundTriangles: makeThinWallSlabTriangles(),
				GroundBoxes:    []geometry.AxisAlignedBoundingBox{wall},
			}
		},
		Step: StepRigidSphereHighSpeedThinWallProjectileCCDScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "CCD projectile never contacted the thin wall.",
				}
			}
			if state.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "CCD projectile still tunneled through the thin wall.",
				}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "CCD projectile stayed on the impact side of the thin wall.",
			}
		},
	}
}

func NewRigidSphereHighSpeedThinWallProjectileMeshCCDScenario() ScenarioDefinition {
	const scenarioTicks = 120

	return ScenarioDefinition{
		Name:        "Rigid Sphere High Speed Thin Wall Projectile Mesh CCD",
		Description: "The same high-speed thin-wall projectile setup as scenes 28-29, but resolved with swept sphere CCD directly against the triangle mesh.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			sphere := physics.NewRigidSphereBody3D(
				fixed.One,
				fixed.FromFraction(1, 2),
				geometry.NewVector3(fixed.FromInt(-12), fixed.FromInt(2), fixed.Zero),
			)
			sphere.Restitution = fixed.FromFraction(1, 5)
			sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero)

			return SceneState{
				RigidSphere:    sphere,
				GroundTriangles: makeThinWallSlabTriangles(),
			}
		},
		Step: StepRigidSphereHighSpeedThinWallProjectileMeshCCDScene,
		Check: func(state SceneState) ScenarioResult {
			if !state.EverTouchedGround {
				return ScenarioResult{
					Status:  Failed,
					Message: "Mesh CCD projectile never contacted the thin wall.",
				}
			}
			if state.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
				return ScenarioResult{
					Status:  Failed,
					Message: "Mesh CCD projectile still tunneled through the thin wall.",
				}
			}
			return ScenarioResult{
				Status:  Passed,
				Message: "Mesh CCD projectile stayed on the impact side of the thin wall.",
			}
		},
	}
}

func StepSphereScene(state *SceneState) {
	if state == nil {
		return
	}

	sphere := &state.Sphere
	state.LastContact = physics.SphereTriangleContact{}

	result := physics.StepSphereBodyWithGravity(
		sphere,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		state.GroundTriangles,
	)

	if result.HadContact {
		state.LastContact = result.LastContact
		state.EverTouchedGround = true
		if state.Sphere.Motion.Velocity.Y.Cmp(fixed.FromRaw(1024)) > 0 {
			state.BounceDetected = true
			if state.Sphere.Motion.Position.Y.Cmp(state.PeakBounceHeight) > 0 {
				state.PeakBounceHeight = state.Sphere.Motion.Position.Y
			}
		}
	}

	if state.BounceDetected && state.Sphere.Motion.Position.Y.Cmp(state.PeakBounceHeight) > 0 {
		state.PeakBounceHeight = state.Sphere.Motion.Position.Y
	}
}

func StepTwoSphereCollisionScene(state *SceneState) {
	if state == nil {
		return
	}
	if len(state.Spheres) != 2 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil

	for index := range state.Spheres {
		physics.StepLinearMotion(&state.Spheres[index].Motion, physics.DefaultTimeStep)
	}

	positionA, velocityA, positionB, velocityB, contact := physics.ResolveSphereSphereContact(
		state.Spheres[0].Motion.Position,
		state.Spheres[0].Motion.Velocity,
		state.Spheres[0].Motion.InverseMass,
		state.Spheres[0].Radius,
		state.Spheres[1].Motion.Position,
		state.Spheres[1].Motion.Velocity,
		state.Spheres[1].Motion.InverseMass,
		state.Spheres[1].Radius,
		fixed.One,
	)

	if contact.Hit {
		state.SphereSphereCollisionDetected = true
		state.Spheres[0].Motion.Position = positionA
		state.Spheres[0].Motion.Velocity = velocityA
		state.Spheres[1].Motion.Position = positionB
		state.Spheres[1].Motion.Velocity = velocityB
	}

	state.Sphere = state.Spheres[0]
}

func StepSphereBoxCollisionScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil

	physics.StepLinearMotion(&state.Sphere.Motion, physics.DefaultTimeStep)
	physics.StepLinearMotion(&state.Box.Motion, physics.DefaultTimeStep)

	combinedFriction := state.Sphere.Friction.Add(state.Box.Friction)
	if combinedFriction.Cmp(fixed.One) > 0 {
		combinedFriction = fixed.One
	}

	spherePosition, sphereVelocity, boxPosition, boxVelocity, contact := physics.ResolveSphereBoxContactWithFriction(
		state.Sphere.Motion.Position,
		state.Sphere.Motion.Velocity,
		state.Sphere.Motion.InverseMass,
		state.Sphere.Radius,
		state.Box.Motion.Position,
		state.Box.Motion.Velocity,
		state.Box.Motion.InverseMass,
		state.Box.HalfExtents,
		fixed.One,
		combinedFriction,
	)

	if contact.Hit {
		state.SphereBoxCollisionDetected = true
		state.Sphere.Motion.Position = spherePosition
		state.Sphere.Motion.Velocity = sphereVelocity
		state.Box.Motion.Position = boxPosition
		state.Box.Motion.Velocity = boxVelocity
	}
}

func StepTwoSphereBoxOpposingMassScene(state *SceneState) {
	if state == nil {
		return
	}
	if len(state.Spheres) != 2 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil

	for index := range state.Spheres {
		physics.StepLinearMotion(&state.Spheres[index].Motion, physics.DefaultTimeStep)
	}
	physics.StepLinearMotion(&state.Box.Motion, physics.DefaultTimeStep)

	for index := range state.Spheres {
		spherePosition, sphereVelocity, boxPosition, boxVelocity, contact := physics.ResolveSphereBoxContact(
			state.Spheres[index].Motion.Position,
			state.Spheres[index].Motion.Velocity,
			state.Spheres[index].Motion.InverseMass,
			state.Spheres[index].Radius,
			state.Box.Motion.Position,
			state.Box.Motion.Velocity,
			state.Box.Motion.InverseMass,
			state.Box.HalfExtents,
			fixed.One,
		)

		if contact.Hit {
			state.SphereBoxCollisionDetected = true
			state.Spheres[index].Motion.Position = spherePosition
			state.Spheres[index].Motion.Velocity = sphereVelocity
			state.Box.Motion.Position = boxPosition
			state.Box.Motion.Velocity = boxVelocity
		}
	}

	state.Sphere = state.Spheres[0]
}

func StepBoxFrictionFlatSlideScene(state *SceneState) {
	if state == nil || len(state.GroundBoxes) == 0 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	result := physics.StepBoxBodyWithGravityAndFloorOverride(
		&state.Box,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		state.GroundBoxes[0],
		state.Box.Restitution,
	)
	if result.HadContact {
		state.LastContact = result.LastContact
		state.EverTouchedGround = true
	}
}

func StepSphereBoxFrictionCollisionScene(state *SceneState) {
	if state == nil || len(state.GroundBoxes) == 0 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	boxResult := physics.StepBoxBodyWithGravityAndFloorOverride(
		&state.Box,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		state.GroundBoxes[0],
		state.Box.Restitution,
	)
	if boxResult.HadContact {
		state.LastContact = boxResult.LastContact
		state.EverTouchedGround = true
	}

	physics.StepLinearMotion(&state.Sphere.Motion, physics.DefaultTimeStep)

	combinedFriction := state.Sphere.Friction.Add(state.Box.Friction)
	if combinedFriction.Cmp(fixed.One) > 0 {
		combinedFriction = fixed.One
	}

	spherePosition, sphereVelocity, boxPosition, boxVelocity, contact := physics.ResolveSphereBoxContactWithFriction(
		state.Sphere.Motion.Position,
		state.Sphere.Motion.Velocity,
		state.Sphere.Motion.InverseMass,
		state.Sphere.Radius,
		state.Box.Motion.Position,
		state.Box.Motion.Velocity,
		state.Box.Motion.InverseMass,
		state.Box.HalfExtents,
		fixed.Zero,
		combinedFriction,
	)

	if contact.Hit {
		state.SphereBoxCollisionDetected = true
		state.Sphere.Motion.Position = spherePosition
		state.Sphere.Motion.Velocity = sphereVelocity
		state.Box.Motion.Position = boxPosition
		state.Box.Motion.Velocity = boxVelocity
	}
}

func StepSphereBoxFrictionBounceCollisionScene(state *SceneState) {
	if state == nil || len(state.GroundBoxes) == 0 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	boxResult := physics.StepBoxBodyWithGravityAndFloorOverride(
		&state.Box,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		state.GroundBoxes[0],
		state.Box.Restitution,
	)
	if boxResult.HadContact {
		state.LastContact = boxResult.LastContact
		state.EverTouchedGround = true
	}

	physics.AdvanceRigidSphereBody3D(&state.RigidSphere, physics.DefaultTimeStep)

	combinedRestitution := state.RigidSphere.Restitution.Add(state.Box.Restitution)
	if combinedRestitution.Cmp(fixed.One) > 0 {
		combinedRestitution = fixed.One
	}
	combinedFriction := state.RigidSphere.Friction.Add(state.Box.Friction)
	if combinedFriction.Cmp(fixed.One) > 0 {
		combinedFriction = fixed.One
	}

	contact := physics.ResolveRigidSphereBoxContactWithFriction(
		&state.RigidSphere,
		&state.Box,
		combinedRestitution,
		combinedFriction,
	)

	if contact.Hit {
		state.SphereBoxCollisionDetected = true
	}
}

func StepHundredRigidSpheresInBoxScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	if len(state.RigidSphereTouchedGroundSet) != len(state.RigidSpheres) {
		state.RigidSphereTouchedGroundSet = make([]bool, len(state.RigidSpheres))
	}
	if len(state.RigidSphereRotationChangedSet) != len(state.RigidSpheres) {
		state.RigidSphereRotationChangedSet = make([]bool, len(state.RigidSpheres))
	}

	for index := range state.RigidSpheres {
		initialOrientation := state.RigidSpheres[index].Orientation
		result := physics.StepRigidSphereBody3DWithGravity(
			&state.RigidSpheres[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			state.GroundTriangles,
		)
		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			state.RigidSphereTouchedGroundSet[index] = true
		}
		if state.RigidSpheres[index].Orientation.W != initialOrientation.W ||
			state.RigidSpheres[index].Orientation.X != initialOrientation.X ||
			state.RigidSpheres[index].Orientation.Y != initialOrientation.Y ||
			state.RigidSpheres[index].Orientation.Z != initialOrientation.Z {
			state.RigidSphereRotationChangedSet[index] = true
		}
	}

	for pass := 0; pass < 2; pass++ {
		for first := 0; first < len(state.RigidSpheres); first++ {
			for second := first + 1; second < len(state.RigidSpheres); second++ {
				combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
				if combinedFriction.Cmp(fixed.One) > 0 {
					combinedFriction = fixed.One
				}

				contact := physics.ResolveRigidSphereSphereContactWithFriction(
					&state.RigidSpheres[first],
					&state.RigidSpheres[second],
					combinedRestitution,
					combinedFriction,
				)
				if contact.Hit {
					state.RigidSphereSphereCollisionDetected = true
				}
			}
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
		if len(state.LastContacts) > 0 {
			state.LastContact = state.LastContacts[0]
		}
	}
}

func StepHundredBoxesInBoxScene(state *SceneState) {
	if state == nil {
		return
	}

	minX, maxX, minZ, maxZ, wallHeight := openBoxContainerParameters()

	for index := range state.Boxes {
		physics.AdvanceBoxBodyWithGravity(&state.Boxes[index], physics.DefaultTimeStep, physics.StandardGravity)
		if physics.ConstrainBoxBodyToOpenContainer(&state.Boxes[index], minX, maxX, minZ, maxZ, fixed.Zero, wallHeight) {
			state.EverTouchedGround = true
		}
	}

	for pass := 0; pass < 2; pass++ {
		for first := 0; first < len(state.Boxes); first++ {
			for second := first + 1; second < len(state.Boxes); second++ {
				combinedRestitution := state.Boxes[first].Restitution.Add(state.Boxes[second].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				contact := physics.ResolveBoxBoxContact(&state.Boxes[first], &state.Boxes[second], combinedRestitution)
				if contact.Hit {
					state.BoxBoxCollisionDetected = true
				}
			}
		}
	}

	if len(state.Boxes) > 0 {
		state.Box = state.Boxes[0]
	}
}

func StepHundredRigidBoxesInBoxAngleScene(state *SceneState) {
	if state == nil {
		return
	}
	if len(state.RigidBoxRotationChangedSet) != len(state.RigidBoxes) {
		state.RigidBoxRotationChangedSet = make([]bool, len(state.RigidBoxes))
	}

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidBoxes {
		initialOrientation := state.RigidBoxes[index].Orientation
		physics.AdvanceRigidBoxBody3D(&state.RigidBoxes[index], physics.DefaultTimeStep, physics.StandardGravity)
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if state.RigidBoxes[index].Orientation.W != initialOrientation.W ||
			state.RigidBoxes[index].Orientation.X != initialOrientation.X ||
			state.RigidBoxes[index].Orientation.Y != initialOrientation.Y ||
			state.RigidBoxes[index].Orientation.Z != initialOrientation.Z {
			state.RigidBoxRotationChangedSet[index] = true
		}
	}

	for pass := 0; pass < 2; pass++ {
		for first := 0; first < len(state.RigidBoxes); first++ {
			for second := first + 1; second < len(state.RigidBoxes); second++ {
				combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
				if contact.Hit {
					state.RigidBoxBoxCollisionDetected = true
				}
			}
		}
	}

	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepFiftyRigidSpheresAndFiftyRigidBoxesInBoxScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		physics.ApplyForce(&state.RigidSpheres[index].Motion, physics.ComputeGravityForce(state.RigidSpheres[index].Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(&state.RigidSpheres[index], physics.DefaultTimeStep)
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	for index := range state.RigidBoxes {
		physics.AdvanceRigidBoxBody3D(&state.RigidBoxes[index], physics.DefaultTimeStep, physics.StandardGravity)
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	for pass := 0; pass < 2; pass++ {
		for first := 0; first < len(state.RigidSpheres); first++ {
			for second := first + 1; second < len(state.RigidSpheres); second++ {
				combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
				if combinedFriction.Cmp(fixed.One) > 0 {
					combinedFriction = fixed.One
				}
				contact := physics.ResolveRigidSphereSphereContactWithFriction(
					&state.RigidSpheres[first],
					&state.RigidSpheres[second],
					combinedRestitution,
					combinedFriction,
				)
				if contact.Hit {
					state.RigidSphereSphereCollisionDetected = true
				}
			}
		}

		for first := 0; first < len(state.RigidBoxes); first++ {
			for second := first + 1; second < len(state.RigidBoxes); second++ {
				combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
				if contact.Hit {
					state.RigidBoxBoxCollisionDetected = true
				}
			}
		}

		for sphereIndex := range state.RigidSpheres {
			for boxIndex := range state.RigidBoxes {
				combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
				if combinedRestitution.Cmp(fixed.One) > 0 {
					combinedRestitution = fixed.One
				}
				contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
					&state.RigidSpheres[sphereIndex],
					&state.RigidBoxes[boxIndex],
					combinedRestitution,
					state.RigidSpheres[sphereIndex].Friction,
				)
				if contact.Hit {
					state.SphereBoxCollisionDetected = true
				}
			}
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepMultiSphereScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Spheres))
	if len(state.BounceDetectedSet) != len(state.Spheres) {
		state.BounceDetectedSet = make([]bool, len(state.Spheres))
	}

	for index := range state.Spheres {
		result := physics.StepSphereBodyWithGravity(
			&state.Spheres[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			state.GroundTriangles,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Spheres[index].Motion.Velocity.Y.Cmp(fixed.FromRaw(1024)) > 0 {
				state.BounceDetectedSet[index] = true
			}
		}
	}

	if len(state.Spheres) > 0 {
		state.Sphere = state.Spheres[0]
		if len(state.LastContacts) > 0 {
			state.LastContact = state.LastContacts[0]
		}
	}
}

func StepThreeSphereDifferentFloorBounceScene(state *SceneState) {
	if state == nil {
		return
	}

	floorRestitutions := []fixed.Fixed{
		fixed.Zero,
		fixed.FromFraction(1, 4),
		fixed.FromFraction(3, 5),
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Spheres))
	if len(state.BounceDetectedSet) != len(state.Spheres) {
		state.BounceDetectedSet = make([]bool, len(state.Spheres))
	}
	if len(state.PeakBounceHeights) != len(state.Spheres) {
		state.PeakBounceHeights = make([]fixed.Fixed, len(state.Spheres))
	}

	for index := range state.Spheres {
		effectiveRestitution := state.Spheres[index].Restitution.Add(floorRestitutions[index])
		if effectiveRestitution.Cmp(fixed.One) > 0 {
			effectiveRestitution = fixed.One
		}

		result := physics.StepSphereBodyWithGravityAndRestitutionOverride(
			&state.Spheres[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			state.GroundTriangles[index*2:index*2+2],
			effectiveRestitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Spheres[index].Motion.Velocity.Y.Cmp(fixed.FromRaw(1024)) > 0 {
				state.BounceDetectedSet[index] = true
			}
		}

		if state.BounceDetectedSet[index] && state.Spheres[index].Motion.Position.Y.Cmp(state.PeakBounceHeights[index]) > 0 {
			state.PeakBounceHeights[index] = state.Spheres[index].Motion.Position.Y
		}
	}

	if len(state.Spheres) > 0 {
		state.Sphere = state.Spheres[0]
		if len(state.LastContacts) > 0 {
			state.LastContact = state.LastContacts[0]
		}
	}
}

func StepThreeBoxDifferentFloorBounceScene(state *SceneState) {
	if state == nil {
		return
	}

	floorRestitutions := []fixed.Fixed{
		fixed.Zero,
		fixed.FromFraction(1, 4),
		fixed.FromFraction(3, 5),
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Boxes))
	if len(state.BoxBounceDetectedSet) != len(state.Boxes) {
		state.BoxBounceDetectedSet = make([]bool, len(state.Boxes))
	}
	if len(state.BoxPeakBounceHeights) != len(state.Boxes) {
		state.BoxPeakBounceHeights = make([]fixed.Fixed, len(state.Boxes))
	}

	for index := range state.Boxes {
		effectiveRestitution := state.Boxes[index].Restitution.Add(floorRestitutions[index])
		if effectiveRestitution.Cmp(fixed.One) > 0 {
			effectiveRestitution = fixed.One
		}

		result := physics.StepBoxBodyWithGravityAndFloorOverride(
			&state.Boxes[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			state.GroundBoxes[index],
			effectiveRestitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Boxes[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.BoxBounceDetectedSet[index] = true
			}
		}

		if state.BoxBounceDetectedSet[index] && state.Boxes[index].Motion.Position.Y.Cmp(state.BoxPeakBounceHeights[index]) > 0 {
			state.BoxPeakBounceHeights[index] = state.Boxes[index].Motion.Position.Y
		}
	}

	if len(state.Boxes) > 0 {
		state.Box = state.Boxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepThreeBoxSameSlopeBounceScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Boxes))
	if len(state.BoxBounceDetectedSet) != len(state.Boxes) {
		state.BoxBounceDetectedSet = make([]bool, len(state.Boxes))
	}
	if len(state.BoxPeakBounceHeights) != len(state.Boxes) {
		state.BoxPeakBounceHeights = make([]fixed.Fixed, len(state.Boxes))
	}

	minX, maxX, minZ, maxZ, slopeRisePerRun := thirtyDegreeSideSlopeParameters()

	for index := range state.Boxes {
		result := physics.StepBoxBodyWithGravityAndSideSlopeOverride(
			&state.Boxes[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			minX,
			maxX,
			minZ,
			maxZ,
			slopeRisePerRun,
			state.Boxes[index].Restitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Boxes[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.BoxBounceDetectedSet[index] = true
			}
		}

		if state.BoxBounceDetectedSet[index] && state.Boxes[index].Motion.Position.Y.Cmp(state.BoxPeakBounceHeights[index]) > 0 {
			state.BoxPeakBounceHeights[index] = state.Boxes[index].Motion.Position.Y
		}
	}

	if len(state.Boxes) > 0 {
		state.Box = state.Boxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepThreeBoxSameSlopeAngleComparisonScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Boxes))
	if len(state.BoxBounceDetectedSet) != len(state.Boxes) {
		state.BoxBounceDetectedSet = make([]bool, len(state.Boxes))
	}
	if len(state.BoxPeakBounceHeights) != len(state.Boxes) {
		state.BoxPeakBounceHeights = make([]fixed.Fixed, len(state.Boxes))
	}
	if len(state.BoxInitialRotations) != len(state.Boxes) {
		state.BoxInitialRotations = make([]fixed.Fixed, len(state.Boxes))
	}
	if len(state.BoxRotationChangedSet) != len(state.Boxes) {
		state.BoxRotationChangedSet = make([]bool, len(state.Boxes))
	}

	_, _, _, _, slopeRisePerRun := thirtyDegreeSideSlopeParameters()
	planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero)
	planeNormal := geometry.NewVector3(slopeRisePerRun.Neg(), fixed.One, fixed.Zero).Normalize()

	for index := range state.Boxes {
		result := physics.StepOrientedBoxBodyWithGravityAndPlaneOverride(
			&state.Boxes[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			planePoint,
			planeNormal,
			state.Boxes[index].Restitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Boxes[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.BoxBounceDetectedSet[index] = true
			}
		}

		if state.BoxBounceDetectedSet[index] && state.Boxes[index].Motion.Position.Y.Cmp(state.BoxPeakBounceHeights[index]) > 0 {
			state.BoxPeakBounceHeights[index] = state.Boxes[index].Motion.Position.Y
		}

		if state.Boxes[index].RotationZ.Sub(state.BoxInitialRotations[index]).Abs().Cmp(fixed.FromRaw(1<<20)) > 0 {
			state.BoxRotationChangedSet[index] = true
		}
	}

	if len(state.Boxes) > 0 {
		state.Box = state.Boxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepNineBoxFlatAngleComparisonScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.Boxes))
	if len(state.BoxBounceDetectedSet) != len(state.Boxes) {
		state.BoxBounceDetectedSet = make([]bool, len(state.Boxes))
	}
	if len(state.BoxPeakBounceHeights) != len(state.Boxes) {
		state.BoxPeakBounceHeights = make([]fixed.Fixed, len(state.Boxes))
	}
	if len(state.BoxInitialRotations) != len(state.Boxes) {
		state.BoxInitialRotations = make([]fixed.Fixed, len(state.Boxes))
	}
	if len(state.BoxRotationChangedSet) != len(state.Boxes) {
		state.BoxRotationChangedSet = make([]bool, len(state.Boxes))
	}

	planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero)
	planeNormal := geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)

	for index := range state.Boxes {
		result := physics.StepOrientedBoxBodyWithGravityAndPlaneOverride(
			&state.Boxes[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			planePoint,
			planeNormal,
			state.Boxes[index].Restitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.Boxes[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.BoxBounceDetectedSet[index] = true
			}
		}

		if state.BoxBounceDetectedSet[index] && state.Boxes[index].Motion.Position.Y.Cmp(state.BoxPeakBounceHeights[index]) > 0 {
			state.BoxPeakBounceHeights[index] = state.Boxes[index].Motion.Position.Y
		}

		if state.Boxes[index].RotationZ.Sub(state.BoxInitialRotations[index]).Abs().Cmp(fixed.FromRaw(1<<20)) > 0 {
			state.BoxRotationChangedSet[index] = true
		}
	}

	if len(state.Boxes) > 0 {
		state.Box = state.Boxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepRigidBox3DFlatBounceScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	initialOrientation := state.RigidBox.Orientation
	result := physics.StepRigidBoxBody3DWithGravityAndPlaneOverride(
		&state.RigidBox,
		physics.DefaultTimeStep,
		physics.StandardGravity,
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
		state.RigidBox.Restitution,
	)

	if result.HadContact {
		state.LastContact = result.LastContact
		state.EverTouchedGround = true
		if state.RigidBox.Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
			state.RigidBoxBounceDetected = true
		}
	}

	if state.RigidBoxBounceDetected && state.RigidBox.Motion.Position.Y.Cmp(state.RigidBoxPeakBounceHeight) > 0 {
		state.RigidBoxPeakBounceHeight = state.RigidBox.Motion.Position.Y
	}

	if state.RigidBox.Orientation.W != initialOrientation.W ||
		state.RigidBox.Orientation.X != initialOrientation.X ||
		state.RigidBox.Orientation.Y != initialOrientation.Y ||
		state.RigidBox.Orientation.Z != initialOrientation.Z {
		state.RigidBoxRotationChanged = true
	}
}

func StepNineRigidBox3DFlatAngleComparisonScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidBoxes))
	if len(state.RigidBoxBounceDetectedSet) != len(state.RigidBoxes) {
		state.RigidBoxBounceDetectedSet = make([]bool, len(state.RigidBoxes))
	}
	if len(state.RigidBoxPeakBounceHeights) != len(state.RigidBoxes) {
		state.RigidBoxPeakBounceHeights = make([]fixed.Fixed, len(state.RigidBoxes))
	}
	if len(state.RigidBoxRotationChangedSet) != len(state.RigidBoxes) {
		state.RigidBoxRotationChangedSet = make([]bool, len(state.RigidBoxes))
	}

	planePoint := geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero)
	planeNormal := geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)

	for index := range state.RigidBoxes {
		initialOrientation := state.RigidBoxes[index].Orientation
		result := physics.StepRigidBoxBody3DWithGravityAndPlaneOverride(
			&state.RigidBoxes[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			planePoint,
			planeNormal,
			state.RigidBoxes[index].Restitution,
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			if state.RigidBoxes[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.RigidBoxBounceDetectedSet[index] = true
			}
		}

		if state.RigidBoxBounceDetectedSet[index] &&
			state.RigidBoxes[index].Motion.Position.Y.Cmp(state.RigidBoxPeakBounceHeights[index]) > 0 {
			state.RigidBoxPeakBounceHeights[index] = state.RigidBoxes[index].Motion.Position.Y
		}

		if state.RigidBoxes[index].Orientation.W != initialOrientation.W ||
			state.RigidBoxes[index].Orientation.X != initialOrientation.X ||
			state.RigidBoxes[index].Orientation.Y != initialOrientation.Y ||
			state.RigidBoxes[index].Orientation.Z != initialOrientation.Z {
			state.RigidBoxRotationChangedSet[index] = true
		}
	}

	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func StepNineRigidSphere3DSmallSlopeScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	if len(state.RigidSphereTouchedGroundSet) != len(state.RigidSpheres) {
		state.RigidSphereTouchedGroundSet = make([]bool, len(state.RigidSpheres))
	}
	if len(state.RigidSphereBounceDetectedSet) != len(state.RigidSpheres) {
		state.RigidSphereBounceDetectedSet = make([]bool, len(state.RigidSpheres))
	}
	if len(state.RigidSpherePeakBounceHeights) != len(state.RigidSpheres) {
		state.RigidSpherePeakBounceHeights = make([]fixed.Fixed, len(state.RigidSpheres))
	}
	if len(state.RigidSphereRotationChangedSet) != len(state.RigidSpheres) {
		state.RigidSphereRotationChangedSet = make([]bool, len(state.RigidSpheres))
	}

	for index := range state.RigidSpheres {
		initialOrientation := state.RigidSpheres[index].Orientation
		result := physics.StepRigidSphereBody3DWithGravity(
			&state.RigidSpheres[index],
			physics.DefaultTimeStep,
			physics.StandardGravity,
			state.GroundTriangles[index*2:index*2+2],
		)

		if result.HadContact {
			state.LastContacts[index] = result.LastContact
			state.LastContact = result.LastContact
			state.EverTouchedGround = true
			state.RigidSphereTouchedGroundSet[index] = true
			if state.RigidSpheres[index].Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
				state.RigidSphereBounceDetectedSet[index] = true
			}
		}

		if state.RigidSphereBounceDetectedSet[index] &&
			state.RigidSpheres[index].Motion.Position.Y.Cmp(state.RigidSpherePeakBounceHeights[index]) > 0 {
			state.RigidSpherePeakBounceHeights[index] = state.RigidSpheres[index].Motion.Position.Y
		}

		if state.RigidSpheres[index].Orientation.W != initialOrientation.W ||
			state.RigidSpheres[index].Orientation.X != initialOrientation.X ||
			state.RigidSpheres[index].Orientation.Y != initialOrientation.Y ||
			state.RigidSpheres[index].Orientation.Z != initialOrientation.Z {
			state.RigidSphereRotationChangedSet[index] = true
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}
}

func makeFlatGroundTriangles() []geometry.Triangle {
	min := fixed.FromInt(-20)
	max := fixed.FromInt(20)
	y := fixed.Zero

	a := geometry.NewVector3(min, y, min)
	b := geometry.NewVector3(max, y, min)
	c := geometry.NewVector3(min, y, max)
	d := geometry.NewVector3(max, y, max)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}

func makeThirtyDegreeSlopeTriangles() []geometry.Triangle {
	minX, maxX, minZ, maxZ, slopeRisePerRun := thirtyDegreeSideSlopeParameters()

	lowY := minX.Mul(slopeRisePerRun)
	highY := maxX.Mul(slopeRisePerRun)

	a := geometry.NewVector3(minX, lowY, minZ)
	b := geometry.NewVector3(maxX, highY, minZ)
	c := geometry.NewVector3(minX, lowY, maxZ)
	d := geometry.NewVector3(maxX, highY, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}

func thirtyDegreeSideSlopeParameters() (fixed.Fixed, fixed.Fixed, fixed.Fixed, fixed.Fixed, fixed.Fixed) {
	return fixed.FromInt(-60),
		fixed.FromInt(60),
		fixed.FromInt(-60),
		fixed.FromInt(60),
		fixed.FromFraction(57735, 100000)
}

func makeThreeFloorBounceTriangles() []geometry.Triangle {
	return append(
		append(makeFlatStripTriangles(fixed.FromInt(-20), fixed.FromInt(-8)), makeFlatStripTriangles(fixed.FromInt(-6), fixed.FromInt(6))...),
		makeFlatStripTriangles(fixed.FromInt(8), fixed.FromInt(20))...,
	)
}

func makeFlatStripTriangles(minX, maxX fixed.Fixed) []geometry.Triangle {
	minZ := fixed.FromInt(-10)
	maxZ := fixed.FromInt(10)
	y := fixed.Zero

	a := geometry.NewVector3(minX, y, minZ)
	b := geometry.NewVector3(maxX, y, minZ)
	c := geometry.NewVector3(minX, y, maxZ)
	d := geometry.NewVector3(maxX, y, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}

func makeThreeFloorBounceBoxes() []geometry.AxisAlignedBoundingBox {
	return []geometry.AxisAlignedBoundingBox{
		makeFlatStripBox(fixed.FromInt(-20), fixed.FromInt(-8)),
		makeFlatStripBox(fixed.FromInt(-6), fixed.FromInt(6)),
		makeFlatStripBox(fixed.FromInt(8), fixed.FromInt(20)),
	}
}

func makeFlatStripBox(minX, maxX fixed.Fixed) geometry.AxisAlignedBoundingBox {
	return geometry.NewAxisAlignedBoundingBox(
		geometry.NewVector3(minX, fixed.FromInt(-1), fixed.FromInt(-10)),
		geometry.NewVector3(maxX, fixed.Zero, fixed.FromInt(10)),
	)
}

func openBoxContainerParameters() (fixed.Fixed, fixed.Fixed, fixed.Fixed, fixed.Fixed, fixed.Fixed) {
	return fixed.FromInt(-6), fixed.FromInt(6), fixed.FromInt(-6), fixed.FromInt(6), fixed.FromInt(30)
}

func makeOpenBoxContainerTriangles() []geometry.Triangle {
	minX, maxX, minZ, maxZ, wallHeight := openBoxContainerParameters()
	y0 := fixed.Zero
	y1 := wallHeight

	floorA := geometry.NewVector3(minX, y0, minZ)
	floorB := geometry.NewVector3(maxX, y0, minZ)
	floorC := geometry.NewVector3(minX, y0, maxZ)
	floorD := geometry.NewVector3(maxX, y0, maxZ)

	leftA := geometry.NewVector3(minX, y0, minZ)
	leftB := geometry.NewVector3(minX, y0, maxZ)
	leftC := geometry.NewVector3(minX, y1, minZ)
	leftD := geometry.NewVector3(minX, y1, maxZ)

	rightA := geometry.NewVector3(maxX, y0, minZ)
	rightB := geometry.NewVector3(maxX, y0, maxZ)
	rightC := geometry.NewVector3(maxX, y1, minZ)
	rightD := geometry.NewVector3(maxX, y1, maxZ)

	backA := geometry.NewVector3(minX, y0, minZ)
	backB := geometry.NewVector3(maxX, y0, minZ)
	backC := geometry.NewVector3(minX, y1, minZ)
	backD := geometry.NewVector3(maxX, y1, minZ)

	frontA := geometry.NewVector3(minX, y0, maxZ)
	frontB := geometry.NewVector3(maxX, y0, maxZ)
	frontC := geometry.NewVector3(minX, y1, maxZ)
	frontD := geometry.NewVector3(maxX, y1, maxZ)

	return []geometry.Triangle{
		geometry.NewTriangle(floorA, floorC, floorB),
		geometry.NewTriangle(floorC, floorD, floorB),
		geometry.NewTriangle(leftA, leftC, leftB),
		geometry.NewTriangle(leftB, leftC, leftD),
		geometry.NewTriangle(rightA, rightB, rightC),
		geometry.NewTriangle(rightB, rightD, rightC),
		geometry.NewTriangle(backA, backB, backC),
		geometry.NewTriangle(backB, backD, backC),
		geometry.NewTriangle(frontA, frontC, frontB),
		geometry.NewTriangle(frontB, frontC, frontD),
	}
}

func makeThinWallSlabTriangles() []geometry.Triangle {
	minX := fixed.FromFraction(-1, 4)
	maxX := fixed.FromFraction(1, 4)
	minY := fixed.Zero
	maxY := fixed.FromInt(5)
	minZ := fixed.FromInt(-4)
	maxZ := fixed.FromInt(4)

	a := geometry.NewVector3(minX, minY, minZ)
	b := geometry.NewVector3(maxX, minY, minZ)
	c := geometry.NewVector3(minX, maxY, minZ)
	d := geometry.NewVector3(maxX, maxY, minZ)
	e := geometry.NewVector3(minX, minY, maxZ)
	f := geometry.NewVector3(maxX, minY, maxZ)
	g := geometry.NewVector3(minX, maxY, maxZ)
	h := geometry.NewVector3(maxX, maxY, maxZ)

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
		geometry.NewTriangle(d, g, h),
		geometry.NewTriangle(a, b, e),
		geometry.NewTriangle(e, b, f),
	}
}

func makeSlopePatchTriangles(patches []smallSlopePatchSpec) []geometry.Triangle {
	triangles := make([]geometry.Triangle, 0, len(patches)*2)
	for _, patch := range patches {
		triangles = append(triangles, makeSmallSlopePatchTriangles(patch)...)
	}
	return triangles
}

type smallSlopePatchSpec struct {
	Center            geometry.Vector3
	DownhillDirection geometry.Vector3
	SlopePerUnit      fixed.Fixed
}

func makeNineSmallSlopePatchSpecs() []smallSlopePatchSpec {
	return []smallSlopePatchSpec{
		{Center: geometry.NewVector3(fixed.FromInt(-18), fixed.Zero, fixed.FromInt(-18)), DownhillDirection: geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero), SlopePerUnit: fixed.FromFraction(176327, 1000000)},
		{Center: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.FromInt(-18)), DownhillDirection: geometry.NewVector3(fixed.One, fixed.Zero, fixed.One).Normalize(), SlopePerUnit: fixed.FromFraction(120000, 1000000)},
		{Center: geometry.NewVector3(fixed.FromInt(18), fixed.Zero, fixed.FromInt(-18)), DownhillDirection: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One), SlopePerUnit: fixed.FromFraction(220000, 1000000)},
		{Center: geometry.NewVector3(fixed.FromInt(-18), fixed.Zero, fixed.Zero), DownhillDirection: geometry.NewVector3(fixed.One, fixed.Zero, fixed.One.Neg()).Normalize(), SlopePerUnit: fixed.FromFraction(150000, 1000000)},
		{Center: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero), DownhillDirection: geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero), SlopePerUnit: fixed.FromFraction(200000, 1000000)},
		{Center: geometry.NewVector3(fixed.FromInt(18), fixed.Zero, fixed.Zero), DownhillDirection: geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.One).Normalize(), SlopePerUnit: fixed.FromFraction(135000, 1000000)},
		{Center: geometry.NewVector3(fixed.FromInt(-18), fixed.Zero, fixed.FromInt(18)), DownhillDirection: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg()), SlopePerUnit: fixed.FromFraction(240000, 1000000)},
		{Center: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.FromInt(18)), DownhillDirection: geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.One.Neg()).Normalize(), SlopePerUnit: fixed.FromFraction(165000, 1000000)},
		{Center: geometry.NewVector3(fixed.FromInt(18), fixed.Zero, fixed.FromInt(18)), DownhillDirection: geometry.NewVector3(fixed.One, fixed.Zero, fixed.One.Neg()).Normalize(), SlopePerUnit: fixed.FromFraction(190000, 1000000)},
	}
}

func makeSmallSlopePatchTriangles(patch smallSlopePatchSpec) []geometry.Triangle {
	halfWidth := fixed.FromInt(7)
	aOffset := geometry.NewVector3(halfWidth.Neg(), fixed.Zero, halfWidth.Neg())
	bOffset := geometry.NewVector3(halfWidth, fixed.Zero, halfWidth.Neg())
	cOffset := geometry.NewVector3(halfWidth.Neg(), fixed.Zero, halfWidth)
	dOffset := geometry.NewVector3(halfWidth, fixed.Zero, halfWidth)

	a := patch.Center.Add(withSlopeHeight(aOffset, patch.DownhillDirection, patch.SlopePerUnit))
	b := patch.Center.Add(withSlopeHeight(bOffset, patch.DownhillDirection, patch.SlopePerUnit))
	c := patch.Center.Add(withSlopeHeight(cOffset, patch.DownhillDirection, patch.SlopePerUnit))
	d := patch.Center.Add(withSlopeHeight(dOffset, patch.DownhillDirection, patch.SlopePerUnit))

	return []geometry.Triangle{
		geometry.NewTriangle(a, c, b),
		geometry.NewTriangle(c, d, b),
	}
}

func withSlopeHeight(offset geometry.Vector3, downhillDirection geometry.Vector3, slopePerUnit fixed.Fixed) geometry.Vector3 {
	height := offset.Dot(downhillDirection).Mul(slopePerUnit).Neg()
	return geometry.NewVector3(offset.X, height, offset.Z)
}
