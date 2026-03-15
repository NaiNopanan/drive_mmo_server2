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
