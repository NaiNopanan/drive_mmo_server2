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
	const scenarioTicks = 360

	return ScenarioDefinition{
		Name:        "Three Sphere Drop",
		Description: "Three spheres fall together from different heights onto the same flat floor and should all settle.",
		MaxTicks:    scenarioTicks,
		Setup: func() SceneState {
			return SceneState{
				Spheres: []physics.SphereBody{
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
				},
				GroundTriangles: makeFlatGroundTriangles(),
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

				verticalSpeed := sphere.Motion.Velocity.Y.Abs()
				if verticalSpeed.Cmp(fixed.FromRaw(1024)) > 0 {
					return ScenarioResult{
						Status:  Failed,
						Message: "At least one sphere still had too much vertical speed at the end.",
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
				Message: "All three spheres fell and settled on the floor.",
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
	state.EverTouchedGround = false

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
		}
	}

	if len(state.Spheres) > 0 {
		state.Sphere = state.Spheres[0]
		if len(state.LastContacts) > 0 {
			state.LastContact = state.LastContacts[0]
		}
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
