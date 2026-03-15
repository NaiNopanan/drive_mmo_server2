package scenario

import (
	"fmt"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

type ScenarioResultStatus int

const (
	Running ScenarioResultStatus = iota
	Passed
	Failed
)

type ScenarioResult struct {
	Status  ScenarioResultStatus
	Message string
}

type SceneState struct {
	Tick                               uint64
	Sphere                             physics.SphereBody
	Spheres                            []physics.SphereBody
	RigidSphere                        physics.RigidSphereBody3D
	RigidSpheres                       []physics.RigidSphereBody3D
	Box                                physics.BoxBody
	Boxes                              []physics.BoxBody
	RigidBox                           physics.RigidBoxBody3D
	RigidBoxes                         []physics.RigidBoxBody3D
	GroundTriangles                    []geometry.Triangle
	GroundBoxes                        []geometry.AxisAlignedBoundingBox
	LastContact                        physics.SphereTriangleContact
	LastContacts                       []physics.SphereTriangleContact
	EverTouchedGround                  bool
	BounceDetected                     bool
	SphereSphereCollisionDetected      bool
	SphereBoxCollisionDetected         bool
	BounceDetectedSet                  []bool
	PeakBounceHeight                   fixed.Fixed
	PeakBounceHeights                  []fixed.Fixed
	RigidSphereBounceDetected          bool
	RigidSphereSphereCollisionDetected bool
	RigidSpherePeakBounceHeight        fixed.Fixed
	RigidSphereRotationChanged         bool
	RigidSphereTouchedGroundSet        []bool
	RigidSphereBounceDetectedSet       []bool
	RigidSpherePeakBounceHeights       []fixed.Fixed
	RigidSphereRotationChangedSet      []bool
	BoxBounceDetectedSet               []bool
	BoxPeakBounceHeights               []fixed.Fixed
	BoxInitialRotations                []fixed.Fixed
	BoxRotationChangedSet              []bool
	RigidBoxBounceDetected             bool
	RigidBoxPeakBounceHeight           fixed.Fixed
	RigidBoxRotationChanged            bool
	RigidBoxBounceDetectedSet          []bool
	RigidBoxPeakBounceHeights          []fixed.Fixed
	RigidBoxRotationChangedSet         []bool
}

type ScenarioDefinition struct {
	Name        string
	Description string
	MaxTicks    int
	Setup       func() SceneState
	Step        func(*SceneState)
	Check       func(SceneState) ScenarioResult
}

type ScenarioRunner struct {
	Definition ScenarioDefinition
	State      SceneState
	Tick       int
	Finished   bool
	LastResult ScenarioResult
}

func NewScenarioRunner(definition ScenarioDefinition) *ScenarioRunner {
	runner := &ScenarioRunner{
		Definition: definition,
		LastResult: ScenarioResult{
			Status:  Running,
			Message: "Initializing",
		},
	}
	runner.Reset()
	runner.LastResult = ScenarioResult{
		Status:  Running,
		Message: "Initializing",
	}
	return runner
}

func (r *ScenarioRunner) Reset() {
	if r == nil {
		return
	}

	r.State = r.Definition.Setup()
	r.State.Tick = 0
	r.Tick = 0
	r.Finished = false
	r.LastResult = ScenarioResult{
		Status:  Running,
		Message: "Reset",
	}
}

func (r *ScenarioRunner) Step() {
	if r == nil || r.Finished {
		return
	}

	if r.Tick >= r.Definition.MaxTicks {
		r.finish()
		return
	}

	r.Definition.Step(&r.State)
	r.Tick++
	r.State.Tick = uint64(r.Tick)

	if r.Tick >= r.Definition.MaxTicks {
		r.finish()
		return
	}

	r.LastResult = ScenarioResult{
		Status:  Running,
		Message: fmt.Sprintf("Step %d/%d", r.Tick, r.Definition.MaxTicks),
	}
}

func (r *ScenarioRunner) finish() {
	r.LastResult = r.Definition.Check(r.State)
	r.Finished = true
}
