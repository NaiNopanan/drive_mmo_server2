package scenario

import (
	"fmt"
	"time"

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
	BroadphaseDebugCells               []geometry.AxisAlignedBoundingBox
	LastContact                        physics.SphereTriangleContact
	LastContacts                       []physics.SphereTriangleContact
	EverTouchedGround                  bool
	BounceDetected                     bool
	SphereSphereCollisionDetected      bool
	SphereBoxCollisionDetected         bool
	BoxBoxCollisionDetected            bool
	BounceDetectedSet                  []bool
	PeakBounceHeight                   fixed.Fixed
	PeakBounceHeights                  []fixed.Fixed
	RigidSphereBounceDetected          bool
	RigidSphereCCDHitDetected          bool
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
	RigidBoxCCDHitDetected             bool
	RigidBoxBoxCollisionDetected       bool
	RigidBoxPeakBounceHeight           fixed.Fixed
	RigidBoxRotationChanged            bool
	RigidBoxBounceDetectedSet          []bool
	RigidBoxPeakBounceHeights          []fixed.Fixed
	RigidBoxRotationChangedSet         []bool
	BroadphaseCellCount                int
	SphereSphereCandidateCount         int
	BoxBoxCandidateCount               int
	SphereBoxCandidateCount            int
	SphereSphereHitCount               int
	BoxBoxHitCount                     int
	SphereBoxHitCount                  int
	CCDContactDetected                 bool
	CCDTimeOfImpact                    fixed.Fixed
	SleepingSphereCount                int
	SleepingBoxCount                   int
	ActiveCCDSphereCount               int
	ActiveCCDBoxCount                  int
	ActiveAngularRiskCCDBoxCount       int
	ActiveDiscreteSphereCount          int
	ActiveDiscreteBoxCount             int
	EverActivatedCCDSphere             bool
	EverActivatedCCDBox                bool
	EverActivatedAngularRiskCCDBox     bool
	EverSleptSphere                    bool
	EverSleptBox                       bool
	SleepingSphereSphereSkipCount      int
	SleepingBoxBoxSkipCount            int
	SleepingSphereBoxSkipCount         int
	EverSkippedSleepingPairs           bool
	BoxCCDPrecheckCount                int
	BoxCCDPrecheckRejectCount          int
	BoxCCDMeshSweepCount               int
	EverRanBoxCCDPrecheck              bool
	EverRejectedBoxCCDPrecheck         bool
	EverExecutedBoxCCDMeshSweep        bool
	SphereCCDPrecheckCount             int
	SphereCCDPrecheckRejectCount       int
	SphereCCDMeshSweepCount            int
	EverRanSphereCCDPrecheck           bool
	EverRejectedSphereCCDPrecheck      bool
	EverExecutedSphereCCDMeshSweep     bool
	SphereCCDHysteresisHoldCount       int
	BoxCCDHysteresisHoldCount          int
	SphereSleepHysteresisHoldCount     int
	BoxSleepHysteresisHoldCount        int
	EverHeldSphereCCDHysteresis        bool
	EverHeldBoxCCDHysteresis           bool
	EverHeldSphereSleepHysteresis      bool
	EverHeldBoxSleepHysteresis         bool
	PersistentContactCache             []PersistentContactCacheEntry
	PersistentPairReuseCount           int
	WarmStartedPairCount               int
	SleepingIslandCount                int
	LargestSleepingIslandSize          int
	EverReusedPersistentPairs          bool
	EverWarmStartedPairs               bool
	EverBuiltSleepingIsland            bool
	EverSleptIsland                    bool
	PhaseIntegrationNanos              int64
	PhaseCCDNanos                      int64
	PhaseBroadphaseNanos               int64
	PhaseSolverNanos                   int64
	PhaseSleepingNanos                 int64
	SolverPass1PairCount               int
	SolverPass2PairCount               int
	EverReducedSolverPass2             bool
	CCDBudgetSkipCount                 int
	SolverFrontierBudgetCount          int
	EverAppliedCCDBudget               bool
	EverAppliedSolverBudget            bool
	VehicleChassis                     physics.RigidBoxBody3D
	VehicleProbeLocalOffsets           []geometry.Vector3
	VehicleWheelRadius                 fixed.Fixed
	VehicleWheelWidth                  fixed.Fixed
	VehicleUseWheelCollider            bool
	VehicleWheelCorrectionClamp        fixed.Fixed
	VehicleWheelProbes                 []VehicleWheelProbeState
	VehicleGroundedProbeCount          int
	VehicleAverageCompression          fixed.Fixed
	VehicleUprightDot                  fixed.Fixed
	VehicleSettled                     bool
	VehicleEverHitWall                 bool
	VehicleThrottleInput               fixed.Fixed
	VehicleSteerInput                  fixed.Fixed
	VehicleFrontGroundedProbeCount     int
	VehicleRearGroundedProbeCount      int
	VehicleSuspensionRestLength        fixed.Fixed
	VehicleSuspensionSweepDistance     fixed.Fixed
	VehicleSuspensionSpring            fixed.Fixed
	VehicleSuspensionDamper            fixed.Fixed
	VehicleDriveForce                  fixed.Fixed
	VehicleEngineBrake                 fixed.Fixed
	VehicleFrontDriveShare             fixed.Fixed
	VehicleFrontGrip                   fixed.Fixed
	VehicleRearGrip                    fixed.Fixed
	VehicleAntiRoll                    fixed.Fixed
	VehicleMaxSteerAngle               fixed.Fixed
	VehicleRearSteerAssist             fixed.Fixed
	VehicleWheelGroundedGraceTicks     []int
	VehicleWallContactActive           bool
}

type PersistentContactCacheEntry struct {
	Key            uint64
	Point          geometry.Vector3
	Normal         geometry.Vector3
	NormalImpulse  fixed.Fixed
	TangentImpulse geometry.Vector3
}

type VehicleWheelProbeState struct {
	LocalOffset   geometry.Vector3
	WorldPosition geometry.Vector3
	ContactPoint  geometry.Vector3
	ContactNormal geometry.Vector3
	WheelRight    geometry.Vector3
	Compression   fixed.Fixed
	Grounded      bool
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
	Definition          ScenarioDefinition
	State               SceneState
	Tick                int
	Finished            bool
	LastResult          ScenarioResult
	LastStepDuration    time.Duration
	AccumulatedStepTime time.Duration
	WallClockStartTime  time.Time
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
	r.LastStepDuration = 0
	r.AccumulatedStepTime = 0
	r.WallClockStartTime = time.Now()
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

	start := time.Now()
	r.Definition.Step(&r.State)
	r.LastStepDuration = time.Since(start)
	r.AccumulatedStepTime += r.LastStepDuration
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
