package scenario_test

import (
	"reflect"
	"testing"

	"server2/internal/scenario"
)

func TestSphereDropScenarioFallsAndSettlesOnFlatGround(t *testing.T) {
	definition := scenario.NewSphereDropOnFlatGroundScenario()
	runner := scenario.NewScenarioRunner(definition)

	if runner.State.Sphere.Motion.Position.X != 0 || runner.State.Sphere.Motion.Position.Z != 0 {
		t.Fatalf("expected sphere to start centered on X/Z, got position=%v", runner.State.Sphere.Motion.Position)
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.EverTouchedGround {
		t.Fatalf("expected sphere to touch ground")
	}

	if !runner.State.Sphere.Grounded {
		t.Fatalf("expected sphere to be grounded at the end")
	}
}

func TestSphereBounceScenarioTouchesGroundAndBounces(t *testing.T) {
	definition := scenario.NewSphereBounceOnFlatGroundScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected bounce scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.EverTouchedGround {
		t.Fatalf("expected sphere to touch ground")
	}

	if !runner.State.BounceDetected {
		t.Fatalf("expected sphere to bounce upward after contact")
	}

	if runner.State.PeakBounceHeight.Cmp(runner.State.Sphere.Radius) <= 0 {
		t.Fatalf("expected bounce peak above ground height, got %v", runner.State.PeakBounceHeight)
	}
}

func TestThreeSphereDropScenarioBouncesAndSettles(t *testing.T) {
	definition := scenario.NewThreeSphereDropScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.Spheres) != 3 {
		t.Fatalf("expected 3 spheres at setup, got %d", len(runner.State.Spheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected three-sphere scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	for index, sphere := range runner.State.Spheres {
		if index >= len(runner.State.BounceDetectedSet) || !runner.State.BounceDetectedSet[index] {
			t.Fatalf("expected sphere %d to bounce before settling", index)
		}
		if !sphere.Grounded {
			t.Fatalf("expected sphere %d to be grounded at the end", index)
		}
	}
}

func TestSphereDropOnThirtyDegreeSlopeSlidesWithoutBounce(t *testing.T) {
	definition := scenario.NewSphereDropOnThirtyDegreeSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected slope scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.EverTouchedGround {
		t.Fatalf("expected sphere to touch the slope")
	}

	if runner.State.BounceDetected {
		t.Fatalf("expected slope scene to have no bounce")
	}

	if !runner.State.Sphere.Grounded {
		t.Fatalf("expected sphere to remain grounded on the slope")
	}
}

func TestSphereBounceOnThirtyDegreeSlopeSlidesWithBounce(t *testing.T) {
	definition := scenario.NewSphereBounceOnThirtyDegreeSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected bounce slope scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.EverTouchedGround {
		t.Fatalf("expected sphere to touch the slope")
	}

	if !runner.State.BounceDetected {
		t.Fatalf("expected slope scene to bounce")
	}

	if runner.State.PeakBounceHeight.Cmp(runner.State.Sphere.Radius) <= 0 {
		t.Fatalf("expected bounce peak above radius height, got %v", runner.State.PeakBounceHeight)
	}
}

func TestThreeSphereDifferentFloorBounceScenarioShowsDifferentBounceHeights(t *testing.T) {
	definition := scenario.NewThreeSphereDifferentFloorBounceScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected three-floor bounce scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if len(runner.State.PeakBounceHeights) != 3 {
		t.Fatalf("expected 3 bounce heights, got %d", len(runner.State.PeakBounceHeights))
	}

	if runner.State.PeakBounceHeights[0].Cmp(runner.State.PeakBounceHeights[1]) >= 0 ||
		runner.State.PeakBounceHeights[1].Cmp(runner.State.PeakBounceHeights[2]) >= 0 {
		t.Fatalf("expected increasing bounce heights, got %v", runner.State.PeakBounceHeights)
	}
}

func TestThreeBoxDifferentFloorBounceScenarioShowsDifferentBounceHeights(t *testing.T) {
	definition := scenario.NewThreeBoxDifferentFloorBounceScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected three-box bounce scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if len(runner.State.BoxPeakBounceHeights) != 3 {
		t.Fatalf("expected 3 box bounce heights, got %d", len(runner.State.BoxPeakBounceHeights))
	}

	if runner.State.BoxPeakBounceHeights[0].Cmp(runner.State.BoxPeakBounceHeights[1]) >= 0 ||
		runner.State.BoxPeakBounceHeights[1].Cmp(runner.State.BoxPeakBounceHeights[2]) >= 0 {
		t.Fatalf("expected increasing box bounce heights, got %v", runner.State.BoxPeakBounceHeights)
	}
}

func TestThreeBoxSameSlopeAngleComparisonScenarioUsesDifferentStartingAngles(t *testing.T) {
	definition := scenario.NewThreeBoxSameSlopeAngleComparisonScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.BoxInitialRotations) != 3 {
		t.Fatalf("expected 3 initial box rotations, got %d", len(runner.State.BoxInitialRotations))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected three-box angle scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	for index := range runner.State.Boxes {
		if !runner.State.BoxRotationChangedSet[index] {
			t.Fatalf("expected box %d to rotate after contact", index)
		}
	}

	reboundCount := 0
	for _, bounced := range runner.State.BoxBounceDetectedSet {
		if bounced {
			reboundCount++
		}
	}
	if reboundCount == 0 {
		t.Fatalf("expected at least one box to rebound")
	}
}

func TestNineBoxFlatAngleComparisonScenarioUsesDifferentStartingAngles(t *testing.T) {
	definition := scenario.NewNineBoxFlatAngleComparisonScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.BoxInitialRotations) != 9 {
		t.Fatalf("expected 9 initial box rotations, got %d", len(runner.State.BoxInitialRotations))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected nine-box flat angle scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	for index := range runner.State.Boxes {
		if !runner.State.BoxRotationChangedSet[index] {
			t.Fatalf("expected box %d to rotate after ground contact", index)
		}
		if !runner.State.Boxes[index].Grounded {
			t.Fatalf("expected box %d to be grounded at the end", index)
		}
	}
}

func TestRigidBox3DFlatBounceScenarioRotatesAndBounces(t *testing.T) {
	definition := scenario.NewRigidBox3DFlatBounceScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected rigid box 3D scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.RigidBoxBounceDetected {
		t.Fatalf("expected rigid box to bounce")
	}
	if !runner.State.RigidBoxRotationChanged {
		t.Fatalf("expected rigid box orientation to change")
	}
}

func TestThreeBoxSameSlopeBounceScenarioShowsDifferentBounceByBox(t *testing.T) {
	definition := scenario.NewThreeBoxSameSlopeBounceScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected three-box slope scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if len(runner.State.BoxBounceDetectedSet) != 3 || len(runner.State.BoxPeakBounceHeights) != 3 {
		t.Fatalf("expected 3 box bounce states, got bounce=%d peaks=%d", len(runner.State.BoxBounceDetectedSet), len(runner.State.BoxPeakBounceHeights))
	}

	if runner.State.BoxBounceDetectedSet[0] {
		t.Fatalf("expected low-bounce box to stay non-bouncy")
	}

	if !runner.State.BoxBounceDetectedSet[1] || !runner.State.BoxBounceDetectedSet[2] {
		t.Fatalf("expected medium and high bounce boxes to rebound")
	}

	if runner.State.BoxPeakBounceHeights[1].Cmp(runner.State.BoxPeakBounceHeights[2]) >= 0 {
		t.Fatalf("expected high-bounce box to rebound higher, got %v", runner.State.BoxPeakBounceHeights)
	}
}

func TestScenarioRunnerResetRestoresInitialState(t *testing.T) {
	definition := scenario.NewSphereDropOnFlatGroundScenario()
	runner := scenario.NewScenarioRunner(definition)
	initial := definition.Setup()

	for step := 0; step < 12; step++ {
		runner.Step()
	}

	runner.Reset()

	if runner.Tick != 0 {
		t.Fatalf("expected tick=0 after reset, got %d", runner.Tick)
	}

	if runner.Finished {
		t.Fatalf("expected runner to be unfinished after reset")
	}

	if runner.LastResult.Status != scenario.Running {
		t.Fatalf("expected reset status to be running, got %v", runner.LastResult.Status)
	}

	if !reflect.DeepEqual(runner.State, initial) {
		t.Fatalf("reset state mismatch:\n got=%#v\nwant=%#v", runner.State, initial)
	}
}

func TestScenarioRunnerIsDeterministic(t *testing.T) {
	definition := scenario.NewSphereDropOnFlatGroundScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}

		if first.Tick != second.Tick {
			t.Fatalf("tick mismatch: %d != %d", first.Tick, second.Tick)
		}

		if first.Finished != second.Finished {
			t.Fatalf("finished flag mismatch: %v != %v", first.Finished, second.Finished)
		}

		if !reflect.DeepEqual(first.LastResult, second.LastResult) {
			t.Fatalf("result mismatch:\nfirst=%#v\nsecond=%#v", first.LastResult, second.LastResult)
		}

		firstHash := scenario.HashSceneState(first.State)
		secondHash := scenario.HashSceneState(second.State)
		if firstHash != secondHash {
			t.Fatalf("hash mismatch: %x != %x", firstHash, secondHash)
		}
	}
}
