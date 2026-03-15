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

func TestThreeSphereDropScenarioFallsAndSettles(t *testing.T) {
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
		if !sphere.Grounded {
			t.Fatalf("expected sphere %d to be grounded at the end", index)
		}
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
