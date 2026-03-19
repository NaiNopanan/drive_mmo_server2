package scenario_test

import (
	"reflect"
	"testing"

	"server2/internal/fixed"
	"server2/internal/physics"
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

func TestNineRigidBox3DFlatAngleComparisonScenarioUsesDifferentStartingAngles(t *testing.T) {
	definition := scenario.NewNineRigidBox3DFlatAngleComparisonScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidBoxes) != 9 {
		t.Fatalf("expected 9 rigid boxes at setup, got %d", len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected nine rigid-box flat angle scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	for index := range runner.State.RigidBoxes {
		if !runner.State.RigidBoxRotationChangedSet[index] {
			t.Fatalf("expected rigid box %d to rotate after ground contact", index)
		}
		if !runner.State.RigidBoxes[index].Grounded {
			t.Fatalf("expected rigid box %d to be grounded at the end", index)
		}
	}
}

func TestNineRigidSphere3DSmallSlopeScenarioRollsAndRotates(t *testing.T) {
	definition := scenario.NewNineRigidSphere3DSmallSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 9 {
		t.Fatalf("expected 9 rigid spheres at setup, got %d", len(runner.State.RigidSpheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected nine rigid-sphere slope scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	for index := range runner.State.RigidSpheres {
		if !runner.State.RigidSphereRotationChangedSet[index] {
			t.Fatalf("expected rigid sphere %d to rotate on the slope", index)
		}
		if index >= len(runner.State.RigidSphereTouchedGroundSet) || !runner.State.RigidSphereTouchedGroundSet[index] {
			t.Fatalf("expected rigid sphere %d to touch its slope", index)
		}
	}
}

func TestTwoSphereCollisionScenarioCollidesAndReversesDirection(t *testing.T) {
	definition := scenario.NewTwoSphereCollisionScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.Spheres) != 2 {
		t.Fatalf("expected 2 spheres at setup, got %d", len(runner.State.Spheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected two-sphere collision scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereSphereCollisionDetected {
		t.Fatalf("expected two spheres to collide")
	}
}

func TestTwoSphereDifferentMassCollisionScenarioShowsDifferentOutcome(t *testing.T) {
	definition := scenario.NewTwoSphereDifferentMassCollisionScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.Spheres) != 2 {
		t.Fatalf("expected 2 spheres at setup, got %d", len(runner.State.Spheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected different-mass sphere collision scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereSphereCollisionDetected {
		t.Fatalf("expected different-mass spheres to collide")
	}

	if runner.State.Spheres[1].Motion.Velocity.X.Cmp(runner.State.Spheres[0].Motion.Velocity.X) <= 0 {
		t.Fatalf("expected lighter sphere to move faster after impact, got heavy=%v light=%v", runner.State.Spheres[0].Motion.Velocity, runner.State.Spheres[1].Motion.Velocity)
	}
}

func TestSphereBoxCollisionScenarioTransfersMomentum(t *testing.T) {
	definition := scenario.NewSphereBoxCollisionScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected sphere-box collision scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected sphere and box to collide")
	}
}

func TestTwoSphereBoxOpposingMassScenarioMovesTowardHeavierSide(t *testing.T) {
	definition := scenario.NewTwoSphereBoxOpposingMassScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.Spheres) != 2 {
		t.Fatalf("expected 2 spheres at setup, got %d", len(runner.State.Spheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected two-sphere box opposing-mass scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected spheres and box to collide")
	}

	if runner.State.Box.Motion.Position.X.Cmp(0) <= 0 {
		t.Fatalf("expected box to move toward positive X, got position=%v", runner.State.Box.Motion.Position)
	}
}

func TestBoxFrictionFlatSlideScenarioSlowsToRest(t *testing.T) {
	definition := scenario.NewBoxFrictionFlatSlideScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected box friction scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.Box.Grounded {
		t.Fatalf("expected box to be grounded at the end")
	}
}

func TestSphereBoxFrictionCollisionScenarioSlidesAndSlows(t *testing.T) {
	definition := scenario.NewSphereBoxFrictionCollisionScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected sphere-box friction scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected sphere to collide with friction box")
	}
	if !runner.State.Box.Grounded {
		t.Fatalf("expected friction box to stay grounded")
	}
	if runner.State.Box.Motion.Position.X.Cmp(0) <= 0 {
		t.Fatalf("expected friction box to move forward, got position=%v", runner.State.Box.Motion.Position)
	}
}

func TestSphereBoxFrictionBounceCollisionScenarioBouncesAndSlides(t *testing.T) {
	definition := scenario.NewSphereBoxFrictionBounceCollisionScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected sphere-box friction bounce scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected bouncing sphere to collide with bouncing box")
	}
	if runner.State.Box.Motion.Position.X.Cmp(0) <= 0 {
		t.Fatalf("expected bouncing box to move forward, got position=%v", runner.State.Box.Motion.Position)
	}
	if runner.State.RigidSphere.Motion.Velocity.X.Cmp(0) >= 0 {
		t.Fatalf("expected rigid sphere to rebound backward, got velocity=%v", runner.State.RigidSphere.Motion.Velocity)
	}
	if runner.State.RigidSphere.AngularVelocity.LengthSquared().Cmp(0) <= 0 {
		t.Fatalf("expected rigid sphere to gain spin, got angular velocity=%v", runner.State.RigidSphere.AngularVelocity)
	}
}

func TestHundredRigidSpheresInBoxScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresInBoxScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 100 {
		t.Fatalf("expected 100 rigid spheres at setup, got %d", len(runner.State.RigidSpheres))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected hundred-rigid-spheres-in-box scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.RigidSphereSphereCollisionDetected {
		t.Fatalf("expected rigid spheres to collide with each other inside the box")
	}
}

func TestHundredBoxesInBoxScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredBoxesInBoxScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.Boxes) != 100 {
		t.Fatalf("expected 100 boxes at setup, got %d", len(runner.State.Boxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected hundred-boxes-in-box scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.BoxBoxCollisionDetected {
		t.Fatalf("expected boxes to collide with each other inside the box")
	}
}

func TestHundredBoxesInBoxAngleScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredBoxesInBoxAngleScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidBoxes) != 100 {
		t.Fatalf("expected 100 rigid boxes at setup, got %d", len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected hundred-rigid-boxes-in-box-angle scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.RigidBoxBoxCollisionDetected {
		t.Fatalf("expected angled rigid boxes to collide with each other inside the box")
	}

	rotatedCount := 0
	for _, changed := range runner.State.RigidBoxRotationChangedSet {
		if changed {
			rotatedCount++
		}
	}
	if rotatedCount == 0 {
		t.Fatalf("expected at least one rigid box to change orientation inside the box")
	}
}

func TestFiftyRigidSpheresAndFiftyRigidBoxesInBoxScenarioStaysContained(t *testing.T) {
	definition := scenario.NewFiftyRigidSpheresAndFiftyRigidBoxesInBoxScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 50 || len(runner.State.RigidBoxes) != 50 {
		t.Fatalf("expected 50 rigid spheres and 50 rigid boxes at setup, got spheres=%d boxes=%d", len(runner.State.RigidSpheres), len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected mixed rigid-body box scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected rigid spheres and rigid boxes to collide")
	}
	if !runner.State.RigidSphereSphereCollisionDetected {
		t.Fatalf("expected rigid spheres to collide with each other")
	}
	if !runner.State.RigidBoxBoxCollisionDetected {
		t.Fatalf("expected rigid boxes to collide with each other")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 100 || len(runner.State.RigidBoxes) != 100 {
		t.Fatalf("expected 100 rigid spheres and 100 rigid boxes at setup, got spheres=%d boxes=%d", len(runner.State.RigidSpheres), len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected dense mixed rigid-body box scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}

	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected rigid spheres and rigid boxes to collide")
	}
	if !runner.State.RigidSphereSphereCollisionDetected {
		t.Fatalf("expected rigid spheres to collide with each other")
	}
	if !runner.State.RigidBoxBoxCollisionDetected {
		t.Fatalf("expected rigid boxes to collide with each other")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenarioReducesCandidates(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 100 || len(runner.State.RigidBoxes) != 100 {
		t.Fatalf("expected 100 rigid spheres and 100 rigid boxes at setup, got spheres=%d boxes=%d", len(runner.State.RigidSpheres), len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected optimized dense mixed rigid-body box scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.SphereSphereCandidateCount >= 4950 {
		t.Fatalf("expected broadphase to reduce sphere-sphere candidates, got %d", runner.State.SphereSphereCandidateCount)
	}
	if runner.State.BoxBoxCandidateCount >= 4950 {
		t.Fatalf("expected broadphase to reduce box-box candidates, got %d", runner.State.BoxBoxCandidateCount)
	}
	if runner.State.SphereBoxCandidateCount >= 10000 {
		t.Fatalf("expected broadphase to reduce sphere-box candidates, got %d", runner.State.SphereBoxCandidateCount)
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("optimized scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if first.Tick != second.Tick {
			t.Fatalf("optimized scene tick mismatch: %d != %d", first.Tick, second.Tick)
		}
		if first.Finished != second.Finished {
			t.Fatalf("optimized scene finished flag mismatch: %v != %v", first.Finished, second.Finished)
		}
		if !reflect.DeepEqual(first.LastResult, second.LastResult) {
			t.Fatalf("optimized scene result mismatch:\nfirst=%#v\nsecond=%#v", first.LastResult, second.LastResult)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("optimized scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedHighSpeedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedHighSpeedScenario()
	runner := scenario.NewScenarioRunner(definition)

	if len(runner.State.RigidSpheres) != 100 || len(runner.State.RigidBoxes) != 100 {
		t.Fatalf("expected 100 rigid spheres and 100 rigid boxes at setup, got spheres=%d boxes=%d", len(runner.State.RigidSpheres), len(runner.State.RigidBoxes))
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected optimized high-speed dense scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.SphereSphereCandidateCount >= 4950 || runner.State.BoxBoxCandidateCount >= 4950 || runner.State.SphereBoxCandidateCount >= 10000 {
		t.Fatalf("expected optimized high-speed scene to reduce candidate counts, got ss=%d bb=%d sb=%d", runner.State.SphereSphereCandidateCount, runner.State.BoxBoxCandidateCount, runner.State.SphereBoxCandidateCount)
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxAllCCDScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAllCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for _, sphere := range runner.State.RigidSpheres {
		if !sphere.UseCCD || sphere.CCDMode != physics.CCDModeSweepTriangleMesh {
			t.Fatalf("expected every rigid sphere to start with mesh CCD enabled")
		}
	}
	for _, box := range runner.State.RigidBoxes {
		if !box.UseCCD || box.CCDMode != physics.CCDModeSweepRotatingOrientedBoxTriangleMesh {
			t.Fatalf("expected every rigid box to start with rotating OBB mesh CCD enabled")
		}
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected all-CCD mixed rigid container scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.SphereBoxCollisionDetected {
		t.Fatalf("expected spheres and boxes to collide in all-CCD scene")
	}
	if !runner.State.RigidSphereSphereCollisionDetected {
		t.Fatalf("expected rigid spheres to collide in all-CCD scene")
	}
	if !runner.State.RigidBoxBoxCollisionDetected {
		t.Fatalf("expected rigid boxes to collide in all-CCD scene")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxHybridCCDOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxHybridCCDOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected hybrid CCD optimized scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverActivatedCCDSphere && !runner.State.EverActivatedCCDBox {
		t.Fatalf("expected hybrid scene to activate CCD for some objects")
	}
	if !runner.State.EverSleptSphere && !runner.State.EverSleptBox {
		t.Fatalf("expected hybrid scene to put some objects to sleep")
	}
	if !runner.State.SphereBoxCollisionDetected || !runner.State.RigidSphereSphereCollisionDetected || !runner.State.RigidBoxBoxCollisionDetected {
		t.Fatalf("expected hybrid scene to exercise all collision paths")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected adaptive CCD optimized scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverActivatedAngularRiskCCDBox {
		t.Fatalf("expected adaptive scene to activate box CCD from angular sweep risk")
	}
	if !runner.State.EverSkippedSleepingPairs {
		t.Fatalf("expected adaptive scene to skip sleeping-sleeping pairs")
	}
	if runner.State.SleepingSphereSphereSkipCount == 0 && runner.State.SleepingBoxBoxSkipCount == 0 && runner.State.SleepingSphereBoxSkipCount == 0 {
		t.Fatalf("expected adaptive scene to record sleeping pair skips")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected adaptive precheck optimized scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.BoxCCDPrecheckCount == 0 {
		t.Fatalf("expected adaptive precheck scene to run box CCD prechecks")
	}
	if !runner.State.EverRejectedBoxCCDPrecheck {
		t.Fatalf("expected adaptive precheck scene to reject some box CCD calls before full mesh sweep")
	}
	if !runner.State.EverExecutedBoxCCDMeshSweep {
		t.Fatalf("expected adaptive precheck scene to still execute some full box mesh sweeps")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckHysteresisOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckHysteresisOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected adaptive hysteresis optimized scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverRanSphereCCDPrecheck || !runner.State.EverRanBoxCCDPrecheck {
		t.Fatalf("expected adaptive hysteresis scene to run both sphere and box CCD prechecks")
	}
	if !runner.State.EverRejectedSphereCCDPrecheck || !runner.State.EverRejectedBoxCCDPrecheck {
		t.Fatalf("expected adaptive hysteresis scene to reject some sphere and box CCD calls before full mesh sweep")
	}
	if !runner.State.EverExecutedSphereCCDMeshSweep || !runner.State.EverExecutedBoxCCDMeshSweep {
		t.Fatalf("expected adaptive hysteresis scene to still execute some sphere and box full mesh sweeps")
	}
	if !runner.State.EverHeldSphereCCDHysteresis || !runner.State.EverHeldBoxCCDHysteresis {
		t.Fatalf("expected adaptive hysteresis scene to hold CCD active within the hysteresis band")
	}
	if !runner.State.EverHeldSphereSleepHysteresis || !runner.State.EverHeldBoxSleepHysteresis {
		t.Fatalf("expected adaptive hysteresis scene to hold sleep counters within the hysteresis band")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxPersistentPairsIslandsWarmStartOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxPersistentPairsIslandsWarmStartOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected persistent-pairs optimized scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverReusedPersistentPairs {
		t.Fatalf("expected persistent-pairs scene to reuse cached pairs")
	}
	if !runner.State.EverWarmStartedPairs {
		t.Fatalf("expected persistent-pairs scene to warm-start repeated contacts")
	}
	if !runner.State.EverBuiltSleepingIsland {
		t.Fatalf("expected persistent-pairs scene to build contact islands")
	}
	if !runner.State.EverSleptIsland {
		t.Fatalf("expected persistent-pairs scene to put an island to sleep")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxSolverManifoldWarmStartOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxSolverManifoldWarmStartOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected solver-manifold warm-start scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverReusedPersistentPairs {
		t.Fatalf("expected solver-manifold scene to reuse cached pairs")
	}
	if !runner.State.EverWarmStartedPairs {
		t.Fatalf("expected solver-manifold scene to warm-start cached manifold impulses")
	}
	hasImpulse := false
	for _, entry := range runner.State.PersistentContactCache {
		if entry.NormalImpulse.Cmp(fixed.Zero) > 0 || entry.TangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
			hasImpulse = true
			break
		}
	}
	if !hasImpulse {
		t.Fatalf("expected solver-manifold scene to retain non-zero cached impulses")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverWarmStartOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverWarmStartOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected profiled solver-warm-start scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.PhaseIntegrationNanos == 0 || runner.State.PhaseCCDNanos == 0 || runner.State.PhaseBroadphaseNanos == 0 || runner.State.PhaseSolverNanos == 0 || runner.State.PhaseSleepingNanos == 0 {
		t.Fatalf("expected profiled scene to record all phase timings")
	}
	if !runner.State.EverReusedPersistentPairs || !runner.State.EverWarmStartedPairs || !runner.State.EverBuiltSleepingIsland {
		t.Fatalf("expected profiled scene to exercise persistent pairs, warm start, and sleeping islands")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverFrontierOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverFrontierOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected profiled solver-frontier scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.PhaseIntegrationNanos == 0 || runner.State.PhaseCCDNanos == 0 || runner.State.PhaseBroadphaseNanos == 0 || runner.State.PhaseSolverNanos == 0 || runner.State.PhaseSleepingNanos == 0 {
		t.Fatalf("expected profiled solver-frontier scene to record all phase timings")
	}
	if !runner.State.EverReducedSolverPass2 || runner.State.SolverPass2PairCount >= runner.State.SolverPass1PairCount {
		t.Fatalf("expected profiled solver-frontier scene to reduce pass-two solver work")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveBudgetOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveBudgetOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected adaptive-budget scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.CCDBudgetSkipCount == 0 || !runner.State.EverAppliedCCDBudget {
		t.Fatalf("expected adaptive-budget scene to apply a CCD budget")
	}
	if !runner.State.EverAppliedSolverBudget || !runner.State.EverReducedSolverPass2 {
		t.Fatalf("expected adaptive-budget scene to reduce solver frontier work with a budget")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxSmartAdaptiveBudgetOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxSmartAdaptiveBudgetOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected smart-adaptive-budget scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverAppliedCCDBudget || !runner.State.EverAppliedSolverBudget {
		t.Fatalf("expected smart-adaptive-budget scene to apply CCD and solver budgets")
	}
	if !runner.State.EverReducedSolverPass2 || runner.State.SolverPass2PairCount >= runner.State.SolverPass1PairCount {
		t.Fatalf("expected smart-adaptive-budget scene to reduce pass-two solver work")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxTimingAwareAdaptiveBudgetOptimizedScenarioStaysContained(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxTimingAwareAdaptiveBudgetOptimizedScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected timing-aware-adaptive-budget scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverAppliedCCDBudget || !runner.State.EverAppliedSolverBudget {
		t.Fatalf("expected timing-aware-adaptive-budget scene to apply CCD and solver budgets")
	}
	if !runner.State.EverReducedSolverPass2 || runner.State.SolverPass2PairCount >= runner.State.SolverPass1PairCount {
		t.Fatalf("expected timing-aware-adaptive-budget scene to reduce pass-two solver work")
	}
}

func TestHundredRigidSpheresAndHundredRigidBoxesInBoxTimingAwareAdaptiveBudgetOptimizedScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxTimingAwareAdaptiveBudgetOptimizedScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	normalizeProfiling := func(state scenario.SceneState) scenario.SceneState {
		state.PhaseIntegrationNanos = 0
		state.PhaseCCDNanos = 0
		state.PhaseBroadphaseNanos = 0
		state.PhaseSolverNanos = 0
		state.PhaseSleepingNanos = 0
		return state
	}

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(normalizeProfiling(first.State), normalizeProfiling(second.State)) {
			t.Fatalf("timing-aware adaptive-budget scene state mismatch after normalizing profiling-only fields")
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("timing-aware adaptive-budget scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestRigidSphereHighSpeedThinWallProjectileScenarioDetectsTunneling(t *testing.T) {
	definition := scenario.NewRigidSphereHighSpeedThinWallProjectileScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Failed {
		t.Fatalf("expected high-speed thin-wall projectile scenario to fail when tunneling is detected, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.LastResult.Message != "Projectile tunneled through the thin wall." {
		t.Fatalf("expected tunneling message, got %q", runner.LastResult.Message)
	}
	if runner.State.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		return
	}
	t.Fatalf("expected projectile to end beyond the thin wall exit, got x=%v", runner.State.RigidSphere.Motion.Position.X)
}

func TestRigidSphereHighSpeedThinWallProjectileCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidSphereHighSpeedThinWallProjectileCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected CCD thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected CCD projectile to touch the thin wall")
	}
	if runner.State.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected CCD projectile to stay before the thin wall exit, got x=%v", runner.State.RigidSphere.Motion.Position.X)
	}
}

func TestRigidSphereHighSpeedThinWallProjectileMeshCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidSphereHighSpeedThinWallProjectileMeshCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected mesh CCD thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected mesh CCD projectile to touch the thin wall")
	}
	if runner.State.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected mesh CCD projectile to stay before the thin wall exit, got x=%v", runner.State.RigidSphere.Motion.Position.X)
	}
}

func TestRigidBoxHighSpeedThinWallProjectileCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidBoxHighSpeedThinWallProjectileCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected CCD box thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected CCD box projectile to touch the thin wall")
	}
	if runner.State.RigidBox.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected CCD box projectile to stay before the thin wall exit, got x=%v", runner.State.RigidBox.Motion.Position.X)
	}
}

func TestRigidBoxRotatingHighSpeedThinWallProjectileCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidBoxRotatingHighSpeedThinWallProjectileCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected rotating CCD box thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected rotating CCD box projectile to touch the thin wall")
	}
	if runner.State.RigidBox.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected rotating CCD box projectile to stay before the thin wall exit, got x=%v", runner.State.RigidBox.Motion.Position.X)
	}
	if runner.State.RigidBox.AngularVelocity.LengthSquared() == fixed.Zero {
		t.Fatalf("expected rotating CCD box projectile to retain angular velocity")
	}
}

func TestRigidBoxRotatingHighSpeedThinWallProjectileOBBCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidBoxRotatingHighSpeedThinWallProjectileOBBCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected rotating OBB CCD box thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected rotating OBB CCD box projectile to touch the thin wall")
	}
	if runner.State.RigidBox.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected rotating OBB CCD box projectile to stay before the thin wall exit, got x=%v", runner.State.RigidBox.Motion.Position.X)
	}
	if runner.State.RigidBox.AngularVelocity.LengthSquared() == fixed.Zero {
		t.Fatalf("expected rotating OBB CCD box projectile to retain angular velocity")
	}
}

func TestRigidBoxRotatingHighSpeedThinWallProjectileOBBMeshCCDScenarioDoesNotTunnel(t *testing.T) {
	definition := scenario.NewRigidBoxRotatingHighSpeedThinWallProjectileOBBMeshCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected rotating OBB mesh CCD box thin-wall projectile scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.EverTouchedGround {
		t.Fatalf("expected rotating OBB mesh CCD box projectile to touch the thin wall")
	}
	if runner.State.RigidBox.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected rotating OBB mesh CCD box projectile to stay before the thin wall exit, got x=%v", runner.State.RigidBox.Motion.Position.X)
	}
	if runner.State.RigidBox.AngularVelocity.LengthSquared() == fixed.Zero {
		t.Fatalf("expected rotating OBB mesh CCD box projectile to retain angular velocity")
	}
}

func TestRigidSphereAndBoxThinWallProjectileObjectCCDScenarioUsesObjectCCDSettings(t *testing.T) {
	definition := scenario.NewRigidSphereAndBoxThinWallProjectileObjectCCDScenario()
	runner := scenario.NewScenarioRunner(definition)

	if !runner.State.RigidSphere.UseCCD || runner.State.RigidSphere.CCDMode != physics.CCDModeSweepTriangleMesh {
		t.Fatalf("expected rigid sphere to start with mesh CCD, got use=%v mode=%v", runner.State.RigidSphere.UseCCD, runner.State.RigidSphere.CCDMode)
	}
	if !runner.State.RigidBox.UseCCD || runner.State.RigidBox.CCDMode != physics.CCDModeSweepRotatingOrientedBoxTriangleMesh {
		t.Fatalf("expected rigid box to start with rotating OBB mesh CCD, got use=%v mode=%v", runner.State.RigidBox.UseCCD, runner.State.RigidBox.CCDMode)
	}

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected mixed object CCD thin-wall scenario to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.RigidSphereCCDHitDetected {
		t.Fatalf("expected rigid sphere to register a CCD wall hit")
	}
	if !runner.State.RigidBoxCCDHitDetected {
		t.Fatalf("expected rigid box to register a CCD wall hit")
	}
	if runner.State.RigidSphere.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected rigid sphere to stay before the thin wall exit, got x=%v", runner.State.RigidSphere.Motion.Position.X)
	}
	if runner.State.RigidBox.Motion.Position.X.Cmp(fixed.FromInt(1)) > 0 {
		t.Fatalf("expected rigid box to stay before the thin wall exit, got x=%v", runner.State.RigidBox.Motion.Position.X)
	}
	if runner.State.RigidBox.Motion.Velocity.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected rigid box to bounce back after wall impact, got vx=%v", runner.State.RigidBox.Motion.Velocity.X)
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

func TestVehicleArcadeDropToGroundScenarioSettles(t *testing.T) {
	definition := scenario.NewVehicleArcadeDropToGroundScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle drop scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleGroundedProbeCount < 2 {
		t.Fatalf("expected at least two grounded probes after landing, got %d", runner.State.VehicleGroundedProbeCount)
	}
	if !runner.State.VehicleSettled {
		t.Fatalf("expected vehicle drop scene to settle")
	}
}

func TestVehicleArcadeDropToGroundScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeDropToGroundScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeIdleSuspensionScenarioSettles(t *testing.T) {
	definition := scenario.NewVehicleArcadeIdleSuspensionScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle idle suspension scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleGroundedProbeCount < 3 {
		t.Fatalf("expected at least three grounded probes in idle suspension scene, got %d", runner.State.VehicleGroundedProbeCount)
	}
}

func TestVehicleArcadeIdleSuspensionScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeIdleSuspensionScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle idle scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle idle scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeStraightDriveScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeStraightDriveScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle straight-drive scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(6)) < 0 {
		t.Fatalf("expected vehicle to move forward in straight-drive scene, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicleArcadeStraightDriveScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeStraightDriveScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle straight-drive scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle straight-drive scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeTurnScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeTurnScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle turn scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.X.Cmp(fixed.One) < 0 {
		t.Fatalf("expected vehicle to arc sideways in turn scene, got x=%v", runner.State.VehicleChassis.Motion.Position.X)
	}
}

func TestVehicleArcadeTurnScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeTurnScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle turn scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle turn scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeWallImpactScenarioHitsWall(t *testing.T) {
	definition := scenario.NewVehicleArcadeWallImpactScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle wall-impact scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromFraction(17, 2)) > 0 {
		t.Fatalf("expected vehicle to stay on the impact side of the wall, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if !runner.State.LastContact.Hit || runner.State.LastContact.Normal.Z.Cmp(fixed.FromFraction(-9, 10)) > 0 {
		t.Fatalf("expected vehicle wall-impact scene to record a forward wall contact, got normal=%v", runner.State.LastContact.Normal)
	}
}

func TestVehicleArcadeWallImpactScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeWallImpactScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle wall-impact scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle wall-impact scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeUphillSlopeScenarioClimbs(t *testing.T) {
	definition := scenario.NewVehicleArcadeUphillSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle uphill slope scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
		t.Fatalf("expected vehicle to gain height on slope, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeUphillSlopeScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeUphillSlopeScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle uphill slope scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle uphill slope scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeSteerOnSlopeScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeSteerOnSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle steer-on-slope scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.X.Cmp(fixed.FromFraction(3, 2)) < 0 {
		t.Fatalf("expected vehicle to arc sideways on slope, got x=%v", runner.State.VehicleChassis.Motion.Position.X)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
		t.Fatalf("expected vehicle to gain height while steering on slope, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeSteerOnSlopeScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeSteerOnSlopeScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle steer-on-slope scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle steer-on-slope scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeDownhillSlopeScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeDownhillSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle downhill slope scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) > 0 {
		t.Fatalf("expected vehicle to descend downhill along z, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(37, 20)) > 0 {
		t.Fatalf("expected vehicle to lose height on downhill slope, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeDownhillSlopeScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeDownhillSlopeScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle downhill slope scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle downhill slope scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeSteerDownhillSlopeScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeSteerDownhillSlopeScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle steer-downhill scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.X.Cmp(fixed.One.Neg()) > 0 && runner.State.VehicleChassis.Motion.Position.X.Cmp(fixed.One) < 0 {
		t.Fatalf("expected vehicle to arc sideways downhill, got x=%v", runner.State.VehicleChassis.Motion.Position.X)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) > 0 {
		t.Fatalf("expected vehicle to descend downhill along z, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.AngularVelocity.Y.Cmp(fixed.FromFraction(1, 20)) <= 0 {
		t.Fatalf("expected vehicle to build downhill steering yaw, got angularY=%v", runner.State.VehicleChassis.AngularVelocity.Y)
	}
}

func TestVehicleArcadeSteerDownhillSlopeScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeSteerDownhillSlopeScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle steer-downhill scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle steer-downhill scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeSlopeCrestTransitionScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeSlopeCrestTransitionScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle slope-crest scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(10)) < 0 {
		t.Fatalf("expected vehicle to reach the flat crest, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(19, 10)) < 0 {
		t.Fatalf("expected vehicle to remain high on the crest, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeSlopeCrestTransitionScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeSlopeCrestTransitionScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle slope-crest scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle slope-crest scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeWallRecoverScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeWallRecoverScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle wall-recover scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.VehicleEverHitWall {
		t.Fatalf("expected vehicle wall-recover scene to hit the wall before recovering")
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(7)) > 0 {
		t.Fatalf("expected vehicle to back away from the wall, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicleArcadeWallRecoverScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeWallRecoverScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle wall-recover scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle wall-recover scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleUprightDot.Cmp(fixed.FromFraction(39, 40)) < 0 {
		t.Fatalf("expected vehicle to recover upright after curb bump, got uprightDot=%v", runner.State.VehicleUprightDot)
	}
}

func TestVehicleArcadeCurbBumpScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeUphillSlopeEdgeProbesScenarioClimbs(t *testing.T) {
	definition := scenario.NewVehicleArcadeUphillSlopeEdgeProbesScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle uphill slope edge-probes scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(13, 10)) < 0 {
		t.Fatalf("expected vehicle with edge probes to gain height on slope, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeUphillSlopeEdgeProbesScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeUphillSlopeEdgeProbesScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle uphill slope edge-probes scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle uphill slope edge-probes scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpEdgeProbesScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpEdgeProbesScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump edge-probes scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle with edge probes to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicleArcadeCurbBumpEdgeProbesScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpEdgeProbesScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump edge-probes scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump edge-probes scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpWheelRadiusScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpWheelRadiusScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump wheel-radius scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle with wheel-radius offset to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(17, 20)) < 0 {
		t.Fatalf("expected vehicle with wheel-radius offset to ride higher over the bump, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeCurbBumpWheelRadiusScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpWheelRadiusScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump wheel-radius scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump wheel-radius scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpSphereWheelProbesScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpSphereWheelProbesScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump sphere-wheel scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle with sphere wheel probes to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 20)) < 0 {
		t.Fatalf("expected vehicle with sphere wheel probes to ride high enough over the bump, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeCurbBumpSphereWheelProbesScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpSphereWheelProbesScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump sphere-wheel scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump sphere-wheel scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpWheelColliderScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpWheelColliderScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump wheel-collider scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle with wheel collider to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(19, 40)) < 0 {
		t.Fatalf("expected vehicle with wheel collider to ride higher over the bump, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeCurbBumpWheelColliderScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpWheelColliderScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump wheel-collider scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump wheel-collider scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicleArcadeCurbBumpClampedWheelColliderScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpClampedWheelColliderScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle curb-bump clamped wheel-collider scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle with clamped wheel collider to clear the curb bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 20)) < 0 {
		t.Fatalf("expected vehicle with clamped wheel collider to ride high enough over the bump, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicleArcadeCurbBumpClampedWheelColliderScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicleArcadeCurbBumpClampedWheelColliderScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle curb-bump clamped wheel-collider scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle curb-bump clamped wheel-collider scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle2MMOGroundingBaselineScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle2MMOGroundingBaselineScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle2 MMO grounding baseline scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(6)) < 0 {
		t.Fatalf("expected vehicle2 baseline to move forward, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicle2MMOGroundingBaselineScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle2MMOGroundingBaselineScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle2 MMO grounding baseline scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle2 MMO grounding baseline scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle2MMOCurbSupportScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle2MMOCurbSupportScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle2 MMO curb support scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle2 curb support to clear bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicle2MMOCurbSupportScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle2MMOCurbSupportScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle2 MMO curb support scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle2 MMO curb support scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle2MMORoadSupportAndWallScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle2MMORoadSupportAndWallScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle2 MMO road-support-and-wall scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.VehicleEverHitWall {
		t.Fatalf("expected vehicle2 road-support-and-wall scene to hit the hard wall obstacle")
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) < 0 {
		t.Fatalf("expected vehicle2 road-support-and-wall scene to advance through the supported road section, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicle2MMORoadSupportAndWallScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle2MMORoadSupportAndWallScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle2 MMO road-support-and-wall scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle2 MMO road-support-and-wall scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle3RayCityGroundingBaselineScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle3RayCityGroundingBaselineScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle3 RayCity grounding baseline scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(8)) < 0 {
		t.Fatalf("expected vehicle3 baseline to move forward, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicle3RayCityGroundingBaselineScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle3RayCityGroundingBaselineScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle3 RayCity grounding baseline scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle3 RayCity grounding baseline scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle3RayCityCurbSupportScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle3RayCityCurbSupportScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle3 RayCity curb support scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(10)) < 0 {
		t.Fatalf("expected vehicle3 curb support to clear bump, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if runner.State.VehicleChassis.Motion.Position.Y.Cmp(fixed.FromFraction(9, 10)) < 0 {
		t.Fatalf("expected vehicle3 curb support to ride high enough over the bump, got y=%v", runner.State.VehicleChassis.Motion.Position.Y)
	}
}

func TestVehicle3RayCityCurbSupportScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle3RayCityCurbSupportScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle3 RayCity curb support scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle3 RayCity curb support scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle3RayCityArcadeTurnScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle3RayCityArcadeTurnScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle3 RayCity arcade turn scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if runner.State.VehicleChassis.Motion.Position.X.Cmp(fixed.FromFraction(3, 5)) < 0 {
		t.Fatalf("expected vehicle3 turn to build a sideways arc, got x=%v", runner.State.VehicleChassis.Motion.Position.X)
	}
	if runner.State.VehicleFrontGroundedProbeCount < 1 || runner.State.VehicleRearGroundedProbeCount < 1 {
		t.Fatalf("expected vehicle3 turn to keep at least one grounded wheel on each axle, got front=%d rear=%d", runner.State.VehicleFrontGroundedProbeCount, runner.State.VehicleRearGroundedProbeCount)
	}
}

func TestVehicle3RayCityArcadeTurnScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle3RayCityArcadeTurnScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle3 RayCity arcade turn scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle3 RayCity arcade turn scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle3RayCityRoadSupportAndWallScenarioAdvances(t *testing.T) {
	definition := scenario.NewVehicle3RayCityRoadSupportAndWallScenario()
	runner := scenario.NewScenarioRunner(definition)

	for !runner.Finished {
		runner.Step()
	}

	if runner.LastResult.Status != scenario.Passed {
		t.Fatalf("expected vehicle3 RayCity road-support-and-wall scene to pass, got status=%v message=%q", runner.LastResult.Status, runner.LastResult.Message)
	}
	if !runner.State.VehicleEverHitWall {
		t.Fatalf("expected vehicle3 road-support-and-wall scene to hit the hard wall obstacle")
	}
	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(2)) < 0 {
		t.Fatalf("expected vehicle3 road-support-and-wall scene to advance through the supported road section, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
}

func TestVehicle3RayCityRoadSupportAndWallScenarioIsDeterministic(t *testing.T) {
	definition := scenario.NewVehicle3RayCityRoadSupportAndWallScenario()
	first := scenario.NewScenarioRunner(definition)
	second := scenario.NewScenarioRunner(definition)

	for !first.Finished || !second.Finished {
		first.Step()
		second.Step()

		if !reflect.DeepEqual(first.State, second.State) {
			t.Fatalf("vehicle3 RayCity road-support-and-wall scene state mismatch:\nfirst=%#v\nsecond=%#v", first.State, second.State)
		}
		if scenario.HashSceneState(first.State) != scenario.HashSceneState(second.State) {
			t.Fatalf("vehicle3 RayCity road-support-and-wall scene hash mismatch: %016x != %016x", scenario.HashSceneState(first.State), scenario.HashSceneState(second.State))
		}
	}
}

func TestVehicle3ManualMapAutoDriveMaintainsGrounding(t *testing.T) {
	definition := scenario.NewVehicle3RayCityManualScenario()
	runner := scenario.NewScenarioRunner(definition)

	maxConsecutiveAirborne := 0
	currentConsecutiveAirborne := 0

	for step := 0; step < 900; step++ {
		scenario.ApplyVehicle3AutoDriveInput(&runner.State)
		runner.Step()

		if step < 30 {
			continue
		}

		if runner.State.VehicleGroundedProbeCount == 0 {
			currentConsecutiveAirborne++
			if currentConsecutiveAirborne > maxConsecutiveAirborne {
				maxConsecutiveAirborne = currentConsecutiveAirborne
			}
		} else {
			currentConsecutiveAirborne = 0
		}
	}

	if runner.State.VehicleChassis.Motion.Position.Z.Cmp(fixed.FromInt(20)) < 0 {
		t.Fatalf("expected auto drive to advance down the manual map, got z=%v", runner.State.VehicleChassis.Motion.Position.Z)
	}
	if maxConsecutiveAirborne > 8 {
		t.Fatalf("expected auto drive to avoid long airborne streaks, got max consecutive airborne ticks=%d", maxConsecutiveAirborne)
	}
}
