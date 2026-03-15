package scenario_test

import (
	"reflect"
	"testing"

	"server2/internal/fixed"
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
