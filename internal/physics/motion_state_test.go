package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestStepLinearMotionWithGravityMovesDown(t *testing.T) {
	state := physics.NewDynamicMotionState(
		fixed.One,
		geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
	)

	physics.StepLinearMotionWithGravity(&state, physics.DefaultTimeStep, physics.StandardGravity)

	if state.Position.Y.Cmp(fixed.FromInt(10)) >= 0 {
		t.Fatalf("expected Y to decrease, got %v", state.Position.Y)
	}
	if state.Velocity.Y.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected downward velocity, got %v", state.Velocity.Y)
	}
	if state.AccumulatedForce != geometry.ZeroVector3() {
		t.Fatalf("expected cleared force accumulator, got %v", state.AccumulatedForce)
	}
}

func TestStaticMotionStateIgnoresForceAndIntegration(t *testing.T) {
	state := physics.NewStaticMotionState(
		geometry.NewVector3(fixed.Zero, fixed.FromInt(3), fixed.Zero),
	)

	physics.ApplyForce(&state, geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero))
	physics.StepLinearMotion(&state, physics.DefaultTimeStep)

	if state.Position != geometry.NewVector3(fixed.Zero, fixed.FromInt(3), fixed.Zero) {
		t.Fatalf("static state position changed: %v", state.Position)
	}
	if state.Velocity != geometry.ZeroVector3() {
		t.Fatalf("static state velocity changed: %v", state.Velocity)
	}
}
