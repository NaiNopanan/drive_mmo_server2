package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/physics"
)

func TestClampToSymmetricRange(t *testing.T) {
	if got := physics.ClampToSymmetricRange(fixed.FromInt(12), fixed.FromInt(5)); got != fixed.FromInt(5) {
		t.Fatalf("positive clamp mismatch: got=%v", got)
	}
	if got := physics.ClampToSymmetricRange(fixed.FromInt(-12), fixed.FromInt(5)); got != fixed.FromInt(-5) {
		t.Fatalf("negative clamp mismatch: got=%v", got)
	}
}

func TestMoveTowardZero(t *testing.T) {
	if got := physics.MoveTowardZero(fixed.FromInt(3), fixed.FromInt(5)); got != fixed.Zero {
		t.Fatalf("expected zero when overshooting positive, got=%v", got)
	}
	if got := physics.MoveTowardZero(fixed.FromInt(-3), fixed.FromInt(5)); got != fixed.Zero {
		t.Fatalf("expected zero when overshooting negative, got=%v", got)
	}
}
