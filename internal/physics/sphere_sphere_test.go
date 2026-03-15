package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestFindSphereSphereContactDetectsOverlap(t *testing.T) {
	contact := physics.FindSphereSphereContact(
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero),
		fixed.One,
		geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.Zero),
		fixed.One,
	)

	if !contact.Hit {
		t.Fatalf("expected sphere-sphere overlap")
	}
	if contact.Penetration.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected positive penetration, got %v", contact.Penetration)
	}
}

func TestResolveSphereSphereContactTransfersVelocity(t *testing.T) {
	positionA, velocityA, positionB, velocityB, contact := physics.ResolveSphereSphereContact(
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero),
		fixed.One,
		fixed.One,
		geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.Zero),
		geometry.ZeroVector3(),
		fixed.One,
		fixed.One,
		fixed.One,
	)

	if !contact.Hit {
		t.Fatalf("expected contact")
	}
	if positionA.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected sphere A to move left during separation, got %v", positionA)
	}
	if positionB.X.Cmp(fixed.FromFraction(3, 2)) <= 0 {
		t.Fatalf("expected sphere B to move right during separation, got %v", positionB)
	}
	if velocityA.X.Abs().Cmp(fixed.FromRaw(1024)) > 0 {
		t.Fatalf("expected sphere A to stop after elastic collision, got %v", velocityA)
	}
	if velocityB.X.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected sphere B to move right after collision, got %v", velocityB)
	}
}
