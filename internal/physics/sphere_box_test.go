package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestFindSphereBoxContactDetectsOverlap(t *testing.T) {
	contact := physics.FindSphereBoxContact(
		geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.Zero),
		fixed.One,
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.Zero, fixed.One.Neg(), fixed.One.Neg()),
			geometry.NewVector3(fixed.One, fixed.One, fixed.One),
		),
	)

	if !contact.Hit {
		t.Fatalf("expected sphere-box overlap")
	}
	if contact.Penetration.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected positive penetration, got %v", contact.Penetration)
	}
}

func TestResolveSphereBoxContactPushesSphereAndMovesBox(t *testing.T) {
	spherePosition, sphereVelocity, boxPosition, boxVelocity, contact := physics.ResolveSphereBoxContact(
		geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero),
		fixed.One,
		fixed.One,
		geometry.NewVector3(fixed.FromInt(3), fixed.Zero, fixed.Zero),
		geometry.ZeroVector3(),
		fixed.FromFraction(1, 2),
		geometry.NewVector3(fixed.One, fixed.One, fixed.One),
		fixed.One,
	)

	if !contact.Hit {
		t.Fatalf("expected contact")
	}
	if spherePosition.X.Cmp(fixed.FromFraction(3, 2)) >= 0 {
		t.Fatalf("expected sphere to be pushed left of its previous overlap, got %v", spherePosition)
	}
	if boxPosition.X.Cmp(fixed.FromInt(3)) <= 0 {
		t.Fatalf("expected box to move right, got %v", boxPosition)
	}
	if sphereVelocity.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected sphere to lose or reverse rightward motion, got %v", sphereVelocity)
	}
	if boxVelocity.X.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected box to gain rightward motion, got %v", boxVelocity)
	}
}
