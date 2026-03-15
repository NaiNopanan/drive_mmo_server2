package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestFindSphereTriangleContactDetectsFlatGroundContact(t *testing.T) {
	triangle := geometry.NewTriangle(
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(-10)),
		geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.FromInt(-10)),
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(10)),
	)

	contact := physics.FindSphereTriangleContact(
		geometry.NewVector3(fixed.Zero, fixed.FromFraction(1, 2), fixed.Zero),
		fixed.One,
		triangle,
	)

	if !contact.Hit {
		t.Fatalf("expected collision")
	}
	if contact.Penetration.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected positive penetration, got %v", contact.Penetration)
	}
	if contact.Normal.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected upward normal, got %v", contact.Normal)
	}
}

func TestResolveSphereTriangleContactPushesOutAndStopsInwardVelocity(t *testing.T) {
	triangle := geometry.NewTriangle(
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(-10)),
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(10)),
		geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.FromInt(-10)),
	)

	position, velocity, contact := physics.ResolveSphereTriangleContact(
		geometry.NewVector3(fixed.Zero, fixed.FromFraction(1, 2), fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.FromInt(-3), fixed.Zero),
		fixed.One,
		triangle,
	)

	if !contact.Hit {
		t.Fatalf("expected collision")
	}
	if position.Y.Cmp(fixed.One) < 0 {
		t.Fatalf("expected resolved sphere center above plane, got %v", position.Y)
	}
	if velocity.Y.Cmp(fixed.Zero) < 0 {
		t.Fatalf("expected downward velocity to be removed, got %v", velocity.Y)
	}
}

func TestResolveSphereTriangleContactWithRestitutionBouncesUpward(t *testing.T) {
	triangle := geometry.NewTriangle(
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(-10)),
		geometry.NewVector3(fixed.FromInt(-10), fixed.Zero, fixed.FromInt(10)),
		geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.FromInt(-10)),
	)

	position, velocity, contact := physics.ResolveSphereTriangleContactWithRestitution(
		geometry.NewVector3(fixed.Zero, fixed.FromFraction(1, 2), fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.FromInt(-3), fixed.Zero),
		fixed.One,
		triangle,
		fixed.FromFraction(4, 5),
	)

	if !contact.Hit {
		t.Fatalf("expected collision")
	}
	if position.Y.Cmp(fixed.One) < 0 {
		t.Fatalf("expected resolved sphere center above plane, got %v", position.Y)
	}
	if velocity.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected bounce velocity to be upward, got %v", velocity.Y)
	}
}
