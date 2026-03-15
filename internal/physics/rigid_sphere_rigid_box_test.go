package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestResolveRigidSphereRigidBoxContactWithFrictionTransfersMotion(t *testing.T) {
	sphere := physics.NewRigidSphereBody3D(
		fixed.One,
		fixed.One,
		geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.Zero),
	)
	sphere.Motion.Velocity = geometry.NewVector3(fixed.FromInt(4), fixed.Zero, fixed.Zero)

	box := physics.NewRigidBoxBody3D(
		fixed.FromInt(2),
		geometry.NewVector3(fixed.One, fixed.One, fixed.One),
		geometry.NewVector3(fixed.FromInt(3), fixed.Zero, fixed.Zero),
	)

	contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(&sphere, &box, fixed.One, fixed.FromFraction(1, 4))
	if !contact.Hit {
		t.Fatalf("expected rigid sphere and rigid box to contact")
	}
	if sphere.Motion.Position.X.Cmp(fixed.FromFraction(3, 2)) >= 0 {
		t.Fatalf("expected sphere to be pushed left, got %v", sphere.Motion.Position)
	}
	if box.Motion.Position.X.Cmp(fixed.FromInt(3)) <= 0 {
		t.Fatalf("expected box to move right, got %v", box.Motion.Position)
	}
	if sphere.Motion.Velocity.X.Cmp(fixed.Zero) >= 0 {
		t.Fatalf("expected sphere to lose rightward speed, got %v", sphere.Motion.Velocity)
	}
	if box.Motion.Velocity.X.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected box to gain rightward speed, got %v", box.Motion.Velocity)
	}
}
