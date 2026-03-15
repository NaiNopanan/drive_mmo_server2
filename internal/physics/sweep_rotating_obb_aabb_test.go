package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestSweepRotatingOrientedBoxAxisAlignedBoundingBoxDetectsThinWallHit(t *testing.T) {
	body := physics.NewRigidBoxBody3D(
		fixed.One,
		geometry.NewVector3(fixed.FromFraction(1, 2), fixed.FromFraction(1, 2), fixed.FromFraction(1, 2)),
		geometry.NewVector3(fixed.FromInt(-2), fixed.FromInt(2), fixed.Zero),
	)
	body.Motion.Velocity = geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero)
	body.Orientation = physics.NewQuaternionFromEulerXYZ(
		fixed.FromFraction(1, 5),
		fixed.FromFraction(3, 20),
		fixed.FromFraction(1, 4),
	)
	body.AngularVelocity = geometry.NewVector3(fixed.FromInt(6), fixed.FromInt(4), fixed.FromInt(5))

	contact := physics.SweepRotatingOrientedBoxAxisAlignedBoundingBox(
		body,
		physics.DefaultTimeStep,
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(-4)),
			geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromInt(5), fixed.FromInt(4)),
		),
	)

	if !contact.Hit {
		t.Fatalf("expected rotating OBB sweep to hit the thin wall")
	}
	if contact.TimeOfImpact.Cmp(fixed.Zero) < 0 || contact.TimeOfImpact.Cmp(fixed.One) > 0 {
		t.Fatalf("expected time of impact inside step, got %v", contact.TimeOfImpact)
	}
}
