package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestSweepAxisAlignedBoxAxisAlignedBoundingBoxDetectsThinWallHit(t *testing.T) {
	contact := physics.SweepAxisAlignedBoxAxisAlignedBoundingBox(
		geometry.NewVector3(fixed.FromInt(-2), fixed.FromInt(2), fixed.Zero),
		geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.FromFraction(1, 2), fixed.FromFraction(1, 2), fixed.FromFraction(1, 2)),
		physics.DefaultTimeStep,
		geometry.NewAxisAlignedBoundingBox(
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(-4)),
			geometry.NewVector3(fixed.FromFraction(1, 4), fixed.FromInt(5), fixed.FromInt(4)),
		),
	)

	if !contact.Hit {
		t.Fatalf("expected swept box to hit the thin wall")
	}
	if contact.TimeOfImpact.Cmp(fixed.Zero) <= 0 || contact.TimeOfImpact.Cmp(fixed.One) >= 0 {
		t.Fatalf("expected time of impact inside the step, got %v", contact.TimeOfImpact)
	}
}
