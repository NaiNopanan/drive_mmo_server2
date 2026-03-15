package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestSweepSphereTriangleMeshDetectsThinWallHit(t *testing.T) {
	triangles := []geometry.Triangle{
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(-4)),
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.FromInt(5), fixed.FromInt(-4)),
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(4)),
		),
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.Zero, fixed.FromInt(4)),
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.FromInt(5), fixed.FromInt(-4)),
			geometry.NewVector3(fixed.FromFraction(-1, 4), fixed.FromInt(5), fixed.FromInt(4)),
		),
	}

	contact := physics.SweepSphereTriangleMesh(
		geometry.NewVector3(fixed.FromInt(-2), fixed.FromInt(2), fixed.Zero),
		geometry.NewVector3(fixed.FromInt(80), fixed.Zero, fixed.Zero),
		fixed.FromFraction(1, 2),
		physics.DefaultTimeStep,
		triangles,
	)

	if !contact.Hit {
		t.Fatalf("expected swept sphere to hit the triangle mesh wall")
	}
	if contact.TimeOfImpact.Cmp(fixed.Zero) <= 0 || contact.TimeOfImpact.Cmp(fixed.One) >= 0 {
		t.Fatalf("expected time of impact inside the step, got %v", contact.TimeOfImpact)
	}
}
