package physics_test

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func TestStepSphereBodyWithGravityFallsAndGroundsOnFlatTriangles(t *testing.T) {
	ground := []geometry.Triangle{
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(-20)),
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(-20)),
		),
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(-20)),
		),
	}

	body := physics.NewDynamicSphereBody(
		fixed.One,
		fixed.One,
		geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
	)

	lastStep := physics.SphereStepResult{}
	for step := 0; step < 180; step++ {
		lastStep = physics.StepSphereBodyWithGravity(&body, physics.DefaultTimeStep, physics.StandardGravity, ground)
	}

	if !lastStep.HadContact {
		t.Fatalf("expected contact during final simulation step")
	}
	if !body.Grounded {
		t.Fatalf("expected body to be grounded at the end")
	}
	if body.Motion.Position.Y.Sub(body.Radius).Abs().Cmp(fixed.FromRaw(1024)) > 0 {
		t.Fatalf("expected sphere to settle at radius height, got position=%v radius=%v", body.Motion.Position.Y, body.Radius)
	}
}

func TestStepSphereBodyWithGravityBouncesWhenRestitutionIsSet(t *testing.T) {
	ground := []geometry.Triangle{
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(-20)),
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(-20)),
		),
		geometry.NewTriangle(
			geometry.NewVector3(fixed.FromInt(-20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(20)),
			geometry.NewVector3(fixed.FromInt(20), fixed.Zero, fixed.FromInt(-20)),
		),
	}

	body := physics.NewDynamicSphereBody(
		fixed.One,
		fixed.One,
		geometry.NewVector3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
	)
	body.Restitution = fixed.FromFraction(4, 5)

	for step := 0; step < 90; step++ {
		result := physics.StepSphereBodyWithGravity(&body, physics.DefaultTimeStep, physics.StandardGravity, ground)
		if result.HadContact && body.Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
			return
		}
	}

	t.Fatalf("expected sphere body to bounce upward after contact")
}
