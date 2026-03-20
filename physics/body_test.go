package physics

import (
	"testing"

	"server2/math/fixed"
	"server2/math/geometry"
)

func TestStaticPlaneNormalIsNormalized(t *testing.T) {
	body := NewStaticPlane(geometry.ZeroVector3(), geometry.NewVector3(fixed.Zero, fixed.FromInt(5), fixed.Zero))
	if body.Plane.Normal.Y.Cmp(fixed.FromFraction(999, 1000)) < 0 {
		t.Fatalf("expected plane normal to point upward after normalization, got %v", body.Plane.Normal)
	}
}

func TestDynamicBoxHasInverseMass(t *testing.T) {
	body := NewDynamicBox(fixed.FromInt(2), geometry.ZeroVector3(), geometry.NewVector3(fixed.One, fixed.One, fixed.One))
	if body.InverseMass.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected dynamic body to have inverse mass")
	}
}

func TestIntegrateBodyMovesDynamicOnly(t *testing.T) {
	dynamicBody := NewDynamicSphere(fixed.One, geometry.ZeroVector3(), fixed.One)
	dynamicBody.Velocity = geometry.NewVector3(fixed.FromInt(2), fixed.Zero, fixed.Zero)
	IntegrateBody(&dynamicBody, fixed.FromFraction(1, 2), geometry.ZeroVector3())
	if dynamicBody.Position.X.Cmp(fixed.One) < 0 {
		t.Fatalf("expected dynamic body to move, got x=%v", dynamicBody.Position.X)
	}

	staticBody := NewStaticBox(geometry.ZeroVector3(), geometry.NewVector3(fixed.One, fixed.One, fixed.One))
	IntegrateBody(&staticBody, fixed.FromFraction(1, 2), geometry.ZeroVector3())
	if staticBody.Position.X.Cmp(fixed.Zero) != 0 {
		t.Fatalf("expected static body to remain still")
	}
}

func TestKinematicBodyIgnoresDynamicIntegration(t *testing.T) {
	body := NewKinematicBox(geometry.ZeroVector3(), geometry.NewVector3(fixed.One, fixed.One, fixed.One))
	body.Velocity = geometry.NewVector3(fixed.FromInt(3), fixed.Zero, fixed.Zero)
	IntegrateBody(&body, fixed.One, geometry.ZeroVector3())
	if body.Position.X.Cmp(fixed.Zero) != 0 {
		t.Fatalf("expected kinematic body not to be integrated as dynamic")
	}
}
