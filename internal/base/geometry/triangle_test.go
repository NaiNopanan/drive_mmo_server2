package geometry_test

import (
	"testing"

	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
)

func TestTriangleNormalPointsUpOnFlatSurface(t *testing.T) {
	triangle := geometry.NewTriangle(
		geometry.NewVector3(fixed.FromInt(0), fixed.Zero, fixed.FromInt(0)),
		geometry.NewVector3(fixed.FromInt(0), fixed.Zero, fixed.FromInt(1)),
		geometry.NewVector3(fixed.FromInt(1), fixed.Zero, fixed.FromInt(0)),
	)

	normal := triangle.Normal()
	if normal.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected upward normal, got %v", normal)
	}
}

func TestTriangleClosestPointOnFace(t *testing.T) {
	triangle := geometry.NewTriangle(
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.FromInt(10), fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.FromInt(10)),
	)

	point := geometry.NewVector3(fixed.FromInt(2), fixed.FromInt(5), fixed.FromInt(2))
	closest := triangle.ClosestPointTo(point)
	want := geometry.NewVector3(fixed.FromInt(2), fixed.Zero, fixed.FromInt(2))

	diff := closest.Sub(want)
	if diff.X.Abs().Cmp(100) > 0 || diff.Y.Abs().Cmp(100) > 0 || diff.Z.Abs().Cmp(100) > 0 {
		t.Fatalf("closest point mismatch: got=%v want=%v", closest, want)
	}
}
