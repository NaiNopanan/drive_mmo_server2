package geom

import (
	"testing"

	"server2/internal/fixed"
)

func TestTriangleNormalFlatUp(t *testing.T) {
	tri := NewTriangle(
		V3(fixed.FromInt(0), fixed.Zero, fixed.FromInt(0)),
		V3(fixed.FromInt(0), fixed.Zero, fixed.FromInt(1)),
		V3(fixed.FromInt(1), fixed.Zero, fixed.FromInt(0)),
	)

	n := tri.Normal()

	if n.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("expected upward normal, got %v", n)
	}
}

func TestClosestPointOnTriangleFace(t *testing.T) {
	tri := NewTriangle(
		V3(fixed.Zero, fixed.Zero, fixed.Zero),
		V3(fixed.FromInt(10), fixed.Zero, fixed.Zero),
		V3(fixed.Zero, fixed.Zero, fixed.FromInt(10)),
	)

	p := V3(fixed.FromInt(2), fixed.FromInt(5), fixed.FromInt(2))
	cp := tri.ClosestPoint(p)

	want := V3(fixed.FromInt(2), fixed.Zero, fixed.FromInt(2))
	// Check with a very small tolerance since normalization/sqrt might cause tiny deviations
	diff := cp.Sub(want)
	if diff.X.Abs().Cmp(100) > 0 || diff.Y.Abs().Cmp(100) > 0 || diff.Z.Abs().Cmp(100) > 0 {
		t.Fatalf("closest point mismatch: got=%v want=%v", cp, want)
	}
}
