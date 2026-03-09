package fixed3d

import (
	"testing"

	"server2/internal/fixed"
)

func TestVec3Add(t *testing.T) {
	a := V3(fixed.FromInt(1), fixed.FromInt(2), fixed.FromInt(3))
	b := V3(fixed.FromInt(4), fixed.FromInt(5), fixed.FromInt(6))

	got := a.Add(b)
	want := V3(fixed.FromInt(5), fixed.FromInt(7), fixed.FromInt(9))

	if got != want {
		t.Fatalf("got=%v want=%v", got, want)
	}
}

func TestVec3Dot(t *testing.T) {
	a := V3(fixed.FromInt(1), fixed.Zero, fixed.Zero)
	b := V3(fixed.Zero, fixed.FromInt(1), fixed.Zero)

	got := a.Dot(b)
	if got != fixed.Zero {
		t.Fatalf("expected 0, got %v raw=%d", got, got.Raw())
	}
}

func TestVec3Cross(t *testing.T) {
	a := V3(fixed.FromInt(1), fixed.Zero, fixed.Zero)
	b := V3(fixed.Zero, fixed.FromInt(1), fixed.Zero)

	got := a.Cross(b)
	want := V3(fixed.Zero, fixed.Zero, fixed.FromInt(1))

	if got != want {
		t.Fatalf("got=%v want=%v", got, want)
	}
}

func TestVec3RawBitConsistency(t *testing.T) {
	v := V3(fixed.FromRatio(1, 3), fixed.FromRatio(1, 3), fixed.FromRatio(1, 3))
	
	// Shift by adding another vector
	v2 := v.Add(V3(fixed.One, fixed.One, fixed.One))
	v3 := v2.Sub(V3(fixed.One, fixed.One, fixed.One))
	
	if v.X.Raw() != v3.X.Raw() || v.Y.Raw() != v3.Y.Raw() || v.Z.Raw() != v3.Z.Raw() {
		t.Fatalf("raw bits not consistent after add/sub: %v != %v", v, v3)
	}
}
