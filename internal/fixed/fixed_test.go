package fixed_test

import (
	"testing"

	"server2/internal/fixed"
)

const maxSignedInt64 = int64(^uint64(0) >> 1)

func TestFixedAddAndSubtract(t *testing.T) {
	a := fixed.FromInt(10)
	b := fixed.FromInt(3)

	if got := a.Add(b); got != fixed.FromInt(13) {
		t.Fatalf("add failed: got=%v want=%v", got, fixed.FromInt(13))
	}

	if got := a.Sub(b); got != fixed.FromInt(7) {
		t.Fatalf("sub failed: got=%v want=%v", got, fixed.FromInt(7))
	}
}

func TestFixedMultiply(t *testing.T) {
	a := fixed.FromFraction(3, 2)
	b := fixed.FromInt(2)

	got := a.Mul(b)
	want := fixed.FromInt(3)
	if got != want {
		t.Fatalf("multiply failed: got=%v want=%v", got, want)
	}
}

func TestFixedMultiplyNegative(t *testing.T) {
	a := fixed.FromFraction(-3, 2)
	b := fixed.FromInt(2)

	got := a.Mul(b)
	want := fixed.FromInt(-3)
	if got != want {
		t.Fatalf("negative multiply failed: got=%v want=%v", got, want)
	}
}

func TestFixedAddPanicsOnOverflow(t *testing.T) {
	defer func() {
		if r := recover(); r == nil {
			t.Fatalf("expected panic on overflow")
		}
	}()

	a := fixed.FromRaw(maxSignedInt64)
	b := fixed.FromInt(1)
	_ = a.Add(b)
}

func TestFromFractionIsBitExact(t *testing.T) {
	f1 := fixed.FromFraction(1, 10)
	f2 := fixed.FromFraction(1, 10)

	if f1.Raw() != f2.Raw() {
		t.Fatalf("FromFraction(1, 10) is not deterministic: %d != %d", f1.Raw(), f2.Raw())
	}

	half := fixed.FromFraction(1, 2)
	if half.Raw() != 0x0000000080000000 {
		t.Fatalf("0.5 raw bit mismatch: got %x want %x", half.Raw(), 0x0000000080000000)
	}
}
