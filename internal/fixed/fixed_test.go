package fixed

import "testing"

func TestFixedAddSub(t *testing.T) {
	a := FromInt(10)
	b := FromInt(3)

	if got := a.Add(b); got != FromInt(13) {
		t.Fatalf("add failed: got=%v want=%v", got, FromInt(13))
	}

	if got := a.Sub(b); got != FromInt(7) {
		t.Fatalf("sub failed: got=%v want=%v", got, FromInt(7))
	}
}

func TestFixedMul(t *testing.T) {
	a := FromFraction(3, 2) // 1.5
	b := FromInt(2)

	got := a.Mul(b)
	want := FromInt(3)

	if got != want {
		t.Fatalf("mul failed: got=%v want=%v", got, want)
	}
}

func TestFixedNegative(t *testing.T) {
	a := FromFraction(-3, 2) // -1.5
	b := FromInt(2)

	got := a.Mul(b)
	want := FromInt(-3)

	if got != want {
		t.Fatalf("negative mul failed: got=%v want=%v", got, want)
	}
}

func TestOverflow(t *testing.T) {
	defer func() {
		if r := recover(); r == nil {
			t.Errorf("The code did not panic on overflow")
		}
	}()

	a := FromRaw(maxInt64)
	b := FromInt(1)
	a.Add(b) // Should panic
}

func TestRawBitExactness(t *testing.T) {
	f1 := FromFraction(1, 10)
	f2 := FromFraction(1, 10)
	
	if f1.Raw() != f2.Raw() {
		t.Fatalf("FromFraction(1, 10) is not deterministic: %d != %d", f1.Raw(), f2.Raw())
	}
	
	half := FromFraction(1, 2)
	if half.Raw() != 0x0000000080000000 {
		t.Fatalf("0.5 raw bit mismatch: got %x want %x", half.Raw(), 0x0000000080000000)
	}
}
