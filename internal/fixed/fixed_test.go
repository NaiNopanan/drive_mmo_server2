package fixed

import "testing"

func TestFromIntRaw(t *testing.T) {
	got := FromInt(1).Raw()
	want := int64(1) << FracBits

	if got != want {
		t.Fatalf("got raw=%d want=%d", got, want)
	}
}

func TestFromRatioHalf(t *testing.T) {
	got := FromRatio(1, 2)
	if got.Raw() != (int64(1) << (FracBits - 1)) {
		t.Fatalf("got raw=%d want=%d", got.Raw(), int64(1)<<(FracBits-1))
	}
}

func TestAddSub(t *testing.T) {
	a := FromInt(10)
	b := FromInt(3)

	if a.Add(b) != FromInt(13) {
		t.Fatalf("10+3 should be 13")
	}
	if a.Sub(b) != FromInt(7) {
		t.Fatalf("10-3 should be 7")
	}
}

func TestMul(t *testing.T) {
	a := FromInt(3)
	b := FromInt(2)

	got := a.Mul(b)
	want := FromInt(6)

	if got != want {
		t.Fatalf("got=%v want=%v rawGot=%d rawWant=%d", got, want, got.Raw(), want.Raw())
	}
}

func TestDiv(t *testing.T) {
	a := FromInt(7)
	b := FromInt(2)

	got := a.Div(b)
	want := FromInt(3).Add(Half) // 3.5

	if got != want {
		t.Fatalf("got=%v want=%v rawGot=%d rawWant=%d", got, want, got.Raw(), want.Raw())
	}
}

func TestNegativeMul(t *testing.T) {
	a := FromInt(-3)
	b := FromInt(2)

	got := a.Mul(b)
	want := FromInt(-6)

	if got != want {
		t.Fatalf("got=%v want=%v", got, want)
	}
}

func TestNegativeDiv(t *testing.T) {
	a := FromInt(-7)
	b := FromInt(2)

	got := a.Div(b)
	want := FromInt(-3).Sub(Half) // -3.5

	if got != want {
		t.Fatalf("got=%v want=%v rawGot=%d rawWant=%d", got, want, got.Raw(), want.Raw())
	}
}

func TestRawBitExactness(t *testing.T) {
	// 0.1 in decimal is not exactly representable in binary fixed point
	// but we should check that it is deterministic
	f1 := FromRatio(1, 10)
	f2 := FromRatio(1, 10)
	
	if f1.Raw() != f2.Raw() {
		t.Fatalf("FromRatio(1, 10) is not deterministic: %d != %d", f1.Raw(), f2.Raw())
	}
	
	// Check specific value for 0.5
	half := FromRatio(1, 2)
	if half.Raw() != 0x0000000080000000 {
		t.Fatalf("0.5 raw bit mismatch: got %x want %x", half.Raw(), 0x0000000080000000)
	}
}
