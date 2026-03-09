package fixed

import (
	"fmt"
	"math/bits"
)

type Fixed int64

const (
	FracBits = 32
)

const (
	Zero Fixed = 0
	One  Fixed = Fixed(1) << FracBits
	Half Fixed = One >> 1
)

const oneRaw uint64 = uint64(1) << FracBits
const maxPositiveAbs uint64 = ^uint64(0) >> 1 // max int64 as uint64

func FromRaw(raw int64) Fixed {
	return Fixed(raw)
}

func FromInt(v int64) Fixed {
	return Fixed(v << FracBits)
}

func FromRatio(num, den int64) Fixed {
	if den == 0 {
		panic("fixed: division by zero in FromRatio")
	}
	// Fixed-point division already handles scaling
	return FromInt(num).Div(FromInt(den))
}

func (f Fixed) Raw() int64 {
	return int64(f)
}

func (f Fixed) Add(g Fixed) Fixed {
	return f + g
}

func (f Fixed) Sub(g Fixed) Fixed {
	return f - g
}

func (f Fixed) Neg() Fixed {
	return -f
}

func (f Fixed) Abs() Fixed {
	if f < 0 {
		return -f
	}
	return f
}

func (f Fixed) IsZero() bool {
	return f == 0
}

func (f Fixed) Cmp(g Fixed) int {
	switch {
	case f < g:
		return -1
	case f > g:
		return 1
	default:
		return 0
	}
}

func (f Fixed) Mul(g Fixed) Fixed {
	neg := (f < 0) != (g < 0)

	uf := abs64(int64(f))
	ug := abs64(int64(g))

	hi, lo := bits.Mul64(uf, ug)

	// ((hi:lo) >> 32)
	// For Q32.32, the result of 64x64 mul is 128 bit.
	// (uf * ug) >> 32
	// Higher 64 bits (hi) and lower 64 bits (lo)
	// res = (hi << 32) | (lo >> 32)
	
	if (hi >> FracBits) != 0 {
		panic("fixed: multiplication overflow")
	}

	resAbs := (hi << (64 - FracBits)) | (lo >> FracBits)
	return applySign(resAbs, neg)
}

func (f Fixed) Div(g Fixed) Fixed {
	if g == 0 {
		panic("fixed: division by zero")
	}

	neg := (f < 0) != (g < 0)

	uf := abs64(int64(f))
	ug := abs64(int64(g))

	// (uf << 32) / ug
	// To maintain precision, we shift uf left by 32 bits before dividing.
	// Since uf is 64-bit, we need bits.Div64 for 128-bit/64-bit division.
	
	hi := uf >> (64 - FracBits)
	lo := uf << FracBits

	q, _ := bits.Div64(hi, lo, ug)
	return applySign(q, neg)
}

func Min(a, b Fixed) Fixed {
	if a < b {
		return a
	}
	return b
}

func Max(a, b Fixed) Fixed {
	if a > b {
		return a
	}
	return b
}

func (f Fixed) String() string {
	return f.Format(6)
}

func (f Fixed) Format(places int) string {
	raw := int64(f)
	neg := raw < 0
	abs := abs64(raw)

	whole := abs >> FracBits
	fracRaw := abs & (oneRaw - 1)

	pow10 := uint64(1)
	for i := 0; i < places; i++ {
		pow10 *= 10
	}

	frac := (fracRaw*pow10 + oneRaw/2) / oneRaw
	if frac >= pow10 {
		whole++
		frac = 0
	}

	if places == 0 {
		if neg {
			return fmt.Sprintf("-%d", whole)
		}
		return fmt.Sprintf("%d", whole)
	}

	if neg {
		return fmt.Sprintf("-%d.%0*d", whole, places, frac)
	}
	return fmt.Sprintf("%d.%0*d", whole, places, frac)
}

func abs64(v int64) uint64 {
	if v < 0 {
		return uint64(^v) + 1
	}
	return uint64(v)
}

func applySign(abs uint64, neg bool) Fixed {
	if !neg {
		if abs > maxPositiveAbs {
			panic("fixed: positive overflow")
		}
		return Fixed(int64(abs))
	}

	if abs > (uint64(1) << 63) {
		panic("fixed: negative overflow")
	}
	if abs == (uint64(1) << 63) {
		return Fixed(-1 << 63)
	}
	return Fixed(-int64(abs))
}
