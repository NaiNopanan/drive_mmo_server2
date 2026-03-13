package fixed

import (
	"fmt"
	"math/big"
	"math/bits"
)

type Fixed int64

const (
	FracBits = 32
)

const (
	maxInt64 = int64(^uint64(0) >> 1)
	minInt64 = -maxInt64 - 1
)

const (
	Zero Fixed = 0
	One  Fixed = 1 << FracBits
)

func Clamp(v, min, max Fixed) Fixed {
	if v.Cmp(min) < 0 {
		return min
	}
	if v.Cmp(max) > 0 {
		return max
	}
	return v
}

func FromInt(v int64) Fixed {
	if v > (maxInt64 >> FracBits) || v < (minInt64 >> FracBits) {
		panic("fixed.FromInt overflow")
	}
	return Fixed(v << FracBits)
}

func FromFraction(num, den int64) Fixed {
	if den == 0 {
		panic("fixed.FromFraction divide by zero")
	}

	// Use big.Int for high-precision constant/config construction
	n := big.NewInt(num)
	n.Lsh(n, FracBits)

	d := big.NewInt(den)
	n.Quo(n, d)

	if !n.IsInt64() {
		panic("fixed.FromFraction overflow")
	}

	return Fixed(n.Int64())
}

func FromRaw(raw int64) Fixed {
	return Fixed(raw)
}

func (f Fixed) Raw() int64 {
	return int64(f)
}

func (f Fixed) IntFloor() int64 {
	return int64(f) >> FracBits
}

func (f Fixed) Add(g Fixed) Fixed {
	return Fixed(add64(int64(f), int64(g)))
}

func (f Fixed) Sub(g Fixed) Fixed {
	return Fixed(sub64(int64(f), int64(g)))
}

func (f Fixed) Neg() Fixed {
	if int64(f) == minInt64 {
		panic("fixed.Neg overflow")
	}
	return -f
}

func (f Fixed) Abs() Fixed {
	if f < 0 {
		return f.Neg()
	}
	return f
}

func (f Fixed) Mul(g Fixed) Fixed {
	return Fixed(mulQ32(int64(f), int64(g)))
}

func (f Fixed) Div(g Fixed) Fixed {
	if g == 0 {
		panic("fixed.Div: division by zero")
	}
	
	neg := (f < 0) != (g < 0)
	
	ua := absToUint64(int64(f))
	ub := absToUint64(int64(g))
	
	// (ua << 32) / ub
	hi := ua >> (64 - FracBits)
	lo := ua << FracBits
	
	q, _ := bits.Div64(hi, lo, ub)
	
	// Check overflow for q (it's result of 128/64, could be > 64 bit but applySign will handle int64 limit)
	return applySign(q, neg)
}

func (f Fixed) Sqrt() Fixed {
	if f < 0 {
		panic("fixed.Sqrt: negative input")
	}
	if f == 0 {
		return Zero
	}

	// raw(result) = sqrt(raw(input) << 32)
	x := new(big.Int).SetInt64(int64(f))
	x.Lsh(x, FracBits)

	z := new(big.Int).Sqrt(x)
	if !z.IsInt64() {
		panic("fixed.Sqrt: overflow")
	}

	return Fixed(z.Int64())
}

func (f Fixed) Cmp(g Fixed) int {
	if f < g {
		return -1
	}
	if f > g {
		return 1
	}
	return 0
}

func (f Fixed) String() string {
	u := absToUint64(int64(f))
	whole := u >> FracBits
	fracRaw := u & ((uint64(1) << FracBits) - 1)

	// Display 6 positions for debug
	frac6 := (fracRaw * 1_000_000) >> FracBits

	if f < 0 {
		return fmt.Sprintf("-%d.%06d", whole, frac6)
	}
	return fmt.Sprintf("%d.%06d", whole, frac6)
}

func add64(a, b int64) int64 {
	r := a + b
	if ((a ^ r) & (b ^ r)) < 0 {
		panic("fixed add overflow")
	}
	return r
}

func sub64(a, b int64) int64 {
	r := a - b
	if ((a ^ b) & (a ^ r)) < 0 {
		panic("fixed sub overflow")
	}
	return r
}

func mulQ32(a, b int64) int64 {
	neg := (a < 0) != (b < 0)

	ua := absToUint64(a)
	ub := absToUint64(b)

	hi, lo := bits.Mul64(ua, ub)

	// (hi:lo) >> 32
	if (hi >> 32) != 0 {
		panic("fixed mul overflow")
	}

	u := (hi << 32) | (lo >> 32)

	return int64(applySign(u, neg))
}

func absToUint64(v int64) uint64 {
	if v >= 0 {
		return uint64(v)
	}
	// two's complement abs that supports MinInt64
	return uint64(^v) + 1
}

func applySign(u uint64, neg bool) Fixed {
	if neg {
		if u == (uint64(1) << 63) {
			return Fixed(minInt64)
		}
		if u > uint64(maxInt64) {
			panic("fixed overflow")
		}
		return Fixed(-int64(u))
	}

	if u > uint64(maxInt64) {
		panic("fixed overflow")
	}
	return Fixed(int64(u))
}
