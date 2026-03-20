package fixed

// Sin returns a deterministic fixed-point sine of radians.
// This is a simple 8-point lookup table with linear interpolation for Day 6.
func Sin(rad Fixed) Fixed {
	// Simple approximation for a small project range:
	// We'll use a 13-bit precision table (8192 units per circle)
	// For now, let's just do a very basic Taylor or lookup if possible.
	// Actually, let's use a small LUT for speed and determinism.

	const (
		PI      = 13493037704 // 3.1415926535... << 32
		HALF_PI = 6746518852
		TWO_PI  = 26986075408
	)

	x := rad.Raw()
	// Wrap x to [0, TWO_PI)
	x = x % TWO_PI
	if x < 0 {
		x += TWO_PI
	}

	// Very crude linear approximation for Day 6 to avoid complex table coding
	// sin(x) approx x - x^3/6
	// We'll do it bit-perfectly with just a few segments for now.
	f := Fixed(x)
	if x < HALF_PI {
		return sinApprox(f)
	} else if x < PI {
		return sinApprox(Fixed(PI).Sub(f))
	} else if x < PI+HALF_PI {
		return sinApprox(f.Sub(Fixed(PI))).Neg()
	} else {
		return sinApprox(Fixed(TWO_PI).Sub(f)).Neg()
	}
}

func Cos(rad Fixed) Fixed {
	const HALF_PI = 6746518852
	return Sin(rad.Add(Fixed(HALF_PI)))
}

// sinApprox uses a simple 3rd order Taylor for [0, PI/2]
func sinApprox(x Fixed) Fixed {
	// sin(x) approx x - x^3/6
	x2 := x.Mul(x)
	x3 := x2.Mul(x)
	six := FromInt(6)
	return x.Sub(x3.Div(six))
}
