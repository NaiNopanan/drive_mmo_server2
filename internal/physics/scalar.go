package physics

import "server2/internal/fixed"

// ClampToSymmetricRange clamps v into the range [-limit, +limit].
func ClampToSymmetricRange(v, limit fixed.Fixed) fixed.Fixed {
	if v.Cmp(limit) > 0 {
		return limit
	}

	min := limit.Neg()
	if v.Cmp(min) < 0 {
		return min
	}

	return v
}

// MoveTowardZero reduces the magnitude of v by delta without crossing zero.
func MoveTowardZero(v, delta fixed.Fixed) fixed.Fixed {
	if v.Cmp(fixed.Zero) > 0 {
		next := v.Sub(delta)
		if next.Cmp(fixed.Zero) < 0 {
			return fixed.Zero
		}
		return next
	}

	if v.Cmp(fixed.Zero) < 0 {
		next := v.Add(delta)
		if next.Cmp(fixed.Zero) > 0 {
			return fixed.Zero
		}
		return next
	}

	return fixed.Zero
}
