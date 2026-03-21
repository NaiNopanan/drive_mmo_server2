package geom

import "math"

// PlanarVec ใช้แทนตำแหน่งหรือเวกเตอร์บนระนาบโลก X/Z
type PlanarVec struct {
	X float32
	Z float32
}

func Planar(x, z float32) PlanarVec {
	return PlanarVec{X: x, Z: z}
}

func (v PlanarVec) Add(other PlanarVec) PlanarVec {
	return PlanarVec{X: v.X + other.X, Z: v.Z + other.Z}
}

func (v PlanarVec) Sub(other PlanarVec) PlanarVec {
	return PlanarVec{X: v.X - other.X, Z: v.Z - other.Z}
}

func (v PlanarVec) MulScalar(s float32) PlanarVec {
	return PlanarVec{X: v.X * s, Z: v.Z * s}
}

func (v PlanarVec) Dot(other PlanarVec) float32 {
	return v.X*other.X + v.Z*other.Z
}

func (v PlanarVec) Length() float32 {
	return float32(math.Hypot(float64(v.X), float64(v.Z)))
}

func (v PlanarVec) Normalized() PlanarVec {
	length := v.Length()
	if length == 0 {
		return PlanarVec{}
	}

	return PlanarVec{
		X: v.X / length,
		Z: v.Z / length,
	}
}

func (v PlanarVec) Distance(other PlanarVec) float32 {
	return v.Sub(other).Length()
}

func (v PlanarVec) ClampLength(max float32) PlanarVec {
	length := v.Length()
	if length == 0 || length <= max {
		return v
	}

	scale := max / length
	return v.MulScalar(scale)
}

func LerpPlanar(a, b PlanarVec, t float32) PlanarVec {
	return PlanarVec{
		X: a.X + (b.X-a.X)*t,
		Z: a.Z + (b.Z-a.Z)*t,
	}
}

func FromHeading(heading float32) PlanarVec {
	return PlanarVec{
		X: float32(math.Cos(float64(heading))),
		Z: float32(math.Sin(float64(heading))),
	}
}
