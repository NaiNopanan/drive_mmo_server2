package geom

import (
	"fmt"
	"server2/internal/fixed"
)

type Vec3 struct {
	X fixed.Fixed
	Y fixed.Fixed
	Z fixed.Fixed
}

func V3(x, y, z fixed.Fixed) Vec3 {
	return Vec3{X: x, Y: y, Z: z}
}

func Zero() Vec3 {
	return Vec3{}
}

func (v Vec3) Add(o Vec3) Vec3 {
	return Vec3{
		X: v.X.Add(o.X),
		Y: v.Y.Add(o.Y),
		Z: v.Z.Add(o.Z),
	}
}

func (v Vec3) Sub(o Vec3) Vec3 {
	return Vec3{
		X: v.X.Sub(o.X),
		Y: v.Y.Sub(o.Y),
		Z: v.Z.Sub(o.Z),
	}
}

func (v Vec3) Scale(s fixed.Fixed) Vec3 {
	return Vec3{
		X: v.X.Mul(s),
		Y: v.Y.Mul(s),
		Z: v.Z.Mul(s),
	}
}

func (v Vec3) Dot(o Vec3) fixed.Fixed {
	return v.X.Mul(o.X).
		Add(v.Y.Mul(o.Y)).
		Add(v.Z.Mul(o.Z))
}

func (v Vec3) Cross(o Vec3) Vec3 {
	return Vec3{
		X: v.Y.Mul(o.Z).Sub(v.Z.Mul(o.Y)),
		Y: v.Z.Mul(o.X).Sub(v.X.Mul(o.Z)),
		Z: v.X.Mul(o.Y).Sub(v.Y.Mul(o.X)),
	}
}

func (v Vec3) Neg() Vec3 {
	return Vec3{
		X: v.X.Neg(),
		Y: v.Y.Neg(),
		Z: v.Z.Neg(),
	}
}

func (v Vec3) LengthSq() fixed.Fixed {
	return v.Dot(v)
}

func (v Vec3) Normalize() Vec3 {
	lenSq := v.LengthSq()
	if lenSq == fixed.Zero {
		return Zero()
	}

	length := lenSq.Sqrt()
	invLen := fixed.One.Div(length)
	return v.Scale(invLen)
}

func (v Vec3) String() string {
	return fmt.Sprintf("{%s, %s, %s}", v.X, v.Y, v.Z)
}
