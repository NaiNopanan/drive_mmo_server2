package geometry

import (
	"fmt"
	"server2/internal/fixed"
)

type Vector3 struct {
	X fixed.Fixed
	Y fixed.Fixed
	Z fixed.Fixed
}

func NewVector3(x, y, z fixed.Fixed) Vector3 {
	return Vector3{X: x, Y: y, Z: z}
}

func ZeroVector3() Vector3 {
	return Vector3{}
}

func (v Vector3) Add(o Vector3) Vector3 {
	return Vector3{
		X: v.X.Add(o.X),
		Y: v.Y.Add(o.Y),
		Z: v.Z.Add(o.Z),
	}
}

func (v Vector3) Sub(o Vector3) Vector3 {
	return Vector3{
		X: v.X.Sub(o.X),
		Y: v.Y.Sub(o.Y),
		Z: v.Z.Sub(o.Z),
	}
}

func (v Vector3) Scale(s fixed.Fixed) Vector3 {
	return Vector3{
		X: v.X.Mul(s),
		Y: v.Y.Mul(s),
		Z: v.Z.Mul(s),
	}
}

func (v Vector3) Dot(o Vector3) fixed.Fixed {
	return v.X.Mul(o.X).
		Add(v.Y.Mul(o.Y)).
		Add(v.Z.Mul(o.Z))
}

func (v Vector3) Cross(o Vector3) Vector3 {
	return Vector3{
		X: v.Y.Mul(o.Z).Sub(v.Z.Mul(o.Y)),
		Y: v.Z.Mul(o.X).Sub(v.X.Mul(o.Z)),
		Z: v.X.Mul(o.Y).Sub(v.Y.Mul(o.X)),
	}
}

func (v Vector3) Neg() Vector3 {
	return Vector3{
		X: v.X.Neg(),
		Y: v.Y.Neg(),
		Z: v.Z.Neg(),
	}
}

func (v Vector3) LengthSquared() fixed.Fixed {
	return v.Dot(v)
}

func (v Vector3) Length() fixed.Fixed {
	return v.LengthSquared().Sqrt()
}

func (v Vector3) Normalize() Vector3 {
	lenSq := v.LengthSquared()
	if lenSq == fixed.Zero {
		return ZeroVector3()
	}

	length := lenSq.Sqrt()
	invLen := fixed.One.Div(length)
	return v.Scale(invLen)
}

func (v Vector3) String() string {
	return fmt.Sprintf("{%s, %s, %s}", v.X, v.Y, v.Z)
}
