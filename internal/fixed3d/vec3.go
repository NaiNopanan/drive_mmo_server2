package fixed3d

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

func (a Vec3) Add(b Vec3) Vec3 {
	return Vec3{
		X: a.X.Add(b.X),
		Y: a.Y.Add(b.Y),
		Z: a.Z.Add(b.Z),
	}
}

func (a Vec3) Sub(b Vec3) Vec3 {
	return Vec3{
		X: a.X.Sub(b.X),
		Y: a.Y.Sub(b.Y),
		Z: a.Z.Sub(b.Z),
	}
}

func (a Vec3) Neg() Vec3 {
	return Vec3{
		X: a.X.Neg(),
		Y: a.Y.Neg(),
		Z: a.Z.Neg(),
	}
}

func (a Vec3) MulScalar(s fixed.Fixed) Vec3 {
	return Vec3{
		X: a.X.Mul(s),
		Y: a.Y.Mul(s),
		Z: a.Z.Mul(s),
	}
}

func (a Vec3) DivScalar(s fixed.Fixed) Vec3 {
	return Vec3{
		X: a.X.Div(s),
		Y: a.Y.Div(s),
		Z: a.Z.Div(s),
	}
}

func (a Vec3) Dot(b Vec3) fixed.Fixed {
	return a.X.Mul(b.X).
		Add(a.Y.Mul(b.Y)).
		Add(a.Z.Mul(b.Z))
}

func (a Vec3) Cross(b Vec3) Vec3 {
	return Vec3{
		X: a.Y.Mul(b.Z).Sub(a.Z.Mul(b.Y)),
		Y: a.Z.Mul(b.X).Sub(a.X.Mul(b.Z)),
		Z: a.X.Mul(b.Y).Sub(a.Y.Mul(b.X)),
	}
}

func (a Vec3) String() string {
	return fmt.Sprintf("(%s, %s, %s)", a.X.String(), a.Y.String(), a.Z.String())
}
