package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) UpdateBasis(up geom.Vec3) {
	// 1) Target Up
	v.UpWS = up.Normalize()

	// 2) Horizontal heading from Yaw
	s := fixed.Sin(v.Yaw)
	c := fixed.Cos(v.Yaw)
	worldFwd := geom.V3(s, fixed.Zero, c)

	// 3) Right = Cross(Up, worldFwd)
	v.RightWS = v.UpWS.Cross(worldFwd)
	if v.RightWS.LengthSq().Cmp(fixed.Zero) == 0 {
		// Degenerate (e.g. looking straight up/down)
		v.RightWS = geom.V3(fixed.One, fixed.Zero, fixed.Zero)
	} else {
		v.RightWS = v.RightWS.Normalize()
	}

	// 4) Forward = Cross(Right, Up)
	v.ForwardWS = v.RightWS.Cross(v.UpWS).Normalize()
}

func (v *Vehicle) UpdateBasisFromYaw() {
	v.UpdateBasis(geom.V3(fixed.Zero, fixed.One, fixed.Zero))
}

func (v *Vehicle) LocalToWorld(local geom.Vec3) geom.Vec3 {
	// world = right*x + up*y + forward*z
	return v.RightWS.Scale(local.X).
		Add(v.UpWS.Scale(local.Y)).
		Add(v.ForwardWS.Scale(local.Z))
}

func HeadingFromYaw(yaw fixed.Fixed) (forward, right geom.Vec3) {
	s := fixed.Sin(yaw)
	c := fixed.Cos(yaw)
	forward = geom.V3(s, fixed.Zero, c)
	right = geom.V3(c, fixed.Zero, s.Neg())
	return
}
