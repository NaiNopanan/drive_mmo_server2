package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) UpdateBasisFromYaw() {
	s := fixed.Sin(v.Yaw)
	c := fixed.Cos(v.Yaw)

	// forward = (sin(yaw), 0, cos(yaw))
	v.ForwardWS = geom.V3(s, fixed.Zero, c)

	// right = (cos(yaw), 0, -sin(yaw))
	v.RightWS = geom.V3(c, fixed.Zero, s.Neg())

	v.UpWS = geom.V3(fixed.Zero, fixed.One, fixed.Zero)
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
