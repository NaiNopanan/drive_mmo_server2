package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func GroundFlatSmall() []geom.Triangle {
	size := fixed.FromInt(50)
	y := fixed.Zero

	a := geom.V3(size.Neg(), y, size.Neg())
	b := geom.V3(size, y, size.Neg())
	c := geom.V3(size.Neg(), y, size)
	d := geom.V3(size, y, size)

	return []geom.Triangle{
		geom.NewTriangle(a, c, b),
		geom.NewTriangle(c, d, b),
	}
}

func GroundSlopeSmall() []geom.Triangle {
	size := fixed.FromInt(40)
	rise := fixed.FromInt(8)

	a := geom.V3(size.Neg(), fixed.Zero, size.Neg())
	b := geom.V3(size, fixed.Zero, size.Neg())
	c := geom.V3(size.Neg(), rise, size)
	d := geom.V3(size, rise, size)

	return []geom.Triangle{
		geom.NewTriangle(a, c, b),
		geom.NewTriangle(c, d, b),
	}
}
