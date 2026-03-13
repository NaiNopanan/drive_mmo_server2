package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type GroundHit struct {
	Hit           bool
	Point         geom.Vec3
	Normal        geom.Vec3
	TriangleIndex int
}

func SampleGroundAtXZ(
	tris []geom.Triangle,
	x fixed.Fixed,
	z fixed.Fixed,
	maxY fixed.Fixed,
) GroundHit {
	found := false
	var best GroundHit

	for i, tri := range tris {
		n := tri.Normal()

		// ignore non-ground / vertical-like triangles
		if n.Y.Cmp(fixed.Zero) <= 0 {
			continue
		}

		if !pointInTriangleXZ(x, z, tri) {
			continue
		}

		y, ok := solvePlaneY(tri, n, x, z)
		if !ok {
			continue
		}

		// only accept ground at or below query height
		if y.Cmp(maxY) > 0 {
			continue
		}

		if !found || y.Cmp(best.Point.Y) > 0 {
			best = GroundHit{
				Hit:           true,
				Point:         geom.V3(x, y, z),
				Normal:        n,
				TriangleIndex: i,
			}
			found = true
		}
	}

	return best
}

func solvePlaneY(
	tri geom.Triangle,
	n geom.Vec3,
	x fixed.Fixed,
	z fixed.Fixed,
) (fixed.Fixed, bool) {
	if n.Y == fixed.Zero {
		return fixed.Zero, false
	}

	dx := x.Sub(tri.A.X)
	dz := z.Sub(tri.A.Z)

	num := n.X.Mul(dx).Add(n.Z.Mul(dz))
	y := tri.A.Y.Sub(num.Div(n.Y))

	return y, true
}

func pointInTriangleXZ(x fixed.Fixed, z fixed.Fixed, tri geom.Triangle) bool {
	d1 := sign2DXZ(x, z, tri.A.X, tri.A.Z, tri.B.X, tri.B.Z)
	d2 := sign2DXZ(x, z, tri.B.X, tri.B.Z, tri.C.X, tri.C.Z)
	d3 := sign2DXZ(x, z, tri.C.X, tri.C.Z, tri.A.X, tri.A.Z)

	hasNeg := d1.Cmp(fixed.Zero) < 0 || d2.Cmp(fixed.Zero) < 0 || d3.Cmp(fixed.Zero) < 0
	hasPos := d1.Cmp(fixed.Zero) > 0 || d2.Cmp(fixed.Zero) > 0 || d3.Cmp(fixed.Zero) > 0

	return !(hasNeg && hasPos)
}

func sign2DXZ(
	px fixed.Fixed,
	pz fixed.Fixed,
	ax fixed.Fixed,
	az fixed.Fixed,
	bx fixed.Fixed,
	bz fixed.Fixed,
) fixed.Fixed {
	return px.Sub(bx).Mul(az.Sub(bz)).Sub(ax.Sub(bx).Mul(pz.Sub(bz)))
}
