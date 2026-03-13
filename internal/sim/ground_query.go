package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type GroundHit struct {
	Hit      bool
	Point    geom.Vec3
	Normal   geom.Vec3
	Distance fixed.Fixed
}

type GroundQuery interface {
	Raycast(origin geom.Vec3, dir geom.Vec3, maxDist fixed.Fixed) GroundHit
}

// WorldGroundQuery implements GroundQuery using the existing triangle slice.
type WorldGroundQuery struct {
	Triangles []geom.Triangle
}

func (wq WorldGroundQuery) Raycast(origin geom.Vec3, dir geom.Vec3, maxDist fixed.Fixed) GroundHit {
	// For Day 6, we assume dir is mostly downward.
	hit := sampleGroundAtXZ(wq.Triangles, origin.X, origin.Z, origin.Y)
	if !hit.Hit {
		return GroundHit{}
	}

	dist := origin.Y.Sub(hit.Point.Y)
	if dist.Cmp(fixed.Zero) < 0 || dist.Cmp(maxDist) > 0 {
		return GroundHit{}
	}

	return GroundHit{
		Hit:      true,
		Point:    hit.Point,
		Normal:   hit.Normal,
		Distance: dist,
	}
}

func sampleGroundAtXZ(tris []geom.Triangle, x, z, maxY fixed.Fixed) GroundHit {
	bestY := fixed.FromInt(-1000000)
	var bestNormal geom.Vec3
	found := false

	for i := range tris {
		t := tris[i]
		if pointInTriangleXZ(x, z, t.A, t.B, t.C) {
			y := solvePlaneY(t, x, z)
			if y.Cmp(maxY.Add(fixed.One)) <= 0 && y.Cmp(bestY) > 0 {
				bestY = y
				bestNormal = t.Normal()
				found = true
			}
		}
	}

	if !found {
		return GroundHit{}
	}

	return GroundHit{
		Hit:    true,
		Point:  geom.V3(x, bestY, z),
		Normal: bestNormal,
	}
}

func solvePlaneY(t geom.Triangle, x, z fixed.Fixed) fixed.Fixed {
	n := t.Normal()
	if n.Y.Cmp(fixed.Zero) == 0 {
		return fixed.Zero
	}
	nx := n.X
	ny := n.Y
	nz := n.Z

	rhs := ny.Mul(t.A.Y).Sub(nx.Mul(x.Sub(t.A.X))).Sub(nz.Mul(z.Sub(t.A.Z)))
	return rhs.Div(ny)
}

func pointInTriangleXZ(px, pz fixed.Fixed, a, b, c geom.Vec3) bool {
	s1 := sign2DXZ(px, pz, a, b)
	s2 := sign2DXZ(px, pz, b, c)
	s3 := sign2DXZ(px, pz, c, a)

	hasNeg := (s1.Cmp(fixed.Zero) < 0) || (s2.Cmp(fixed.Zero) < 0) || (s3.Cmp(fixed.Zero) < 0)
	hasPos := (s1.Cmp(fixed.Zero) > 0) || (s2.Cmp(fixed.Zero) > 0) || (s3.Cmp(fixed.Zero) > 0)

	return !(hasNeg && hasPos)
}

func sign2DXZ(px, pz fixed.Fixed, a, b geom.Vec3) fixed.Fixed {
	return (a.X.Sub(px)).Mul(b.Z.Sub(a.Z)).Sub((a.Z.Sub(pz)).Mul(b.X.Sub(a.X)))
}
