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

// FlatGround implements GroundQuery for a simple horizontal plane.
type FlatGround struct {
	Y                      fixed.Fixed
	MinX, MaxX, MinZ, MaxZ fixed.Fixed
}

func (g FlatGround) Raycast(origin geom.Vec3, dir geom.Vec3, maxDist fixed.Fixed) GroundHit {
	// Only support downward raycasts for suspension/CCD
	if dir.Y.Cmp(fixed.Zero) >= 0 {
		return GroundHit{}
	}

	if origin.Y.Cmp(g.Y) < 0 {
		return GroundHit{}
	}

	// Boundary check (exclude if all zero)
	if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
		if origin.X.Cmp(g.MinX) < 0 || origin.X.Cmp(g.MaxX) > 0 || origin.Z.Cmp(g.MinZ) < 0 || origin.Z.Cmp(g.MaxZ) > 0 {
			return GroundHit{}
		}
	}

	dist := origin.Y.Sub(g.Y)
	if dist.Cmp(maxDist) > 0 {
		return GroundHit{}
	}

	return GroundHit{
		Hit:      true,
		Point:    geom.V3(origin.X, g.Y, origin.Z),
		Normal:   geom.V3(fixed.Zero, fixed.One, fixed.Zero),
		Distance: dist,
	}
}

// SlopeGround implements GroundQuery for an infinite slope along Z axis.
type SlopeGround struct {
	BaseY                  fixed.Fixed
	Slope                  fixed.Fixed // dy/dz
	MinX, MaxX, MinZ, MaxZ fixed.Fixed
}

func (g SlopeGround) Raycast(origin geom.Vec3, dir geom.Vec3, maxDist fixed.Fixed) GroundHit {
	// Only support downward raycasts
	if dir.Y.Cmp(fixed.Zero) >= 0 {
		return GroundHit{}
	}

	// Boundary check
	if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
		if origin.X.Cmp(g.MinX) < 0 || origin.X.Cmp(g.MaxX) > 0 || origin.Z.Cmp(g.MinZ) < 0 || origin.Z.Cmp(g.MaxZ) > 0 {
			return GroundHit{}
		}
	}

	// y = BaseY + Slope * z
	groundY := g.BaseY.Add(g.Slope.Mul(origin.Z))
	if origin.Y.Cmp(groundY) < 0 {
		return GroundHit{}
	}

	dist := origin.Y.Sub(groundY)
	if dist.Cmp(maxDist) > 0 {
		return GroundHit{}
	}

	// Normal calculation:
	// Plane eqn: y - Slope*z - BaseY = 0
	// Vector (0, 1, -Slope)
	n := geom.V3(fixed.Zero, fixed.One, g.Slope.Neg()).Normalize()

	return GroundHit{
		Hit:      true,
		Point:    geom.V3(origin.X, groundY, origin.Z),
		Normal:   n,
		Distance: dist,
	}
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
	const ceilingEpsilonRaw = 1 << 24

	bestY := fixed.FromInt(-1000000)
	var bestNormal geom.Vec3
	found := false
	ceiling := maxY.Add(fixed.FromRaw(ceilingEpsilonRaw))

	for i := range tris {
		t := tris[i]
		if pointInTriangleXZ(x, z, t.A, t.B, t.C) {
			y := solvePlaneY(t, x, z)
			if y.Cmp(ceiling) <= 0 && y.Cmp(bestY) > 0 {
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
