package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type TriangleContact struct {
	Hit           bool
	Point         geom.Vec3
	Normal        geom.Vec3
	Penetration   fixed.Fixed
	TriangleIndex int
}

func sphereTriangleContact(center geom.Vec3, radius fixed.Fixed, tri geom.Triangle, triIndex int) TriangleContact {
	cp := tri.ClosestPoint(center)
	delta := center.Sub(cp)

	distSq := delta.Dot(delta)
	radiusSq := radius.Mul(radius)

	if distSq.Cmp(radiusSq) > 0 {
		return TriangleContact{}
	}

	var normal geom.Vec3
	if distSq == fixed.Zero {
		normal = tri.Normal()
		// Ensure normal faces up (for ground triangles)
		if normal.Y.Cmp(fixed.Zero) < 0 {
			normal = normal.Neg()
		}
	} else {
		dist := distSq.Sqrt()
		if dist == fixed.Zero {
			normal = tri.Normal()
		} else {
			normal = delta.Scale(fixed.One.Div(dist))
		}
	}

	dist := fixed.Zero
	if distSq != fixed.Zero {
		dist = distSq.Sqrt()
	}

	penetration := radius.Sub(dist)
	if penetration.Cmp(fixed.Zero) < 0 {
		return TriangleContact{}
	}

	return TriangleContact{
		Hit:           true,
		Point:         cp,
		Normal:        normal,
		Penetration:   penetration,
		TriangleIndex: triIndex,
	}
}
