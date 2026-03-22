package physics

import (
	"math"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const bodyCapsuleEpsilon float32 = 0.000001

type bodyCapsule struct {
	start  geom.Vec3
	end    geom.Vec3
	radius float32
}

func bodyCapsuleFromVehicle(vehicle VehicleBody) bodyCapsule {
	forward, _, up := vehicleAxes(vehicle.Heading, vehicle.Pitch, vehicle.Roll)
	radius, halfSegment := bodyCapsuleDimensions(vehicle.Params)
	base := geom.V3(vehicle.Position.X, vehicle.Height, vehicle.Position.Z)
	center := base.Add(up.MulScalar(radius))
	return bodyCapsule{
		start:  center.Add(forward.MulScalar(halfSegment)),
		end:    center.Sub(forward.MulScalar(halfSegment)),
		radius: radius,
	}
}

func bodyCapsuleDimensions(params VehicleParams) (float32, float32) {
	radius := minf(params.BodyHeight*0.5, params.BodyWidth*0.35)
	halfSegment := maxf(0, params.BodyLength*0.5-radius)
	return radius, halfSegment
}

func (w *PhysicsWorld) bodyCapsuleIntersectsMap(vehicle VehicleBody) bool {
	_, hit := w.queryBodyCapsuleMapHit(vehicle)
	return hit
}

func (w *PhysicsWorld) queryBodyCapsuleMapHit(vehicle VehicleBody) (bodyMapHit, bool) {
	if len(w.staticMesh.Triangles) == 0 {
		return bodyMapHit{}, false
	}

	capsule := bodyCapsuleFromVehicle(vehicle)
	minBounds, maxBounds := bodyCapsuleAABB(capsule)
	bestHit := bodyMapHit{}
	hasHit := false
	for _, triangle := range w.staticMesh.Triangles {
		if !triangleCouldHitBodyOBB(minBounds, maxBounds, triangle) {
			continue
		}
		hit, intersects := capsuleIntersectsTriangle(capsule, triangle)
		if !intersects {
			continue
		}
		if !hasHit || hit.Penetration > bestHit.Penetration {
			bestHit = hit
			hasHit = true
		}
	}

	return bestHit, hasHit
}

func bodyCapsuleAABB(capsule bodyCapsule) (geom.Vec3, geom.Vec3) {
	minBounds := geom.V3(
		minf(capsule.start.X, capsule.end.X)-capsule.radius,
		minf(capsule.start.Y, capsule.end.Y)-capsule.radius,
		minf(capsule.start.Z, capsule.end.Z)-capsule.radius,
	)
	maxBounds := geom.V3(
		maxf(capsule.start.X, capsule.end.X)+capsule.radius,
		maxf(capsule.start.Y, capsule.end.Y)+capsule.radius,
		maxf(capsule.start.Z, capsule.end.Z)+capsule.radius,
	)
	return minBounds, maxBounds
}

func capsuleIntersectsTriangle(capsule bodyCapsule, triangle worldmesh.Triangle) (bodyMapHit, bool) {
	capsulePoint, trianglePoint := closestPointsSegmentTriangle(capsule.start, capsule.end, triangle.A, triangle.B, triangle.C)
	separation := capsulePoint.Sub(trianglePoint)
	distanceSquared := separation.LengthSquared()
	radiusSquared := capsule.radius * capsule.radius
	if distanceSquared >= radiusSquared {
		return bodyMapHit{}, false
	}

	distance := float32(math.Sqrt(float64(distanceSquared)))
	normal := separation.Normalize()
	if distance <= bodyCapsuleEpsilon || normal.LengthSquared() <= bodyCapsuleEpsilon {
		normal = triangleNormalTowardPoint(triangle, capsulePoint)
	}
	if normal.LengthSquared() <= bodyCapsuleEpsilon {
		return bodyMapHit{}, false
	}

	return bodyMapHit{
		Normal:      normal.Normalize(),
		Penetration: capsule.radius - distance,
	}, true
}

func closestPointsSegmentTriangle(segA, segB, triA, triB, triC geom.Vec3) (geom.Vec3, geom.Vec3) {
	bestSeg := segA
	bestTri := closestPointOnTriangle(segA, triA, triB, triC)
	bestDistanceSquared := bestSeg.Sub(bestTri).LengthSquared()

	updateBest := func(segPoint, triPoint geom.Vec3) {
		distanceSquared := segPoint.Sub(triPoint).LengthSquared()
		if distanceSquared < bestDistanceSquared {
			bestDistanceSquared = distanceSquared
			bestSeg = segPoint
			bestTri = triPoint
		}
	}

	updateBest(segB, closestPointOnTriangle(segB, triA, triB, triC))

	triangleNormal := triB.Sub(triA).Cross(triC.Sub(triA))
	if triangleNormal.LengthSquared() > bodyCapsuleEpsilon {
		triangleNormal = triangleNormal.Normalize()
		segDirection := segB.Sub(segA)
		denominator := triangleNormal.Dot(segDirection)
		var planeT float32
		if absf(denominator) > bodyCapsuleEpsilon {
			planeT = clamp(triangleNormal.Dot(triA.Sub(segA))/denominator, 0, 1)
		} else {
			distanceA := absf(triangleNormal.Dot(segA.Sub(triA)))
			distanceB := absf(triangleNormal.Dot(segB.Sub(triA)))
			if distanceA <= distanceB {
				planeT = 0
			} else {
				planeT = 1
			}
		}
		planePoint := segA.Add(segDirection.MulScalar(planeT))
		updateBest(planePoint, closestPointOnTriangle(planePoint, triA, triB, triC))
	}

	edges := [][2]geom.Vec3{
		{triA, triB},
		{triB, triC},
		{triC, triA},
	}
	for _, edge := range edges {
		segPoint, edgePoint := closestPointsSegmentSegment(segA, segB, edge[0], edge[1])
		updateBest(segPoint, edgePoint)
	}

	return bestSeg, bestTri
}

func closestPointOnTriangle(point, a, b, c geom.Vec3) geom.Vec3 {
	ab := b.Sub(a)
	ac := c.Sub(a)
	ap := point.Sub(a)
	d1 := ab.Dot(ap)
	d2 := ac.Dot(ap)
	if d1 <= 0 && d2 <= 0 {
		return a
	}

	bp := point.Sub(b)
	d3 := ab.Dot(bp)
	d4 := ac.Dot(bp)
	if d3 >= 0 && d4 <= d3 {
		return b
	}

	vc := d1*d4 - d3*d2
	if vc <= 0 && d1 >= 0 && d3 <= 0 {
		v := d1 / (d1 - d3)
		return a.Add(ab.MulScalar(v))
	}

	cp := point.Sub(c)
	d5 := ab.Dot(cp)
	d6 := ac.Dot(cp)
	if d6 >= 0 && d5 <= d6 {
		return c
	}

	vb := d5*d2 - d1*d6
	if vb <= 0 && d2 >= 0 && d6 <= 0 {
		w := d2 / (d2 - d6)
		return a.Add(ac.MulScalar(w))
	}

	va := d3*d6 - d5*d4
	if va <= 0 && (d4-d3) >= 0 && (d5-d6) >= 0 {
		w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
		return b.Add(c.Sub(b).MulScalar(w))
	}

	denominator := 1 / (va + vb + vc)
	v := vb * denominator
	w := vc * denominator
	return a.Add(ab.MulScalar(v)).Add(ac.MulScalar(w))
}

func closestPointsSegmentSegment(a0, a1, b0, b1 geom.Vec3) (geom.Vec3, geom.Vec3) {
	const epsilon float32 = 0.000001

	d1 := a1.Sub(a0)
	d2 := b1.Sub(b0)
	r := a0.Sub(b0)
	a := d1.Dot(d1)
	e := d2.Dot(d2)
	f := d2.Dot(r)

	var s, t float32
	switch {
	case a <= epsilon && e <= epsilon:
		return a0, b0
	case a <= epsilon:
		s = 0
		t = clamp(f/e, 0, 1)
	case e <= epsilon:
		t = 0
		s = clamp(-d1.Dot(r)/a, 0, 1)
	default:
		c := d1.Dot(r)
		b := d1.Dot(d2)
		denominator := a*e - b*b
		if denominator != 0 {
			s = clamp((b*f-c*e)/denominator, 0, 1)
		} else {
			s = 0
		}

		t = (b*s + f) / e
		if t < 0 {
			t = 0
			s = clamp(-c/a, 0, 1)
		} else if t > 1 {
			t = 1
			s = clamp((b-c)/a, 0, 1)
		}
	}

	return a0.Add(d1.MulScalar(s)), b0.Add(d2.MulScalar(t))
}

func triangleNormalTowardPoint(triangle worldmesh.Triangle, point geom.Vec3) geom.Vec3 {
	normal := triangle.B.Sub(triangle.A).Cross(triangle.C.Sub(triangle.A)).Normalize()
	if normal.LengthSquared() <= bodyCapsuleEpsilon {
		return geom.Vec3{}
	}
	if normal.Dot(point.Sub(triangle.A)) < 0 {
		normal = normal.MulScalar(-1)
	}
	return normal
}
