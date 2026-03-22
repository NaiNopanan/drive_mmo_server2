package physics

import (
	"math"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const bodyOBBCCDSkin float32 = 0.01

type bodyMapCCDHit struct {
	Time   float32
	Normal geom.Vec3
}

func (w *PhysicsWorld) queryBodyOBBMapCCD(previous, current VehicleBody) (bodyMapCCDHit, bool) {
	if len(w.staticMesh.Triangles) == 0 {
		return bodyMapCCDHit{}, false
	}

	startHit, startIntersects := w.queryBodyOBBMapHit(previous)
	if startIntersects {
		return bodyMapCCDHit{
			Time:   0,
			Normal: startHit.Normal,
		}, true
	}

	startOBB := bodyOBBFromVehicle(previous)
	endOBB := bodyOBBFromVehicle(current)
	deltaWorld := endOBB.center.Sub(startOBB.center)
	if deltaWorld.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapCCDHit{}, false
	}

	startMin, startMax := bodyOBBAABB(startOBB)
	endMin, endMax := bodyOBBAABB(endOBB)
	sweptMin, sweptMax := mergeAABB(startMin, startMax, endMin, endMax)

	bestHit := bodyMapCCDHit{Time: float32(math.MaxFloat32)}
	hasHit := false
	for _, triangle := range w.staticMesh.Triangles {
		if !triangleCouldHitBodyOBB(sweptMin, sweptMax, triangle) {
			continue
		}

		hit, intersects := sweepOBBTriangle(startOBB, deltaWorld, triangle)
		if !intersects {
			continue
		}
		if !hasHit || hit.Time < bestHit.Time {
			bestHit = hit
			hasHit = true
		}
	}

	return bestHit, hasHit
}

func sweepOBBTriangle(obb bodyOBB, deltaWorld geom.Vec3, triangle worldmesh.Triangle) (bodyMapCCDHit, bool) {
	vertices := [3]geom.Vec3{
		toOBBLocalPoint(obb, triangle.A),
		toOBBLocalPoint(obb, triangle.B),
		toOBBLocalPoint(obb, triangle.C),
	}
	localDelta := geom.V3(
		deltaWorld.Dot(obb.axes[0]),
		deltaWorld.Dot(obb.axes[1]),
		deltaWorld.Dot(obb.axes[2]),
	)

	localHit, intersects := sweepTriangleBox(vertices, obb.halfExtents, localDelta)
	if !intersects {
		return bodyMapCCDHit{}, false
	}

	worldNormal := obb.axes[0].MulScalar(localHit.Normal.X).
		Add(obb.axes[1].MulScalar(localHit.Normal.Y)).
		Add(obb.axes[2].MulScalar(localHit.Normal.Z)).
		Normalize()
	if worldNormal.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapCCDHit{}, false
	}

	return bodyMapCCDHit{
		Time:   localHit.Time,
		Normal: worldNormal,
	}, true
}

func sweepTriangleBox(vertices [3]geom.Vec3, halfExtents [3]float32, delta geom.Vec3) (bodyMapCCDHit, bool) {
	edges := [3]geom.Vec3{
		vertices[1].Sub(vertices[0]),
		vertices[2].Sub(vertices[1]),
		vertices[0].Sub(vertices[2]),
	}

	entryTime := float32(0)
	exitTime := float32(1)
	impactAxis := geom.Vec3{}
	impactSpeed := float32(0)

	testAxis := func(axis geom.Vec3) bool {
		axisEntry, axisExit, speed, ok := sweepAxisInterval(axis, vertices, halfExtents, delta)
		if !ok {
			return false
		}
		if axisEntry > entryTime {
			entryTime = axisEntry
			impactAxis = axis.Normalize()
			impactSpeed = speed
		}
		if axisExit < exitTime {
			exitTime = axisExit
		}
		return entryTime <= exitTime
	}

	boxAxes := [3]geom.Vec3{
		geom.V3(1, 0, 0),
		geom.V3(0, 1, 0),
		geom.V3(0, 0, 1),
	}
	for _, edge := range edges {
		for _, axis := range boxAxes {
			testAxisVector := edge.Cross(axis)
			if testAxisVector.LengthSquared() <= obbTriangleEpsilon {
				continue
			}
			if !testAxis(testAxisVector) {
				return bodyMapCCDHit{}, false
			}
		}
	}

	for axisIndex := 0; axisIndex < 3; axisIndex++ {
		if !testAxis(axisFromIndex(axisIndex)) {
			return bodyMapCCDHit{}, false
		}
	}

	triangleNormal := edges[0].Cross(edges[1])
	if triangleNormal.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapCCDHit{}, false
	}
	if !testAxis(triangleNormal) {
		return bodyMapCCDHit{}, false
	}

	if entryTime < 0 || entryTime > 1 || exitTime < 0 {
		return bodyMapCCDHit{}, false
	}
	if impactAxis.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapCCDHit{}, false
	}

	impactNormal := impactAxis
	if impactSpeed > 0 {
		impactNormal = impactNormal.MulScalar(-1)
	}

	return bodyMapCCDHit{
		Time:   clamp(entryTime, 0, 1),
		Normal: impactNormal,
	}, true
}

func sweepAxisInterval(axis geom.Vec3, vertices [3]geom.Vec3, halfExtents [3]float32, delta geom.Vec3) (float32, float32, float32, bool) {
	axis = axis.Normalize()

	minProjection := vertices[0].Dot(axis)
	maxProjection := minProjection
	for index := 1; index < len(vertices); index++ {
		projection := vertices[index].Dot(axis)
		if projection < minProjection {
			minProjection = projection
		}
		if projection > maxProjection {
			maxProjection = projection
		}
	}

	radius := halfExtents[0]*absf(axis.X) + halfExtents[1]*absf(axis.Y) + halfExtents[2]*absf(axis.Z)
	speed := delta.Dot(axis)
	if absf(speed) <= obbTriangleEpsilon {
		if minProjection > radius || maxProjection < -radius {
			return 0, 0, 0, false
		}
		maxTime := float32(math.MaxFloat32)
		return -maxTime, maxTime, 0, true
	}

	entry := (minProjection - radius) / speed
	exit := (maxProjection + radius) / speed
	if entry > exit {
		entry, exit = exit, entry
	}

	return entry, exit, speed, true
}

func mergeAABB(minA, maxA, minB, maxB geom.Vec3) (geom.Vec3, geom.Vec3) {
	return geom.V3(
			minf(minA.X, minB.X),
			minf(minA.Y, minB.Y),
			minf(minA.Z, minB.Z),
		), geom.V3(
			maxf(maxA.X, maxB.X),
			maxf(maxA.Y, maxB.Y),
			maxf(maxA.Z, maxB.Z),
		)
}
