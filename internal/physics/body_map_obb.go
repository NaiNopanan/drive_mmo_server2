package physics

import (
	"math"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const obbTriangleEpsilon float32 = 0.000001

type bodyOBB struct {
	center      geom.Vec3
	axes        [3]geom.Vec3
	halfExtents [3]float32
}

type bodyMapHit struct {
	Normal      geom.Vec3
	Penetration float32
}

func (w *PhysicsWorld) bodyOBBIntersectsMap(vehicle VehicleBody) bool {
	_, hit := w.queryBodyOBBMapHit(vehicle)
	return hit
}

func (w *PhysicsWorld) queryBodyOBBMapHit(vehicle VehicleBody) (bodyMapHit, bool) {
	if len(w.staticMesh.Triangles) == 0 {
		return bodyMapHit{}, false
	}

	obb := bodyOBBFromVehicle(vehicle)
	minBounds, maxBounds := bodyOBBAABB(obb)
	bestHit := bodyMapHit{}
	hasHit := false
	for _, triangle := range w.staticMesh.Triangles {
		if !triangleCouldHitBodyOBB(minBounds, maxBounds, triangle) {
			continue
		}
		hit, intersects := obbIntersectsTriangle(obb, triangle)
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

func bodyOBBFromVehicle(vehicle VehicleBody) bodyOBB {
	forward, right, up := vehicleAxes(vehicle.Heading, vehicle.Pitch, vehicle.Roll)
	halfHeight := vehicle.Params.BodyHeight * 0.5
	base := geom.V3(vehicle.Position.X, vehicle.Height, vehicle.Position.Z)

	return bodyOBB{
		center: base.Add(up.MulScalar(halfHeight)),
		axes: [3]geom.Vec3{
			forward.Normalize(),
			up.Normalize(),
			right.Normalize(),
		},
		halfExtents: [3]float32{
			vehicle.Params.BodyLength * 0.5,
			halfHeight,
			vehicle.Params.BodyWidth * 0.5,
		},
	}
}

func bodyOBBAABB(obb bodyOBB) (geom.Vec3, geom.Vec3) {
	extentX := absf(obb.axes[0].X)*obb.halfExtents[0] + absf(obb.axes[1].X)*obb.halfExtents[1] + absf(obb.axes[2].X)*obb.halfExtents[2]
	extentY := absf(obb.axes[0].Y)*obb.halfExtents[0] + absf(obb.axes[1].Y)*obb.halfExtents[1] + absf(obb.axes[2].Y)*obb.halfExtents[2]
	extentZ := absf(obb.axes[0].Z)*obb.halfExtents[0] + absf(obb.axes[1].Z)*obb.halfExtents[1] + absf(obb.axes[2].Z)*obb.halfExtents[2]

	min := geom.V3(obb.center.X-extentX, obb.center.Y-extentY, obb.center.Z-extentZ)
	max := geom.V3(obb.center.X+extentX, obb.center.Y+extentY, obb.center.Z+extentZ)
	return min, max
}

func triangleCouldHitBodyOBB(minBounds, maxBounds geom.Vec3, triangle worldmesh.Triangle) bool {
	minX, maxX := triangle.A.X, triangle.A.X
	minY, maxY := triangle.A.Y, triangle.A.Y
	minZ, maxZ := triangle.A.Z, triangle.A.Z
	points := [2]geom.Vec3{triangle.B, triangle.C}
	for _, point := range points {
		if point.X < minX {
			minX = point.X
		}
		if point.X > maxX {
			maxX = point.X
		}
		if point.Y < minY {
			minY = point.Y
		}
		if point.Y > maxY {
			maxY = point.Y
		}
		if point.Z < minZ {
			minZ = point.Z
		}
		if point.Z > maxZ {
			maxZ = point.Z
		}
	}

	return !(maxX < minBounds.X || minX > maxBounds.X ||
		maxY < minBounds.Y || minY > maxBounds.Y ||
		maxZ < minBounds.Z || minZ > maxBounds.Z)
}

func obbIntersectsTriangle(obb bodyOBB, triangle worldmesh.Triangle) (bodyMapHit, bool) {
	vertices := [3]geom.Vec3{
		toOBBLocalPoint(obb, triangle.A),
		toOBBLocalPoint(obb, triangle.B),
		toOBBLocalPoint(obb, triangle.C),
	}

	localHit, intersects := triangleBoxOverlap(vertices, obb.halfExtents)
	if !intersects {
		return bodyMapHit{}, false
	}

	worldNormal := obb.axes[0].MulScalar(localHit.Normal.X).
		Add(obb.axes[1].MulScalar(localHit.Normal.Y)).
		Add(obb.axes[2].MulScalar(localHit.Normal.Z)).
		Normalize()
	if worldNormal.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapHit{}, false
	}

	return bodyMapHit{
		Normal:      worldNormal,
		Penetration: localHit.Penetration,
	}, true
}

func toOBBLocalPoint(obb bodyOBB, point geom.Vec3) geom.Vec3 {
	delta := point.Sub(obb.center)
	return geom.V3(
		delta.Dot(obb.axes[0]),
		delta.Dot(obb.axes[1]),
		delta.Dot(obb.axes[2]),
	)
}

func triangleBoxOverlap(vertices [3]geom.Vec3, halfExtents [3]float32) (bodyMapHit, bool) {
	edges := [3]geom.Vec3{
		vertices[1].Sub(vertices[0]),
		vertices[2].Sub(vertices[1]),
		vertices[0].Sub(vertices[2]),
	}

	// Test the 9 axes from edge cross box-axis.
	boxAxes := [3]geom.Vec3{
		geom.V3(1, 0, 0),
		geom.V3(0, 1, 0),
		geom.V3(0, 0, 1),
	}
	for _, edge := range edges {
		for _, axis := range boxAxes {
			testAxis := edge.Cross(axis)
			if testAxis.LengthSquared() <= obbTriangleEpsilon {
				continue
			}
			_, intersects := overlapOnAxis(testAxis, vertices, halfExtents)
			if !intersects {
				return bodyMapHit{}, false
			}
		}
	}

	// Test the 3 box face axes.
	for axisIndex := 0; axisIndex < 3; axisIndex++ {
		axis := axisFromIndex(axisIndex)
		_, intersects := overlapOnAxis(axis, vertices, halfExtents)
		if !intersects {
			return bodyMapHit{}, false
		}
	}

	// Test the triangle plane.
	triangleNormal := edges[0].Cross(edges[1])
	if triangleNormal.LengthSquared() <= obbTriangleEpsilon {
		return bodyMapHit{}, false
	}
	_, intersects := overlapOnAxis(triangleNormal, vertices, halfExtents)
	if !intersects {
		return bodyMapHit{}, false
	}

	triangleNormal = triangleNormal.Normalize()
	triangleCenter := vertices[0].Add(vertices[1]).Add(vertices[2]).MulScalar(1.0 / 3.0)
	if triangleNormal.Dot(triangleCenter) > 0 {
		triangleNormal = triangleNormal.MulScalar(-1)
	}

	planeDistance := absf(triangleNormal.Dot(vertices[0]))
	planeRadius := halfExtents[0]*absf(triangleNormal.X) +
		halfExtents[1]*absf(triangleNormal.Y) +
		halfExtents[2]*absf(triangleNormal.Z)
	penetration := planeRadius - planeDistance
	if penetration <= obbTriangleEpsilon {
		return bodyMapHit{}, false
	}

	return bodyMapHit{
		Normal:      triangleNormal,
		Penetration: penetration,
	}, true
}

func overlapOnAxis(axis geom.Vec3, vertices [3]geom.Vec3, halfExtents [3]float32) (float32, bool) {
	axis = axis.Normalize()
	projections := [3]float32{
		vertices[0].Dot(axis),
		vertices[1].Dot(axis),
		vertices[2].Dot(axis),
	}

	minProjection := projections[0]
	maxProjection := projections[0]
	for i := 1; i < 3; i++ {
		if projections[i] < minProjection {
			minProjection = projections[i]
		}
		if projections[i] > maxProjection {
			maxProjection = projections[i]
		}
	}

	radius := halfExtents[0]*absf(axis.X) + halfExtents[1]*absf(axis.Y) + halfExtents[2]*absf(axis.Z)
	if minProjection > radius || maxProjection < -radius {
		return 0, false
	}

	return minf(maxProjection, radius) - maxf(minProjection, -radius), true
}

func axisFromIndex(axisIndex int) geom.Vec3 {
	switch axisIndex {
	case 0:
		return geom.V3(1, 0, 0)
	case 1:
		return geom.V3(0, 1, 0)
	default:
		return geom.V3(0, 0, 1)
	}
}

func component(value geom.Vec3, axisIndex int) float32 {
	switch axisIndex {
	case 0:
		return value.X
	case 1:
		return value.Y
	default:
		return value.Z
	}
}

func absf(value float32) float32 {
	return float32(math.Abs(float64(value)))
}

func minf(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func maxf(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
}
