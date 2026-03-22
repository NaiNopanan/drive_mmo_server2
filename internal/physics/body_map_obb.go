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

func (w *PhysicsWorld) bodyOBBIntersectsMap(vehicle VehicleBody) bool {
	if len(w.staticMesh.Triangles) == 0 {
		return false
	}

	obb := bodyOBBFromVehicle(vehicle)
	minBounds, maxBounds := bodyOBBAABB(obb)
	for _, triangle := range w.staticMesh.Triangles {
		if !triangleCouldHitBodyOBB(minBounds, maxBounds, triangle) {
			continue
		}
		if obbIntersectsTriangle(obb, triangle) {
			return true
		}
	}

	return false
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

func obbIntersectsTriangle(obb bodyOBB, triangle worldmesh.Triangle) bool {
	vertices := [3]geom.Vec3{
		toOBBLocalPoint(obb, triangle.A),
		toOBBLocalPoint(obb, triangle.B),
		toOBBLocalPoint(obb, triangle.C),
	}

	return triangleBoxOverlap(vertices, obb.halfExtents)
}

func toOBBLocalPoint(obb bodyOBB, point geom.Vec3) geom.Vec3 {
	delta := point.Sub(obb.center)
	return geom.V3(
		delta.Dot(obb.axes[0]),
		delta.Dot(obb.axes[1]),
		delta.Dot(obb.axes[2]),
	)
}

func triangleBoxOverlap(vertices [3]geom.Vec3, halfExtents [3]float32) bool {
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
			if separatedOnAxis(testAxis, vertices, halfExtents) {
				return false
			}
		}
	}

	// Test the 3 box face axes.
	for axisIndex := 0; axisIndex < 3; axisIndex++ {
		minValue := component(vertices[0], axisIndex)
		maxValue := minValue
		for i := 1; i < 3; i++ {
			value := component(vertices[i], axisIndex)
			if value < minValue {
				minValue = value
			}
			if value > maxValue {
				maxValue = value
			}
		}
		if minValue > halfExtents[axisIndex] || maxValue < -halfExtents[axisIndex] {
			return false
		}
	}

	// Test the triangle plane.
	triangleNormal := edges[0].Cross(edges[1])
	if triangleNormal.LengthSquared() <= obbTriangleEpsilon {
		return false
	}
	if separatedOnAxis(triangleNormal, vertices, halfExtents) {
		return false
	}

	return true
}

func separatedOnAxis(axis geom.Vec3, vertices [3]geom.Vec3, halfExtents [3]float32) bool {
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
	return minProjection > radius || maxProjection < -radius
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
