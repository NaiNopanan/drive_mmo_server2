package worldmesh

import "server2/pkg/geom"

const raycastEpsilon float32 = 0.00001

// RaycastHit คือผลลัพธ์ของการยิง ray หา triangle ใน static mesh
type RaycastHit struct {
	Hit      bool
	Distance float32
	Point    geom.Vec3
	Normal   geom.Vec3
}

// Raycast ยิง ray ตามทิศ direction ไปชน triangle แรกใน static mesh ภายใน maxDistance
func (mesh StaticMesh) Raycast(origin, direction geom.Vec3, maxDistance float32) RaycastHit {
	if len(mesh.Triangles) == 0 || maxDistance <= 0 || direction.LengthSquared() <= raycastEpsilon {
		return RaycastHit{}
	}

	direction = direction.Normalize()
	end := origin.Add(direction.MulScalar(maxDistance))
	minBounds, maxBounds := segmentBounds(origin, end)
	closest := RaycastHit{
		Distance: maxDistance,
	}

	for _, triangle := range mesh.Triangles {
		if !triangleCouldHitSegment(minBounds, maxBounds, triangle) {
			continue
		}

		hit, distance := intersectRayTriangle(origin, direction, maxDistance, triangle)
		if !hit || distance > closest.Distance {
			continue
		}

		normal := triangle.B.Sub(triangle.A).Cross(triangle.C.Sub(triangle.A)).Normalize()
		if normal.Dot(direction) > 0 {
			normal = normal.MulScalar(-1)
		}

		closest.Hit = true
		closest.Distance = distance
		closest.Point = origin.Add(direction.MulScalar(distance))
		closest.Normal = normal
	}

	return closest
}

// RaycastDown ยิง ray แนวดิ่งลงเพื่อถามว่าที่ตำแหน่งนี้พื้น mesh อยู่ตรงไหน
func (mesh StaticMesh) RaycastDown(origin geom.Vec3, maxDistance float32) RaycastHit {
	return mesh.Raycast(origin, geom.V3(0, -1, 0), maxDistance)
}

func segmentBounds(a, b geom.Vec3) (geom.Vec3, geom.Vec3) {
	minBounds := geom.V3(a.X, a.Y, a.Z)
	maxBounds := geom.V3(a.X, a.Y, a.Z)

	if b.X < minBounds.X {
		minBounds.X = b.X
	}
	if b.Y < minBounds.Y {
		minBounds.Y = b.Y
	}
	if b.Z < minBounds.Z {
		minBounds.Z = b.Z
	}
	if b.X > maxBounds.X {
		maxBounds.X = b.X
	}
	if b.Y > maxBounds.Y {
		maxBounds.Y = b.Y
	}
	if b.Z > maxBounds.Z {
		maxBounds.Z = b.Z
	}

	return minBounds, maxBounds
}

func triangleCouldHitSegment(minBounds, maxBounds geom.Vec3, triangle Triangle) bool {
	triangleMinX, triangleMaxX := triangle.A.X, triangle.A.X
	triangleMinY, triangleMaxY := triangle.A.Y, triangle.A.Y
	triangleMinZ, triangleMaxZ := triangle.A.Z, triangle.A.Z
	points := [2]geom.Vec3{triangle.B, triangle.C}
	for _, point := range points {
		if point.X < triangleMinX {
			triangleMinX = point.X
		}
		if point.X > triangleMaxX {
			triangleMaxX = point.X
		}
		if point.Y < triangleMinY {
			triangleMinY = point.Y
		}
		if point.Y > triangleMaxY {
			triangleMaxY = point.Y
		}
		if point.Z < triangleMinZ {
			triangleMinZ = point.Z
		}
		if point.Z > triangleMaxZ {
			triangleMaxZ = point.Z
		}
	}

	return !(triangleMaxX < minBounds.X || triangleMinX > maxBounds.X ||
		triangleMaxY < minBounds.Y || triangleMinY > maxBounds.Y ||
		triangleMaxZ < minBounds.Z || triangleMinZ > maxBounds.Z)
}

func intersectRayTriangle(origin, direction geom.Vec3, maxDistance float32, triangle Triangle) (bool, float32) {
	edgeAB := triangle.B.Sub(triangle.A)
	edgeAC := triangle.C.Sub(triangle.A)
	pVec := direction.Cross(edgeAC)
	determinant := edgeAB.Dot(pVec)
	if determinant > -raycastEpsilon && determinant < raycastEpsilon {
		return false, 0
	}

	invDeterminant := 1 / determinant
	tVec := origin.Sub(triangle.A)
	u := tVec.Dot(pVec) * invDeterminant
	if u < 0 || u > 1 {
		return false, 0
	}

	qVec := tVec.Cross(edgeAB)
	v := direction.Dot(qVec) * invDeterminant
	if v < 0 || u+v > 1 {
		return false, 0
	}

	distance := edgeAC.Dot(qVec) * invDeterminant
	if distance < 0 || distance > maxDistance {
		return false, 0
	}

	return true, distance
}
