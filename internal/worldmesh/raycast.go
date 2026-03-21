package worldmesh

import "server2/pkg/geom"

const raycastEpsilon float32 = 0.00001

// RaycastHit คือผลลัพธ์ของการยิง ray หา triangle ใต้รถ
type RaycastHit struct {
	Hit      bool
	Distance float32
	Point    geom.Vec3
	Normal   geom.Vec3
}

// RaycastDown ยิง ray แนวดิ่งลงเพื่อถามว่าที่ตำแหน่งนี้พื้น mesh อยู่ตรงไหน
func (mesh StaticMesh) RaycastDown(origin geom.Vec3, maxDistance float32) RaycastHit {
	if len(mesh.Triangles) == 0 || maxDistance <= 0 {
		return RaycastHit{}
	}

	closest := RaycastHit{
		Distance: maxDistance,
	}

	for _, triangle := range mesh.Triangles {
		if !triangleCouldHitDownRay(origin, maxDistance, triangle) {
			continue
		}

		hit, distance := intersectDownRayTriangle(origin, maxDistance, triangle)
		if !hit || distance > closest.Distance {
			continue
		}

		point := geom.V3(origin.X, origin.Y-distance, origin.Z)
		normal := triangle.B.Sub(triangle.A).Cross(triangle.C.Sub(triangle.A)).Normalize()
		if normal.Y < 0 {
			normal = normal.MulScalar(-1)
		}

		closest.Hit = true
		closest.Distance = distance
		closest.Point = point
		closest.Normal = normal
	}

	return closest
}

func triangleCouldHitDownRay(origin geom.Vec3, maxDistance float32, triangle Triangle) bool {
	minX := triangle.A.X
	maxX := triangle.A.X
	minY := triangle.A.Y
	maxY := triangle.A.Y
	minZ := triangle.A.Z
	maxZ := triangle.A.Z

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

	if origin.X < minX || origin.X > maxX || origin.Z < minZ || origin.Z > maxZ {
		return false
	}
	if origin.Y < minY || origin.Y-maxDistance > maxY {
		return false
	}
	return true
}

func intersectDownRayTriangle(origin geom.Vec3, maxDistance float32, triangle Triangle) (bool, float32) {
	direction := geom.V3(0, -1, 0)
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
