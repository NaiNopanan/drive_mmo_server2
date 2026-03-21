package worldmesh

import (
	"fmt"
	"unsafe"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/pkg/geom"
)

// Triangle คือ primitive พื้นฐานของ world mesh ที่ server จะเก็บไว้ใช้ต่อยอด physics
type Triangle struct {
	A geom.Vec3
	B geom.Vec3
	C geom.Vec3
}

// StaticMesh คือข้อมูลโลกแบบ immutable หลัง import เสร็จ
type StaticMesh struct {
	SourcePath string
	Min        geom.Vec3
	Max        geom.Vec3
	Triangles  []Triangle
}

// LoadGLB โหลดไฟล์ world แล้วแตกออกมาเป็น triangle list จาก model mesh ทันที
func LoadGLB(path string) (StaticMesh, error) {
	model := rl.LoadModel(path)
	if !rl.IsModelValid(model) {
		return StaticMesh{}, fmt.Errorf("load world mesh %q failed", path)
	}
	defer rl.UnloadModel(model)

	meshes := model.GetMeshes()
	triangles := make([]Triangle, 0)
	min := geom.Vec3{X: 0, Y: 0, Z: 0}
	max := geom.Vec3{X: 0, Y: 0, Z: 0}
	hasBounds := false

	for _, mesh := range meshes {
		meshTriangles, err := extractTriangles(mesh, model.Transform)
		if err != nil {
			return StaticMesh{}, fmt.Errorf("extract triangles from %q: %w", path, err)
		}
		triangles = append(triangles, meshTriangles...)
		for _, triangle := range meshTriangles {
			min, max, hasBounds = extendBounds(min, max, hasBounds, triangle.A)
			min, max, hasBounds = extendBounds(min, max, hasBounds, triangle.B)
			min, max, hasBounds = extendBounds(min, max, hasBounds, triangle.C)
		}
	}

	return StaticMesh{
		SourcePath: path,
		Min:        min,
		Max:        max,
		Triangles:  triangles,
	}, nil
}

func extractTriangles(mesh rl.Mesh, transform rl.Matrix) ([]Triangle, error) {
	if mesh.VertexCount == 0 || mesh.Vertices == nil {
		return nil, nil
	}

	rawVertices := unsafe.Slice(mesh.Vertices, mesh.VertexCount*3)
	triangles := make([]Triangle, 0, mesh.TriangleCount)

	if mesh.Indices != nil {
		rawIndices := unsafe.Slice(mesh.Indices, mesh.TriangleCount*3)
		for index := 0; index+2 < len(rawIndices); index += 3 {
			a, err := readVertex(rawVertices, int(rawIndices[index]), transform)
			if err != nil {
				return nil, err
			}
			b, err := readVertex(rawVertices, int(rawIndices[index+1]), transform)
			if err != nil {
				return nil, err
			}
			c, err := readVertex(rawVertices, int(rawIndices[index+2]), transform)
			if err != nil {
				return nil, err
			}

			triangles = append(triangles, Triangle{A: a, B: b, C: c})
		}

		return triangles, nil
	}

	for vertex := 0; vertex+2 < int(mesh.VertexCount); vertex += 3 {
		a, err := readVertex(rawVertices, vertex, transform)
		if err != nil {
			return nil, err
		}
		b, err := readVertex(rawVertices, vertex+1, transform)
		if err != nil {
			return nil, err
		}
		c, err := readVertex(rawVertices, vertex+2, transform)
		if err != nil {
			return nil, err
		}

		triangles = append(triangles, Triangle{A: a, B: b, C: c})
	}

	return triangles, nil
}

func readVertex(rawVertices []float32, vertexIndex int, transform rl.Matrix) (geom.Vec3, error) {
	base := vertexIndex * 3
	if base+2 >= len(rawVertices) {
		return geom.Vec3{}, fmt.Errorf("vertex index %d out of range", vertexIndex)
	}

	source := rl.NewVector3(
		rawVertices[base],
		rawVertices[base+1],
		rawVertices[base+2],
	)
	transformed := rl.Vector3Transform(source, transform)

	return geom.Vec3{
		X: transformed.X,
		Y: transformed.Y,
		Z: transformed.Z,
	}, nil
}

func extendBounds(min, max geom.Vec3, hasBounds bool, point geom.Vec3) (geom.Vec3, geom.Vec3, bool) {
	if !hasBounds {
		return point, point, true
	}

	if point.X < min.X {
		min.X = point.X
	}
	if point.Y < min.Y {
		min.Y = point.Y
	}
	if point.Z < min.Z {
		min.Z = point.Z
	}
	if point.X > max.X {
		max.X = point.X
	}
	if point.Y > max.Y {
		max.Y = point.Y
	}
	if point.Z > max.Z {
		max.Z = point.Z
	}

	return min, max, true
}
