package worldmesh

import (
	"testing"

	"server2/pkg/geom"
)

func TestRaycastHitsVerticalWall(t *testing.T) {
	mesh := StaticMesh{
		Triangles: []Triangle{
			{
				A: geom.V3(2, 0, -2),
				B: geom.V3(2, 2, -2),
				C: geom.V3(2, 0, 2),
			},
			{
				A: geom.V3(2, 2, -2),
				B: geom.V3(2, 2, 2),
				C: geom.V3(2, 0, 2),
			},
		},
	}

	hit := mesh.Raycast(geom.V3(0, 1, 0), geom.V3(1, 0, 0), 5)
	if !hit.Hit {
		t.Fatal("expected raycast to hit wall")
	}
	if hit.Distance < 1.99 || hit.Distance > 2.01 {
		t.Fatalf("expected hit distance near 2, got %f", hit.Distance)
	}
	if hit.Normal.X >= -0.9 {
		t.Fatalf("expected wall normal to oppose ray direction, got %+v", hit.Normal)
	}
}
