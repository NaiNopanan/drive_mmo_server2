package physics

import (
	"testing"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

func TestBodyOBBIntersectsMapHit(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testWallMesh(),
	}
	vehicle := VehicleBody{
		Position: geom.Planar(0, 0),
		Height:   0,
		Params:   DefaultVehicleParams(),
	}

	if !world.bodyOBBIntersectsMap(vehicle) {
		t.Fatal("expected body OBB to intersect wall mesh")
	}
}

func TestBodyOBBIntersectsMapMiss(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testWallMesh(),
	}
	vehicle := VehicleBody{
		Position: geom.Planar(-6, 0),
		Height:   0,
		Params:   DefaultVehicleParams(),
	}

	if world.bodyOBBIntersectsMap(vehicle) {
		t.Fatal("expected body OBB to miss wall mesh")
	}
}

func testWallMesh() worldmesh.StaticMesh {
	return worldmesh.StaticMesh{
		Triangles: []worldmesh.Triangle{
			{
				A: geom.V3(1.8, 0, -3),
				B: geom.V3(1.8, 2.5, -3),
				C: geom.V3(1.8, 0, 3),
			},
			{
				A: geom.V3(1.8, 2.5, -3),
				B: geom.V3(1.8, 2.5, 3),
				C: geom.V3(1.8, 0, 3),
			},
		},
	}
}
