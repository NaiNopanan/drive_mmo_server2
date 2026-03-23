package physics

import (
	"testing"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

func TestStepVehicleKinematicStaysOnRoadUnderSlopeRoofAtSpeed(t *testing.T) {
	world := PhysicsWorld{
		config: WorldConfig{
			FixedDT:     1.0 / 20.0,
			WorldBounds: geom.NewAABB(-20, -20, 20, 20),
		},
		staticMesh: combineStaticMeshes(testLongFloorMesh(), testSlopeRoofMesh()),
		player: VehicleBody{
			Position:     geom.Planar(-4.5, 0),
			Height:       0,
			Heading:      0,
			SupportState: SupportStateStable,
			SupportHits:  1,
			GroundHeight: 0,
			Kinematic: KinematicDebug{
				Grounded:     true,
				GroundNormal: geom.V3(0, 1, 0),
			},
			Params: DefaultVehicleParams(),
		},
	}

	for tick := 0; tick < 40; tick++ {
		world.Step(DriveInput{Throttle: 1, Nitro: true})
	}

	if world.player.Height > 0.6 {
		t.Fatalf("expected player to stay on road under slope roof, got height %f", world.player.Height)
	}
	if world.player.Height < -0.25 {
		t.Fatalf("expected player not to be pushed below road under slope roof, got height %f", world.player.Height)
	}
}

func testSlopeRoofMesh() worldmesh.StaticMesh {
	return worldmesh.StaticMesh{
		Triangles: []worldmesh.Triangle{
			{
				A: geom.V3(-4, 1.0, -5),
				B: geom.V3(-4, 1.0, 5),
				C: geom.V3(4, 4.2, -5),
			},
			{
				A: geom.V3(-4, 1.0, 5),
				B: geom.V3(4, 4.2, 5),
				C: geom.V3(4, 4.2, -5),
			},
		},
	}
}

func testLongFloorMesh() worldmesh.StaticMesh {
	return worldmesh.StaticMesh{
		Triangles: []worldmesh.Triangle{
			{
				A: geom.V3(-40, 0, -10),
				B: geom.V3(40, 0, -10),
				C: geom.V3(-40, 0, 10),
			},
			{
				A: geom.V3(40, 0, -10),
				B: geom.V3(40, 0, 10),
				C: geom.V3(-40, 0, 10),
			},
		},
	}
}
