package physics

import (
	"testing"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

func TestStepVehicleKinematicSettlesOnFloorWithoutRaycastGrounding(t *testing.T) {
	world := PhysicsWorld{
		config: WorldConfig{
			FixedDT:     1.0 / 20.0,
			WorldBounds: geom.NewAABB(-20, -20, 20, 20),
		},
		staticMesh: testFlatFloorMesh(),
		player: VehicleBody{
			Position: geom.Planar(0, 0),
			Height:   1.5,
			Params:   DefaultVehicleParams(),
		},
	}

	for tick := 0; tick < 30; tick++ {
		world.Step(DriveInput{})
	}

	if world.player.SupportState != SupportStateStable {
		t.Fatalf("expected player to settle into grounded state, got %s", world.player.SupportState)
	}
	if world.player.Height < -0.01 || world.player.Height > 0.05 {
		t.Fatalf("expected grounded height near floor, got %f", world.player.Height)
	}
	if world.player.VerticalVel != 0 {
		t.Fatalf("expected grounded vertical velocity to be zero, got %f", world.player.VerticalVel)
	}
}

func TestStepVehicleKinematicSlidesAlongWall(t *testing.T) {
	world := PhysicsWorld{
		config: WorldConfig{
			FixedDT:     1.0 / 20.0,
			WorldBounds: geom.NewAABB(-20, -20, 20, 20),
		},
		staticMesh: combineStaticMeshes(testFlatFloorMesh(), testWallMesh()),
		player: VehicleBody{
			Position:     geom.Planar(-4.5, 0),
			Height:       0,
			Heading:      0.35,
			SupportState: SupportStateStable,
			SupportHits:  1,
			Params:       DefaultVehicleParams(),
		},
	}

	for tick := 0; tick < 30; tick++ {
		world.Step(DriveInput{Throttle: 1})
	}

	if world.player.Position.X > 1.5 {
		t.Fatalf("expected wall to block forward motion, got position %+v", world.player.Position)
	}
	if world.player.Position.Z <= 0.5 {
		t.Fatalf("expected tangential motion to keep advancing along wall, got position %+v", world.player.Position)
	}
	if world.bodyCapsuleIntersectsMap(world.player) {
		t.Fatal("expected kinematic step to keep capsule outside the wall")
	}
}

func combineStaticMeshes(meshes ...worldmesh.StaticMesh) worldmesh.StaticMesh {
	combined := worldmesh.StaticMesh{}
	for _, mesh := range meshes {
		combined.Triangles = append(combined.Triangles, mesh.Triangles...)
	}
	return combined
}
