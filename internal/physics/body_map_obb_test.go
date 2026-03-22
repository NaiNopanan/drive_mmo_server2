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

func TestQueryBodyOBBMapHitReturnsNormalAndPenetration(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testWallMesh(),
	}
	vehicle := VehicleBody{
		Position: geom.Planar(0, 0),
		Height:   0,
		Params:   DefaultVehicleParams(),
	}

	hit, intersects := world.queryBodyOBBMapHit(vehicle)
	if !intersects {
		t.Fatal("expected body OBB query to hit wall mesh")
	}
	if hit.Penetration <= 0 {
		t.Fatalf("expected positive penetration, got %f", hit.Penetration)
	}
	if hit.Normal.X >= -0.9 {
		t.Fatalf("expected hit normal to push body away from wall on -X, got %+v", hit.Normal)
	}
	if absf(hit.Normal.Y) > 0.1 || absf(hit.Normal.Z) > 0.1 {
		t.Fatalf("expected wall normal to be mostly horizontal, got %+v", hit.Normal)
	}
}

func TestQueryBodyOBBMapCCDHitsForwardWall(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testWallMesh(),
	}
	params := DefaultVehicleParams()
	previous := VehicleBody{
		Position: geom.Planar(-5, 0),
		Height:   0,
		Params:   params,
	}
	current := previous
	current.Position = geom.Planar(2, 0)

	hit, intersects := world.queryBodyOBBMapCCD(previous, current)
	if !intersects {
		t.Fatal("expected swept OBB CCD to hit wall")
	}
	if hit.Time <= 0 || hit.Time >= 1 {
		t.Fatalf("expected CCD time in (0,1), got %f", hit.Time)
	}
	if hit.Normal.X >= -0.9 {
		t.Fatalf("expected CCD normal to oppose forward movement on -X, got %+v", hit.Normal)
	}

	current.Position = geom.LerpPlanar(previous.Position, current.Position, hit.Time)
	current.Height = lerpFloat32(previous.Height, current.Height, hit.Time)
	current.Position.X += hit.Normal.X * bodyOBBCCDSkin
	current.Position.Z += hit.Normal.Z * bodyOBBCCDSkin
	current.Height += hit.Normal.Y * bodyOBBCCDSkin
	if world.bodyOBBIntersectsMap(current) {
		t.Fatal("expected CCD contact pose to stay out of wall")
	}
}

func TestQueryBodyOBBMapCCDHitsSideWall(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testSideWallMesh(),
	}
	params := DefaultVehicleParams()
	previous := VehicleBody{
		Position: geom.Planar(0, 0),
		Height:   0,
		Params:   params,
	}
	current := previous
	current.Position = geom.Planar(0, 3)

	hit, intersects := world.queryBodyOBBMapCCD(previous, current)
	if !intersects {
		t.Fatal("expected swept OBB CCD to hit side wall")
	}
	if hit.Time <= 0 || hit.Time >= 1 {
		t.Fatalf("expected CCD time in (0,1), got %f", hit.Time)
	}
	if hit.Normal.Z >= -0.9 {
		t.Fatalf("expected CCD normal to oppose side movement on -Z, got %+v", hit.Normal)
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

func testSideWallMesh() worldmesh.StaticMesh {
	return worldmesh.StaticMesh{
		Triangles: []worldmesh.Triangle{
			{
				A: geom.V3(-3, 0, 1.4),
				B: geom.V3(-3, 2.5, 1.4),
				C: geom.V3(3, 0, 1.4),
			},
			{
				A: geom.V3(-3, 2.5, 1.4),
				B: geom.V3(3, 2.5, 1.4),
				C: geom.V3(3, 0, 1.4),
			},
		},
	}
}
