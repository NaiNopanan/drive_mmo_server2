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

func TestSlideVelocityAgainstNormalPreservesTangent(t *testing.T) {
	velocity := geom.V3(6, 0, 2)
	normal := geom.V3(-1, 0, 0)

	sliding := slideVelocityAgainstNormal(velocity, normal)
	if absf(sliding.X) > 0.001 {
		t.Fatalf("expected wall-normal speed to be removed, got %+v", sliding)
	}
	if sliding.Z < 1.99 || sliding.Z > 2.01 {
		t.Fatalf("expected tangent speed to be preserved, got %+v", sliding)
	}
}

func TestSlideVelocityAgainstNormalHandlesVerticalComponent(t *testing.T) {
	velocity := geom.V3(3, -4, 0)
	normal := geom.V3(0, 1, 0)

	sliding := slideVelocityAgainstNormal(velocity, normal)
	if absf(sliding.Y) > 0.001 {
		t.Fatalf("expected floor-normal speed to be removed, got %+v", sliding)
	}
	if sliding.X < 2.99 || sliding.X > 3.01 {
		t.Fatalf("expected tangent speed to be preserved, got %+v", sliding)
	}
}

func TestApplyBodyOBBCCDWithSlideMovesAlongWall(t *testing.T) {
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
	current.Position = geom.Planar(2, 2)
	current.Velocity = geom.Planar(7, 2)
	current.Speed = current.Velocity.Length()

	resolved := world.applyBodyOBBCCDWithSlide(previous, current, 1)
	if !resolved.BodyHitMap {
		t.Fatal("expected CCD+slide to register map hit")
	}
	if resolved.Position.Z <= 0.5 {
		t.Fatalf("expected remaining time solve to advance along wall, got position %+v", resolved.Position)
	}
	if world.bodyOBBIntersectsMap(resolved) {
		t.Fatal("expected resolved vehicle to stay out of wall")
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
