package physics

import (
	"math"
	"testing"

	"server2/pkg/geom"
)

func TestQueryBodyCapsuleMapHitReturnsNormalAndPenetration(t *testing.T) {
	world := PhysicsWorld{
		staticMesh: testWallMesh(),
	}
	vehicle := VehicleBody{
		Position: geom.Planar(0, 0),
		Height:   0,
		Params:   DefaultVehicleParams(),
	}

	hit, intersects := world.queryBodyCapsuleMapHit(vehicle)
	if !intersects {
		t.Fatal("expected body capsule query to hit wall mesh")
	}
	if hit.Penetration <= 0 {
		t.Fatalf("expected positive penetration, got %f", hit.Penetration)
	}
	if hit.Normal.X >= -0.7 {
		t.Fatalf("expected capsule hit normal to push body away from wall on -X, got %+v", hit.Normal)
	}
}

func TestQueryBodyCapsuleMapCCDHitsForwardWall(t *testing.T) {
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

	hit, intersects := world.queryBodyCapsuleMapCCD(previous, current)
	if !intersects {
		t.Fatal("expected swept capsule CCD to hit wall")
	}
	if hit.Time <= 0 || hit.Time >= 1 {
		t.Fatalf("expected capsule CCD time in (0,1), got %f", hit.Time)
	}
	if hit.Normal.X >= -0.7 {
		t.Fatalf("expected capsule CCD normal to oppose forward movement on -X, got %+v", hit.Normal)
	}
}

func TestApplyBodyCapsuleCCDWithSlideMovesAlongWall(t *testing.T) {
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

	resolved := world.applyBodyCapsuleCCDWithSlide(previous, current, 1)
	if !resolved.BodyHitMap {
		t.Fatal("expected capsule CCD+slide to register map hit")
	}
	if resolved.Position.Z <= 0.5 {
		t.Fatalf("expected remaining-time solve to advance along wall, got position %+v", resolved.Position)
	}
	if world.bodyCapsuleIntersectsMap(resolved) {
		t.Fatal("expected resolved vehicle to stay out of wall")
	}
}

func TestBodyCapsuleFromVehicleUsesPitchAndKeepsBottomHeight(t *testing.T) {
	params := DefaultVehicleParams()
	vehicle := VehicleBody{
		Position: geom.Planar(0, 0),
		Height:   1.25,
		Heading:  0,
		Pitch:    0.35,
		Roll:     0.2,
		Params:   params,
	}

	capsule := bodyCapsuleFromVehicle(vehicle)
	if capsule.start.Y <= capsule.end.Y {
		t.Fatalf("expected pitched capsule nose to rise, got start=%f end=%f", capsule.start.Y, capsule.end.Y)
	}

	bottomY := minf(capsule.start.Y, capsule.end.Y) - capsule.radius
	if math.Abs(float64(bottomY-vehicle.Height)) > 0.0001 {
		t.Fatalf("expected capsule bottom to stay anchored at vehicle height, got bottom=%f height=%f", bottomY, vehicle.Height)
	}
}
