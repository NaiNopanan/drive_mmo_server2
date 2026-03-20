package ecs

import (
	"testing"

	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	enginephysics "server2/internal/engine/physics"
	"server2/internal/engine/scene"
)

type recordSystem struct {
	steps int
}

func (r *recordSystem) Step(w *World, dt fixed.Fixed) {
	r.steps++
}

func TestWorldCreateAndDestroyEntity(t *testing.T) {
	world := NewWorld(WorldConfig{})
	id := world.NewEntity()
	if !world.HasEntity(id) {
		t.Fatalf("expected entity to exist after creation")
	}

	world.SetName(id, scene.NameComponent{Value: "crate"})
	world.SetTransform(id, scene.IdentityTransform())
	world.SetBody(id, enginephysics.NewStaticBox(geometry.ZeroVector3(), geometry.NewVector3(fixed.One, fixed.One, fixed.One)))
	world.SetPrimitiveRender(id, scene.NewPrimitiveRender(scene.NewColorRGBA(255, 255, 255, 255)))

	world.DestroyEntity(id)
	if world.HasEntity(id) {
		t.Fatalf("expected entity to be removed")
	}
	if _, ok := world.Names[id]; ok {
		t.Fatalf("expected name component to be removed with entity")
	}
	if _, ok := world.Bodies[id]; ok {
		t.Fatalf("expected body component to be removed with entity")
	}
}

func TestWorldStepCallsSystems(t *testing.T) {
	world := NewWorld(WorldConfig{})
	system := &recordSystem{}
	world.AddSystem(system)
	world.Step(fixed.FromFraction(1, 60))
	if system.steps != 1 {
		t.Fatalf("expected system to be called once, got %d", system.steps)
	}
}
