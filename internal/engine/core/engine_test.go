package core

import (
	"testing"

	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	enginephysics "server2/internal/engine/physics"
	"server2/internal/engine/scene"
)

func TestEngineUpdateAdvancesDynamicBodyAndSyncsTransform(t *testing.T) {
	engine := New(EngineConfig{
		FixedStep: fixed.FromFraction(1, 2),
		Gravity:   geometry.ZeroVector3(),
	})

	entity := engine.World().NewEntity()
	body := enginephysics.NewDynamicBox(
		fixed.One,
		geometry.ZeroVector3(),
		geometry.NewVector3(fixed.One, fixed.One, fixed.One),
	)
	body.Velocity = geometry.NewVector3(fixed.FromInt(2), fixed.Zero, fixed.Zero)

	engine.World().SetBody(entity, body)
	engine.World().SetTransform(entity, scene.IdentityTransform())
	engine.Update(fixed.FromFraction(1, 2))

	transform := engine.World().Transforms[entity]
	if transform.Position.X.Cmp(fixed.One) < 0 {
		t.Fatalf("expected transform to sync moved body, got x=%v", transform.Position.X)
	}
}

func TestKinematicSystemMovesBodyToTarget(t *testing.T) {
	engine := New(EngineConfig{
		FixedStep: fixed.One,
		Gravity:   geometry.ZeroVector3(),
	})

	entity := engine.World().NewEntity()
	body := enginephysics.NewKinematicBox(
		geometry.ZeroVector3(),
		geometry.NewVector3(fixed.One, fixed.One, fixed.One),
	)
	body.KinematicTargetPosition = geometry.NewVector3(fixed.FromInt(4), fixed.FromInt(2), fixed.Zero)
	engine.World().SetBody(entity, body)
	engine.World().SetTransform(entity, scene.IdentityTransform())

	engine.Update(fixed.One)
	if engine.World().Bodies[entity].Position.X.Cmp(fixed.FromInt(4)) != 0 {
		t.Fatalf("expected kinematic body to move to target")
	}
}
