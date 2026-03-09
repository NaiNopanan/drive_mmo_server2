package body

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type RigidBody struct {
	ID uint64

	Type BodyType

	Position       geom.Vec3
	LinearVelocity geom.Vec3
	ForceAccum     geom.Vec3

	Mass    fixed.Fixed
	InvMass fixed.Fixed

	Active bool
}

func NewDynamicBody(id uint64, mass fixed.Fixed, pos geom.Vec3) *RigidBody {
	if mass.Cmp(fixed.Zero) <= 0 {
		panic("body: dynamic body mass must be > 0")
	}

	return &RigidBody{
		ID:             id,
		Type:           BodyDynamic,
		Position:       pos,
		LinearVelocity: geom.Zero(),
		ForceAccum:     geom.Zero(),
		Mass:           mass,
		InvMass:        fixed.One.Div(mass),
		Active:         true,
	}
}

func NewStaticBody(id uint64, pos geom.Vec3) *RigidBody {
	return &RigidBody{
		ID:             id,
		Type:           BodyStatic,
		Position:       pos,
		LinearVelocity: geom.Zero(),
		ForceAccum:     geom.Zero(),
		Mass:           fixed.Zero,
		InvMass:        fixed.Zero,
		Active:         true,
	}
}

func (b *RigidBody) ApplyForce(force geom.Vec3) {
	if b.Type != BodyDynamic {
		return
	}
	b.ForceAccum = b.ForceAccum.Add(force)
}

func (b *RigidBody) ApplyImpulse(impulse geom.Vec3) {
	if b.Type != BodyDynamic {
		return
	}
	b.LinearVelocity = b.LinearVelocity.Add(impulse.Scale(b.InvMass))
}

func (b *RigidBody) ClearAccumulators() {
	b.ForceAccum = geom.Zero()
}
