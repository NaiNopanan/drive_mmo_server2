package body

import (
	"server2/internal/fixed"
	"server2/internal/fixed3d"
)

type RigidBody struct {
	ID uint64

	Type BodyType

	Position       fixed3d.Vec3
	LinearVelocity fixed3d.Vec3
	ForceAccum     fixed3d.Vec3

	Mass    fixed.Fixed
	InvMass fixed.Fixed

	Active bool
}

func NewDynamicBody(id uint64, mass fixed.Fixed, pos fixed3d.Vec3) *RigidBody {
	if mass.Cmp(fixed.Zero) <= 0 {
		panic("body: dynamic body mass must be > 0")
	}

	return &RigidBody{
		ID:             id,
		Type:           BodyDynamic,
		Position:       pos,
		LinearVelocity: fixed3d.Zero(),
		ForceAccum:     fixed3d.Zero(),
		Mass:           mass,
		InvMass:        fixed.One.Div(mass),
		Active:         true,
	}
}

func NewStaticBody(id uint64, pos fixed3d.Vec3) *RigidBody {
	return &RigidBody{
		ID:             id,
		Type:           BodyStatic,
		Position:       pos,
		LinearVelocity: fixed3d.Zero(),
		ForceAccum:     fixed3d.Zero(),
		Mass:           fixed.Zero,
		InvMass:        fixed.Zero,
		Active:         true,
	}
}

func (b *RigidBody) ApplyForce(force fixed3d.Vec3) {
	if b.Type != BodyDynamic {
		return
	}
	b.ForceAccum = b.ForceAccum.Add(force)
}

func (b *RigidBody) ApplyImpulse(impulse fixed3d.Vec3) {
	if b.Type != BodyDynamic {
		return
	}
	b.LinearVelocity = b.LinearVelocity.Add(impulse.MulScalar(b.InvMass))
}

func (b *RigidBody) ClearAccumulators() {
	b.ForceAccum = fixed3d.Zero()
}
