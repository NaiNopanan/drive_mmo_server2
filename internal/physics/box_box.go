package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type BoxBoxContact struct {
	Hit         bool
	Normal      geometry.Vector3
	Penetration fixed.Fixed
}

func FindBoxBoxContact(positionA, halfExtentsA, positionB, halfExtentsB geometry.Vector3) BoxBoxContact {
	delta := positionB.Sub(positionA)
	overlapX := halfExtentsA.X.Add(halfExtentsB.X).Sub(delta.X.Abs())
	if overlapX.Cmp(fixed.Zero) <= 0 {
		return BoxBoxContact{}
	}
	overlapY := halfExtentsA.Y.Add(halfExtentsB.Y).Sub(delta.Y.Abs())
	if overlapY.Cmp(fixed.Zero) <= 0 {
		return BoxBoxContact{}
	}
	overlapZ := halfExtentsA.Z.Add(halfExtentsB.Z).Sub(delta.Z.Abs())
	if overlapZ.Cmp(fixed.Zero) <= 0 {
		return BoxBoxContact{}
	}

	normal := geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
	penetration := overlapX
	if delta.X.Cmp(fixed.Zero) < 0 {
		normal = normal.Neg()
	}

	if overlapY.Cmp(penetration) < 0 {
		penetration = overlapY
		normal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
		if delta.Y.Cmp(fixed.Zero) < 0 {
			normal = normal.Neg()
		}
	}

	if overlapZ.Cmp(penetration) < 0 {
		penetration = overlapZ
		normal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
		if delta.Z.Cmp(fixed.Zero) < 0 {
			normal = normal.Neg()
		}
	}

	return BoxBoxContact{
		Hit:         true,
		Normal:      normal,
		Penetration: penetration,
	}
}

func ResolveBoxBoxContact(
	bodyA *BoxBody,
	bodyB *BoxBody,
	restitution fixed.Fixed,
) BoxBoxContact {
	if bodyA == nil || bodyB == nil {
		return BoxBoxContact{}
	}

	contact := FindBoxBoxContact(bodyA.Motion.Position, bodyA.HalfExtents, bodyB.Motion.Position, bodyB.HalfExtents)
	if !contact.Hit {
		return contact
	}

	restitution = clampUnitFixed(restitution)
	totalInverseMass := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		bodyA.Motion.Position = bodyA.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(bodyA.Motion.InverseMass)))
		bodyB.Motion.Position = bodyB.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(bodyB.Motion.InverseMass)))
	}

	relativeVelocity := bodyB.Motion.Velocity.Sub(bodyA.Motion.Velocity)
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	if normalVelocity.Cmp(fixed.Zero) < 0 && totalInverseMass != fixed.Zero {
		impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(totalInverseMass)
		impulse := contact.Normal.Scale(impulseMagnitude)
		bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
		bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
	}

	return contact
}

func AdvanceBoxBodyWithGravity(body *BoxBody, dt fixed.Fixed, gravity geometry.Vector3) {
	if body == nil {
		return
	}

	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateBoxRotation(body, dt)
}

func ConstrainBoxBodyToOpenContainer(body *BoxBody, minX, maxX, minZ, maxZ, floorY, wallHeight fixed.Fixed) bool {
	if body == nil {
		return false
	}

	hadContact := false
	body.Grounded = false

	if body.Motion.Position.Y.Sub(body.HalfExtents.Y).Cmp(floorY) < 0 {
		body.Motion.Position.Y = floorY.Add(body.HalfExtents.Y)
		if body.Motion.Velocity.Y.Cmp(fixed.Zero) < 0 {
			body.Motion.Velocity.Y = body.Motion.Velocity.Y.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		applyBoxContactFriction(body, geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
		body.Grounded = true
		hadContact = true
	}

	if body.Motion.Position.Y.Add(body.HalfExtents.Y).Cmp(wallHeight) > 0 {
		body.Motion.Position.Y = wallHeight.Sub(body.HalfExtents.Y)
		if body.Motion.Velocity.Y.Cmp(fixed.Zero) > 0 {
			body.Motion.Velocity.Y = body.Motion.Velocity.Y.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		hadContact = true
	}

	if body.Motion.Position.X.Sub(body.HalfExtents.X).Cmp(minX) < 0 {
		body.Motion.Position.X = minX.Add(body.HalfExtents.X)
		if body.Motion.Velocity.X.Cmp(fixed.Zero) < 0 {
			body.Motion.Velocity.X = body.Motion.Velocity.X.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		hadContact = true
	}

	if body.Motion.Position.X.Add(body.HalfExtents.X).Cmp(maxX) > 0 {
		body.Motion.Position.X = maxX.Sub(body.HalfExtents.X)
		if body.Motion.Velocity.X.Cmp(fixed.Zero) > 0 {
			body.Motion.Velocity.X = body.Motion.Velocity.X.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		hadContact = true
	}

	if body.Motion.Position.Z.Sub(body.HalfExtents.Z).Cmp(minZ) < 0 {
		body.Motion.Position.Z = minZ.Add(body.HalfExtents.Z)
		if body.Motion.Velocity.Z.Cmp(fixed.Zero) < 0 {
			body.Motion.Velocity.Z = body.Motion.Velocity.Z.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		hadContact = true
	}

	if body.Motion.Position.Z.Add(body.HalfExtents.Z).Cmp(maxZ) > 0 {
		body.Motion.Position.Z = maxZ.Sub(body.HalfExtents.Z)
		if body.Motion.Velocity.Z.Cmp(fixed.Zero) > 0 {
			body.Motion.Velocity.Z = body.Motion.Velocity.Z.Neg().Mul(clampUnitFixed(body.Restitution))
		}
		hadContact = true
	}

	return hadContact
}
