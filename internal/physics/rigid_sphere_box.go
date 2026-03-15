package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

func ResolveRigidSphereBoxContactWithFriction(
	sphere *RigidSphereBody3D,
	box *BoxBody,
	restitution fixed.Fixed,
	friction fixed.Fixed,
) SphereBoxContact {
	if sphere == nil || box == nil {
		return SphereBoxContact{}
	}

	bounds := geometry.NewAxisAlignedBoundingBox(box.Motion.Position.Sub(box.HalfExtents), box.Motion.Position.Add(box.HalfExtents))
	contact := FindSphereBoxContact(sphere.Motion.Position, sphere.Radius, bounds)
	if !contact.Hit {
		return contact
	}

	restitution = clampUnitFixed(restitution)
	friction = clampUnitFixed(friction)

	totalInverseMass := sphere.Motion.InverseMass.Add(box.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		sphere.Motion.Position = sphere.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(sphere.Motion.InverseMass)))
		box.Motion.Position = box.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(box.Motion.InverseMass)))
	}

	r := contact.Point.Sub(sphere.Motion.Position)
	relativeVelocity := sphere.Motion.Velocity.Add(sphere.AngularVelocity.Cross(r)).Sub(box.Motion.Velocity)
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	normalImpulseMagnitude := fixed.Zero

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		rCrossN := r.Cross(contact.Normal)
		angularContribution := applyRigidSphereInverseInertia(*sphere, rCrossN).Cross(r)
		denominator := sphere.Motion.InverseMass.Add(box.Motion.InverseMass).Add(contact.Normal.Dot(angularContribution))
		if denominator != fixed.Zero {
			normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(denominator)
			normalImpulse := contact.Normal.Scale(normalImpulseMagnitude)
			sphere.Motion.Velocity = sphere.Motion.Velocity.Add(normalImpulse.Scale(sphere.Motion.InverseMass))
			box.Motion.Velocity = box.Motion.Velocity.Sub(normalImpulse.Scale(box.Motion.InverseMass))
			sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, r.Cross(normalImpulse)))
		}
	}

	updatedRelativeVelocity := sphere.Motion.Velocity.Add(sphere.AngularVelocity.Cross(r)).Sub(box.Motion.Velocity)
	tangentVelocity := updatedRelativeVelocity.Sub(contact.Normal.Scale(updatedRelativeVelocity.Dot(contact.Normal)))
	if friction != fixed.Zero && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 {
		tangentDirection := tangentVelocity.Normalize().Neg()
		rCrossT := r.Cross(tangentDirection)
		angularContribution := applyRigidSphereInverseInertia(*sphere, rCrossT).Cross(r)
		denominator := sphere.Motion.InverseMass.Add(box.Motion.InverseMass).Add(tangentDirection.Dot(angularContribution))
		if denominator != fixed.Zero {
			tangentImpulseMagnitude := tangentVelocity.Length().Div(denominator)
			maxFrictionImpulse := normalImpulseMagnitude.Mul(friction)
			if tangentImpulseMagnitude.Cmp(maxFrictionImpulse) > 0 {
				tangentImpulseMagnitude = maxFrictionImpulse
			}
			if tangentImpulseMagnitude.Cmp(fixed.Zero) > 0 {
				tangentImpulse := tangentDirection.Scale(tangentImpulseMagnitude)
				sphere.Motion.Velocity = sphere.Motion.Velocity.Add(tangentImpulse.Scale(sphere.Motion.InverseMass))
				box.Motion.Velocity = box.Motion.Velocity.Sub(tangentImpulse.Scale(box.Motion.InverseMass))
				sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, r.Cross(tangentImpulse)))
			}
		}
	}

	return contact
}
