package physics

import "server2/internal/fixed"

func ResolveRigidSphereSphereContactWithFriction(
	bodyA *RigidSphereBody3D,
	bodyB *RigidSphereBody3D,
	restitution fixed.Fixed,
	friction fixed.Fixed,
) SphereSphereContact {
	if bodyA == nil || bodyB == nil {
		return SphereSphereContact{}
	}

	contact := FindSphereSphereContact(bodyA.Motion.Position, bodyA.Radius, bodyB.Motion.Position, bodyB.Radius)
	if !contact.Hit {
		return contact
	}

	restitution = clampUnitFixed(restitution)
	friction = clampUnitFixed(friction)

	totalInverseMass := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		bodyA.Motion.Position = bodyA.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(bodyA.Motion.InverseMass)))
		bodyB.Motion.Position = bodyB.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(bodyB.Motion.InverseMass)))
	}

	rA := contact.Normal.Scale(bodyA.Radius)
	rB := contact.Normal.Scale(bodyB.Radius.Neg())
	relativeVelocity := bodyB.Motion.Velocity.Add(bodyB.AngularVelocity.Cross(rB)).
		Sub(bodyA.Motion.Velocity.Add(bodyA.AngularVelocity.Cross(rA)))
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	normalImpulseMagnitude := fixed.Zero

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		rACrossN := rA.Cross(contact.Normal)
		rBCrossN := rB.Cross(contact.Normal)
		angularContribution := applyRigidSphereInverseInertia(*bodyA, rACrossN).Cross(rA).
			Add(applyRigidSphereInverseInertia(*bodyB, rBCrossN).Cross(rB))
		denominator := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass).Add(contact.Normal.Dot(angularContribution))
		if denominator != fixed.Zero {
			normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(denominator)
			impulse := contact.Normal.Scale(normalImpulseMagnitude)
			bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
			bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
			bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyRigidSphereInverseInertia(*bodyA, rA.Cross(impulse)))
			bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyRigidSphereInverseInertia(*bodyB, rB.Cross(impulse)))
		}
	}

	updatedRelativeVelocity := bodyB.Motion.Velocity.Add(bodyB.AngularVelocity.Cross(rB)).
		Sub(bodyA.Motion.Velocity.Add(bodyA.AngularVelocity.Cross(rA)))
	tangentVelocity := updatedRelativeVelocity.Sub(contact.Normal.Scale(updatedRelativeVelocity.Dot(contact.Normal)))
	if friction != fixed.Zero && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 {
		tangentDirection := tangentVelocity.Normalize().Neg()
		rACrossT := rA.Cross(tangentDirection)
		rBCrossT := rB.Cross(tangentDirection)
		angularContribution := applyRigidSphereInverseInertia(*bodyA, rACrossT).Cross(rA).
			Add(applyRigidSphereInverseInertia(*bodyB, rBCrossT).Cross(rB))
		denominator := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass).Add(tangentDirection.Dot(angularContribution))
		if denominator != fixed.Zero {
			tangentImpulseMagnitude := tangentVelocity.Length().Div(denominator)
			maxFrictionImpulse := normalImpulseMagnitude.Mul(friction)
			if tangentImpulseMagnitude.Cmp(maxFrictionImpulse) > 0 {
				tangentImpulseMagnitude = maxFrictionImpulse
			}
			if tangentImpulseMagnitude.Cmp(fixed.Zero) > 0 {
				impulse := tangentDirection.Scale(tangentImpulseMagnitude)
				bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
				bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
				bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyRigidSphereInverseInertia(*bodyA, rA.Cross(impulse)))
				bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyRigidSphereInverseInertia(*bodyB, rB.Cross(impulse)))
			}
		}
	}

	return contact
}
