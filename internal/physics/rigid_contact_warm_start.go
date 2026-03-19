package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type ContactWarmStartManifold struct {
	Point          geometry.Vector3
	Normal         geometry.Vector3
	NormalImpulse  fixed.Fixed
	TangentImpulse geometry.Vector3
}

func applyRigidSphereSphereWarmStart(bodyA, bodyB *RigidSphereBody3D, normal geometry.Vector3, normalImpulse fixed.Fixed, tangentImpulse geometry.Vector3) {
	if bodyA == nil || bodyB == nil {
		return
	}

	rA := normal.Scale(bodyA.Radius)
	rB := normal.Scale(bodyB.Radius.Neg())
	if normalImpulse.Cmp(fixed.Zero) > 0 {
		impulse := normal.Scale(normalImpulse)
		bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
		bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
		bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyRigidSphereInverseInertia(*bodyA, rA.Cross(impulse)))
		bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyRigidSphereInverseInertia(*bodyB, rB.Cross(impulse)))
	}
	if tangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
		bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(tangentImpulse.Scale(bodyA.Motion.InverseMass))
		bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(tangentImpulse.Scale(bodyB.Motion.InverseMass))
		bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyRigidSphereInverseInertia(*bodyA, rA.Cross(tangentImpulse)))
		bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyRigidSphereInverseInertia(*bodyB, rB.Cross(tangentImpulse)))
	}
}

func applyRigidSphereRigidBoxWarmStart(sphere *RigidSphereBody3D, box *RigidBoxBody3D, point, normal geometry.Vector3, normalImpulse fixed.Fixed, tangentImpulse geometry.Vector3) {
	if sphere == nil || box == nil {
		return
	}

	rSphere := point.Sub(sphere.Motion.Position)
	rBox := point.Sub(box.Motion.Position)
	if normalImpulse.Cmp(fixed.Zero) > 0 {
		impulse := normal.Scale(normalImpulse)
		sphere.Motion.Velocity = sphere.Motion.Velocity.Add(impulse.Scale(sphere.Motion.InverseMass))
		box.Motion.Velocity = box.Motion.Velocity.Sub(impulse.Scale(box.Motion.InverseMass))
		sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, rSphere.Cross(impulse)))
		box.AngularVelocity = box.AngularVelocity.Sub(applyInverseInertiaWorld(*box, rBox.Cross(impulse)))
	}
	if tangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
		sphere.Motion.Velocity = sphere.Motion.Velocity.Add(tangentImpulse.Scale(sphere.Motion.InverseMass))
		box.Motion.Velocity = box.Motion.Velocity.Sub(tangentImpulse.Scale(box.Motion.InverseMass))
		sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, rSphere.Cross(tangentImpulse)))
		box.AngularVelocity = box.AngularVelocity.Sub(applyInverseInertiaWorld(*box, rBox.Cross(tangentImpulse)))
	}
}

func applyRigidBoxBoxWarmStart(bodyA, bodyB *RigidBoxBody3D, point, normal geometry.Vector3, normalImpulse fixed.Fixed) {
	if bodyA == nil || bodyB == nil || normalImpulse.Cmp(fixed.Zero) <= 0 {
		return
	}

	rA := point.Sub(bodyA.Motion.Position)
	rB := point.Sub(bodyB.Motion.Position)
	impulse := normal.Scale(normalImpulse)
	bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
	bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
	bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyInverseInertiaWorld(*bodyA, rA.Cross(impulse)))
	bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyInverseInertiaWorld(*bodyB, rB.Cross(impulse)))
}

func ResolveRigidSphereSphereContactWithFrictionWarmStart(
	bodyA *RigidSphereBody3D,
	bodyB *RigidSphereBody3D,
	restitution fixed.Fixed,
	friction fixed.Fixed,
	manifold ContactWarmStartManifold,
) (SphereSphereContact, ContactWarmStartManifold) {
	if bodyA == nil || bodyB == nil {
		return SphereSphereContact{}, ContactWarmStartManifold{}
	}

	contact := FindSphereSphereContact(bodyA.Motion.Position, bodyA.Radius, bodyB.Motion.Position, bodyB.Radius)
	if !contact.Hit {
		return contact, ContactWarmStartManifold{}
	}

	restitution = clampUnitFixed(restitution)
	friction = clampUnitFixed(friction)

	totalInverseMass := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		bodyA.Motion.Position = bodyA.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(bodyA.Motion.InverseMass)))
		bodyB.Motion.Position = bodyB.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(bodyB.Motion.InverseMass)))
	}

	if manifold.NormalImpulse.Cmp(fixed.Zero) > 0 || manifold.TangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
		applyRigidSphereSphereWarmStart(bodyA, bodyB, contact.Normal, manifold.NormalImpulse, manifold.TangentImpulse)
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

	tangentImpulse := geometry.ZeroVector3()
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
				tangentImpulse = tangentDirection.Scale(tangentImpulseMagnitude)
				bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(tangentImpulse.Scale(bodyA.Motion.InverseMass))
				bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(tangentImpulse.Scale(bodyB.Motion.InverseMass))
				bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyRigidSphereInverseInertia(*bodyA, rA.Cross(tangentImpulse)))
				bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyRigidSphereInverseInertia(*bodyB, rB.Cross(tangentImpulse)))
			}
		}
	}

	return contact, ContactWarmStartManifold{
		Point:          bodyA.Motion.Position.Add(bodyB.Motion.Position).Scale(fixed.FromFraction(1, 2)),
		Normal:         contact.Normal,
		NormalImpulse:  normalImpulseMagnitude,
		TangentImpulse: tangentImpulse,
	}
}

func ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(
	sphere *RigidSphereBody3D,
	box *RigidBoxBody3D,
	restitution fixed.Fixed,
	friction fixed.Fixed,
	manifold ContactWarmStartManifold,
) (SphereBoxContact, ContactWarmStartManifold) {
	if sphere == nil || box == nil {
		return SphereBoxContact{}, ContactWarmStartManifold{}
	}

	contact := FindRigidSphereRigidBoxContact(*sphere, *box)
	if !contact.Hit {
		return contact, ContactWarmStartManifold{}
	}

	restitution = clampUnitFixed(restitution)
	friction = clampUnitFixed(friction)

	totalInverseMass := sphere.Motion.InverseMass.Add(box.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		sphere.Motion.Position = sphere.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(sphere.Motion.InverseMass)))
		box.Motion.Position = box.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(box.Motion.InverseMass)))
	}

	if manifold.NormalImpulse.Cmp(fixed.Zero) > 0 || manifold.TangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
		applyRigidSphereRigidBoxWarmStart(sphere, box, contact.Point, contact.Normal, manifold.NormalImpulse, manifold.TangentImpulse)
	}

	rSphere := contact.Point.Sub(sphere.Motion.Position)
	rBox := contact.Point.Sub(box.Motion.Position)
	relativeVelocity := sphere.Motion.Velocity.Add(sphere.AngularVelocity.Cross(rSphere)).
		Sub(box.Motion.Velocity.Add(box.AngularVelocity.Cross(rBox)))
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	normalImpulseMagnitude := fixed.Zero

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		angularContribution := applyRigidSphereInverseInertia(*sphere, rSphere.Cross(contact.Normal)).Cross(rSphere).
			Add(applyInverseInertiaWorld(*box, rBox.Cross(contact.Normal)).Cross(rBox))
		denominator := sphere.Motion.InverseMass.Add(box.Motion.InverseMass).Add(contact.Normal.Dot(angularContribution))
		if denominator != fixed.Zero {
			normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(denominator)
			impulse := contact.Normal.Scale(normalImpulseMagnitude)
			sphere.Motion.Velocity = sphere.Motion.Velocity.Add(impulse.Scale(sphere.Motion.InverseMass))
			box.Motion.Velocity = box.Motion.Velocity.Sub(impulse.Scale(box.Motion.InverseMass))
			sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, rSphere.Cross(impulse)))
			box.AngularVelocity = box.AngularVelocity.Sub(applyInverseInertiaWorld(*box, rBox.Cross(impulse)))
		}
	}

	tangentImpulse := geometry.ZeroVector3()
	updatedRelativeVelocity := sphere.Motion.Velocity.Add(sphere.AngularVelocity.Cross(rSphere)).
		Sub(box.Motion.Velocity.Add(box.AngularVelocity.Cross(rBox)))
	tangentVelocity := updatedRelativeVelocity.Sub(contact.Normal.Scale(updatedRelativeVelocity.Dot(contact.Normal)))
	if friction != fixed.Zero && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 {
		tangentDirection := tangentVelocity.Normalize().Neg()
		angularContribution := applyRigidSphereInverseInertia(*sphere, rSphere.Cross(tangentDirection)).Cross(rSphere).
			Add(applyInverseInertiaWorld(*box, rBox.Cross(tangentDirection)).Cross(rBox))
		denominator := sphere.Motion.InverseMass.Add(box.Motion.InverseMass).Add(tangentDirection.Dot(angularContribution))
		if denominator != fixed.Zero {
			tangentImpulseMagnitude := tangentVelocity.Length().Div(denominator)
			maxFrictionImpulse := normalImpulseMagnitude.Mul(friction)
			if tangentImpulseMagnitude.Cmp(maxFrictionImpulse) > 0 {
				tangentImpulseMagnitude = maxFrictionImpulse
			}
			if tangentImpulseMagnitude.Cmp(fixed.Zero) > 0 {
				tangentImpulse = tangentDirection.Scale(tangentImpulseMagnitude)
				sphere.Motion.Velocity = sphere.Motion.Velocity.Add(tangentImpulse.Scale(sphere.Motion.InverseMass))
				box.Motion.Velocity = box.Motion.Velocity.Sub(tangentImpulse.Scale(box.Motion.InverseMass))
				sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, rSphere.Cross(tangentImpulse)))
				box.AngularVelocity = box.AngularVelocity.Sub(applyInverseInertiaWorld(*box, rBox.Cross(tangentImpulse)))
			}
		}
	}

	return contact, ContactWarmStartManifold{
		Point:          contact.Point,
		Normal:         contact.Normal,
		NormalImpulse:  normalImpulseMagnitude,
		TangentImpulse: tangentImpulse,
	}
}

func ResolveRigidBoxBoxContactWarmStart(
	bodyA *RigidBoxBody3D,
	bodyB *RigidBoxBody3D,
	restitution fixed.Fixed,
	manifold ContactWarmStartManifold,
) (RigidBoxBoxContact, ContactWarmStartManifold) {
	if bodyA == nil || bodyB == nil {
		return RigidBoxBoxContact{}, ContactWarmStartManifold{}
	}

	contact := FindRigidBoxBoxContact(*bodyA, *bodyB)
	if !contact.Hit {
		return contact, ContactWarmStartManifold{}
	}

	restitution = clampUnitFixed(restitution)
	totalInverseMass := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		bodyA.Motion.Position = bodyA.Motion.Position.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(bodyA.Motion.InverseMass)))
		bodyB.Motion.Position = bodyB.Motion.Position.Add(contact.Normal.Scale(separationPerInverseMass.Mul(bodyB.Motion.InverseMass)))
	}

	if manifold.NormalImpulse.Cmp(fixed.Zero) > 0 {
		applyRigidBoxBoxWarmStart(bodyA, bodyB, contact.Point, contact.Normal, manifold.NormalImpulse)
	}

	rA := contact.Point.Sub(bodyA.Motion.Position)
	rB := contact.Point.Sub(bodyB.Motion.Position)
	relativeVelocity := bodyB.Motion.Velocity.Add(bodyB.AngularVelocity.Cross(rB)).
		Sub(bodyA.Motion.Velocity.Add(bodyA.AngularVelocity.Cross(rA)))
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	normalImpulseMagnitude := fixed.Zero

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		angularContributionA := applyInverseInertiaWorld(*bodyA, rA.Cross(contact.Normal)).Cross(rA)
		angularContributionB := applyInverseInertiaWorld(*bodyB, rB.Cross(contact.Normal)).Cross(rB)
		denominator := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass).
			Add(contact.Normal.Dot(angularContributionA.Add(angularContributionB)))
		if denominator != fixed.Zero {
			normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(denominator)
			impulse := contact.Normal.Scale(normalImpulseMagnitude)
			bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
			bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
			bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyInverseInertiaWorld(*bodyA, rA.Cross(impulse)))
			bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyInverseInertiaWorld(*bodyB, rB.Cross(impulse)))
		}
	}

	return contact, ContactWarmStartManifold{
		Point:         contact.Point,
		Normal:        contact.Normal,
		NormalImpulse: normalImpulseMagnitude,
	}
}
