package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

func FindRigidSphereRigidBoxContact(sphere RigidSphereBody3D, box RigidBoxBody3D) SphereBoxContact {
	localCenter := box.Orientation.Conjugate().RotateVector(sphere.Motion.Position.Sub(box.Motion.Position))
	closestLocal := geometry.NewVector3(
		fixed.Clamp(localCenter.X, box.HalfExtents.X.Neg(), box.HalfExtents.X),
		fixed.Clamp(localCenter.Y, box.HalfExtents.Y.Neg(), box.HalfExtents.Y),
		fixed.Clamp(localCenter.Z, box.HalfExtents.Z.Neg(), box.HalfExtents.Z),
	)
	closestWorld := box.Motion.Position.Add(box.Orientation.RotateVector(closestLocal))
	delta := sphere.Motion.Position.Sub(closestWorld)
	distanceSquared := delta.LengthSquared()
	radiusSquared := sphere.Radius.Mul(sphere.Radius)
	if distanceSquared.Cmp(radiusSquared) > 0 {
		return SphereBoxContact{}
	}

	if distanceSquared != fixed.Zero {
		distance := distanceSquared.Sqrt()
		if distance == fixed.Zero {
			return SphereBoxContact{}
		}
		normal := delta.Scale(fixed.One.Div(distance))
		penetration := sphere.Radius.Sub(distance)
		if penetration.Cmp(fixed.Zero) <= 0 {
			return SphereBoxContact{}
		}
		return SphereBoxContact{
			Hit:         true,
			Point:       closestWorld,
			Normal:      normal,
			Penetration: penetration,
		}
	}

	dxMin := localCenter.X.Sub(box.HalfExtents.X.Neg()).Abs()
	dxMax := box.HalfExtents.X.Sub(localCenter.X).Abs()
	dyMin := localCenter.Y.Sub(box.HalfExtents.Y.Neg()).Abs()
	dyMax := box.HalfExtents.Y.Sub(localCenter.Y).Abs()
	dzMin := localCenter.Z.Sub(box.HalfExtents.Z.Neg()).Abs()
	dzMax := box.HalfExtents.Z.Sub(localCenter.Z).Abs()

	minDistance := dxMin
	localNormal := geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero)
	closestLocal = geometry.NewVector3(box.HalfExtents.X.Neg(), localCenter.Y, localCenter.Z)
	if dxMax.Cmp(minDistance) < 0 {
		minDistance = dxMax
		localNormal = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
		closestLocal = geometry.NewVector3(box.HalfExtents.X, localCenter.Y, localCenter.Z)
	}
	if dyMin.Cmp(minDistance) < 0 {
		minDistance = dyMin
		localNormal = geometry.NewVector3(fixed.Zero, fixed.One.Neg(), fixed.Zero)
		closestLocal = geometry.NewVector3(localCenter.X, box.HalfExtents.Y.Neg(), localCenter.Z)
	}
	if dyMax.Cmp(minDistance) < 0 {
		minDistance = dyMax
		localNormal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
		closestLocal = geometry.NewVector3(localCenter.X, box.HalfExtents.Y, localCenter.Z)
	}
	if dzMin.Cmp(minDistance) < 0 {
		minDistance = dzMin
		localNormal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())
		closestLocal = geometry.NewVector3(localCenter.X, localCenter.Y, box.HalfExtents.Z.Neg())
	}
	if dzMax.Cmp(minDistance) < 0 {
		minDistance = dzMax
		localNormal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
		closestLocal = geometry.NewVector3(localCenter.X, localCenter.Y, box.HalfExtents.Z)
	}

	closestWorld = box.Motion.Position.Add(box.Orientation.RotateVector(closestLocal))
	normal := box.Orientation.RotateVector(localNormal)
	penetration := sphere.Radius.Add(minDistance)

	return SphereBoxContact{
		Hit:         true,
		Point:       closestWorld,
		Normal:      normal,
		Penetration: penetration,
	}
}

func ResolveRigidSphereRigidBoxContactWithFriction(
	sphere *RigidSphereBody3D,
	box *RigidBoxBody3D,
	restitution fixed.Fixed,
	friction fixed.Fixed,
) SphereBoxContact {
	if sphere == nil || box == nil {
		return SphereBoxContact{}
	}

	contact := FindRigidSphereRigidBoxContact(*sphere, *box)
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
				impulse := tangentDirection.Scale(tangentImpulseMagnitude)
				sphere.Motion.Velocity = sphere.Motion.Velocity.Add(impulse.Scale(sphere.Motion.InverseMass))
				box.Motion.Velocity = box.Motion.Velocity.Sub(impulse.Scale(box.Motion.InverseMass))
				sphere.AngularVelocity = sphere.AngularVelocity.Add(applyRigidSphereInverseInertia(*sphere, rSphere.Cross(impulse)))
				box.AngularVelocity = box.AngularVelocity.Sub(applyInverseInertiaWorld(*box, rBox.Cross(impulse)))
			}
		}
	}

	return contact
}
