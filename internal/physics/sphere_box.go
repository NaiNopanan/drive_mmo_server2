package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SphereBoxContact struct {
	Hit         bool
	Point       geometry.Vector3
	Normal      geometry.Vector3
	Penetration fixed.Fixed
}

func FindSphereBoxContact(center geometry.Vector3, radius fixed.Fixed, bounds geometry.AxisAlignedBoundingBox) SphereBoxContact {
	closest := geometry.NewVector3(
		fixed.Clamp(center.X, bounds.Min.X, bounds.Max.X),
		fixed.Clamp(center.Y, bounds.Min.Y, bounds.Max.Y),
		fixed.Clamp(center.Z, bounds.Min.Z, bounds.Max.Z),
	)
	delta := center.Sub(closest)
	distanceSquared := delta.LengthSquared()
	radiusSquared := radius.Mul(radius)
	if distanceSquared.Cmp(radiusSquared) > 0 {
		return SphereBoxContact{}
	}

	normal := geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
	distance := fixed.Zero
	if distanceSquared != fixed.Zero {
		distance = distanceSquared.Sqrt()
		if distance != fixed.Zero {
			normal = delta.Scale(fixed.One.Div(distance))
		}
	} else {
		dxMin := center.X.Sub(bounds.Min.X).Abs()
		dxMax := bounds.Max.X.Sub(center.X).Abs()
		dyMin := center.Y.Sub(bounds.Min.Y).Abs()
		dyMax := bounds.Max.Y.Sub(center.Y).Abs()
		dzMin := center.Z.Sub(bounds.Min.Z).Abs()
		dzMax := bounds.Max.Z.Sub(center.Z).Abs()

		minDistance := dxMin
		normal = geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero)
		if dxMax.Cmp(minDistance) < 0 {
			minDistance = dxMax
			normal = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
		}
		if dyMin.Cmp(minDistance) < 0 {
			minDistance = dyMin
			normal = geometry.NewVector3(fixed.Zero, fixed.One.Neg(), fixed.Zero)
		}
		if dyMax.Cmp(minDistance) < 0 {
			minDistance = dyMax
			normal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
		}
		if dzMin.Cmp(minDistance) < 0 {
			minDistance = dzMin
			normal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())
		}
		if dzMax.Cmp(minDistance) < 0 {
			minDistance = dzMax
			normal = geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)
		}
	}

	penetration := radius.Sub(distance)
	if penetration.Cmp(fixed.Zero) <= 0 {
		return SphereBoxContact{}
	}

	return SphereBoxContact{
		Hit:         true,
		Point:       closest,
		Normal:      normal,
		Penetration: penetration,
	}
}

func ResolveSphereBoxContact(
	spherePosition geometry.Vector3,
	sphereVelocity geometry.Vector3,
	sphereInverseMass fixed.Fixed,
	sphereRadius fixed.Fixed,
	boxPosition geometry.Vector3,
	boxVelocity geometry.Vector3,
	boxInverseMass fixed.Fixed,
	boxHalfExtents geometry.Vector3,
	restitution fixed.Fixed,
) (geometry.Vector3, geometry.Vector3, geometry.Vector3, geometry.Vector3, SphereBoxContact) {
	return ResolveSphereBoxContactWithFriction(
		spherePosition,
		sphereVelocity,
		sphereInverseMass,
		sphereRadius,
		boxPosition,
		boxVelocity,
		boxInverseMass,
		boxHalfExtents,
		restitution,
		fixed.Zero,
	)
}

func ResolveSphereBoxContactWithFriction(
	spherePosition geometry.Vector3,
	sphereVelocity geometry.Vector3,
	sphereInverseMass fixed.Fixed,
	sphereRadius fixed.Fixed,
	boxPosition geometry.Vector3,
	boxVelocity geometry.Vector3,
	boxInverseMass fixed.Fixed,
	boxHalfExtents geometry.Vector3,
	restitution fixed.Fixed,
	friction fixed.Fixed,
) (geometry.Vector3, geometry.Vector3, geometry.Vector3, geometry.Vector3, SphereBoxContact) {
	bounds := geometry.NewAxisAlignedBoundingBox(boxPosition.Sub(boxHalfExtents), boxPosition.Add(boxHalfExtents))
	contact := FindSphereBoxContact(spherePosition, sphereRadius, bounds)
	if !contact.Hit {
		return spherePosition, sphereVelocity, boxPosition, boxVelocity, contact
	}

	if restitution.Cmp(fixed.Zero) < 0 {
		restitution = fixed.Zero
	}
	if restitution.Cmp(fixed.One) > 0 {
		restitution = fixed.One
	}
	friction = clampUnitFixed(friction)

	totalInverseMass := sphereInverseMass.Add(boxInverseMass)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		spherePosition = spherePosition.Add(contact.Normal.Scale(separationPerInverseMass.Mul(sphereInverseMass)))
		boxPosition = boxPosition.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(boxInverseMass)))
	}

	relativeVelocity := sphereVelocity.Sub(boxVelocity)
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	if normalVelocity.Cmp(fixed.Zero) < 0 && totalInverseMass != fixed.Zero {
		impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(totalInverseMass)
		impulse := contact.Normal.Scale(impulseMagnitude)
		sphereVelocity = sphereVelocity.Add(impulse.Scale(sphereInverseMass))
		boxVelocity = boxVelocity.Sub(impulse.Scale(boxInverseMass))
	}

	relativeVelocity = sphereVelocity.Sub(boxVelocity)
	tangentVelocity := relativeVelocity.Sub(contact.Normal.Scale(relativeVelocity.Dot(contact.Normal)))
	if friction != fixed.Zero && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 && totalInverseMass != fixed.Zero {
		tangentDirection := tangentVelocity.Normalize().Neg()
		tangentSpeed := tangentVelocity.Length()
		impulseMagnitude := tangentSpeed.Div(totalInverseMass).Mul(friction)
		impulse := tangentDirection.Scale(impulseMagnitude)
		sphereVelocity = sphereVelocity.Add(impulse.Scale(sphereInverseMass))
		boxVelocity = boxVelocity.Sub(impulse.Scale(boxInverseMass))
	}

	return spherePosition, sphereVelocity, boxPosition, boxVelocity, contact
}
