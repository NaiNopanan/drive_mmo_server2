package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SphereSphereContact struct {
	Hit         bool
	Normal      geometry.Vector3
	Penetration fixed.Fixed
}

func FindSphereSphereContact(centerA geometry.Vector3, radiusA fixed.Fixed, centerB geometry.Vector3, radiusB fixed.Fixed) SphereSphereContact {
	delta := centerB.Sub(centerA)
	distanceSquared := delta.LengthSquared()
	radiusSum := radiusA.Add(radiusB)
	radiusSumSquared := radiusSum.Mul(radiusSum)
	if distanceSquared.Cmp(radiusSumSquared) > 0 {
		return SphereSphereContact{}
	}

	normal := geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
	distance := fixed.Zero
	if distanceSquared != fixed.Zero {
		distance = distanceSquared.Sqrt()
		if distance != fixed.Zero {
			normal = delta.Scale(fixed.One.Div(distance))
		}
	}

	penetration := radiusSum.Sub(distance)
	if penetration.Cmp(fixed.Zero) <= 0 {
		return SphereSphereContact{}
	}

	return SphereSphereContact{
		Hit:         true,
		Normal:      normal,
		Penetration: penetration,
	}
}

func ResolveSphereSphereContact(
	positionA geometry.Vector3,
	velocityA geometry.Vector3,
	inverseMassA fixed.Fixed,
	radiusA fixed.Fixed,
	positionB geometry.Vector3,
	velocityB geometry.Vector3,
	inverseMassB fixed.Fixed,
	radiusB fixed.Fixed,
	restitution fixed.Fixed,
) (geometry.Vector3, geometry.Vector3, geometry.Vector3, geometry.Vector3, SphereSphereContact) {
	contact := FindSphereSphereContact(positionA, radiusA, positionB, radiusB)
	if !contact.Hit {
		return positionA, velocityA, positionB, velocityB, contact
	}

	if restitution.Cmp(fixed.Zero) < 0 {
		restitution = fixed.Zero
	}
	if restitution.Cmp(fixed.One) > 0 {
		restitution = fixed.One
	}

	totalInverseMass := inverseMassA.Add(inverseMassB)
	if totalInverseMass != fixed.Zero {
		separationPerInverseMass := contact.Penetration.Div(totalInverseMass)
		positionA = positionA.Sub(contact.Normal.Scale(separationPerInverseMass.Mul(inverseMassA)))
		positionB = positionB.Add(contact.Normal.Scale(separationPerInverseMass.Mul(inverseMassB)))
	}

	relativeVelocity := velocityB.Sub(velocityA)
	normalVelocity := relativeVelocity.Dot(contact.Normal)
	if normalVelocity.Cmp(fixed.Zero) < 0 && totalInverseMass != fixed.Zero {
		impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(totalInverseMass)
		impulse := contact.Normal.Scale(impulseMagnitude)
		velocityA = velocityA.Sub(impulse.Scale(inverseMassA))
		velocityB = velocityB.Add(impulse.Scale(inverseMassB))
	}

	return positionA, velocityA, positionB, velocityB, contact
}
