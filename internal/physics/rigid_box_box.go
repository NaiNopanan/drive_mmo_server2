package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type RigidBoxBoxContact struct {
	Hit         bool
	Point       geometry.Vector3
	Normal      geometry.Vector3
	Penetration fixed.Fixed
}

func rigidBoxWorldAxes(body RigidBoxBody3D) [3]geometry.Vector3 {
	return [3]geometry.Vector3{
		body.Orientation.RotateVector(geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)).Normalize(),
		body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)).Normalize(),
		body.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)).Normalize(),
	}
}

func rigidBoxProjectionRadius(body RigidBoxBody3D, axis geometry.Vector3, worldAxes [3]geometry.Vector3) fixed.Fixed {
	absDot := func(a fixed.Fixed) fixed.Fixed {
		return a.Abs()
	}

	return absDot(axis.Dot(worldAxes[0])).Mul(body.HalfExtents.X).
		Add(absDot(axis.Dot(worldAxes[1])).Mul(body.HalfExtents.Y)).
		Add(absDot(axis.Dot(worldAxes[2])).Mul(body.HalfExtents.Z))
}

func rigidBoxSupportPoint(body RigidBoxBody3D, direction geometry.Vector3, worldAxes [3]geometry.Vector3) geometry.Vector3 {
	point := body.Motion.Position
	extents := []fixed.Fixed{body.HalfExtents.X, body.HalfExtents.Y, body.HalfExtents.Z}
	for index, axis := range worldAxes {
		sign := fixed.One
		if direction.Dot(axis).Cmp(fixed.Zero) < 0 {
			sign = fixed.One.Neg()
		}
		point = point.Add(axis.Scale(extents[index].Mul(sign)))
	}
	return point
}

func FindRigidBoxBoxContact(bodyA, bodyB RigidBoxBody3D) RigidBoxBoxContact {
	axesA := rigidBoxWorldAxes(bodyA)
	axesB := rigidBoxWorldAxes(bodyB)
	centerDelta := bodyB.Motion.Position.Sub(bodyA.Motion.Position)

	bestAxis := geometry.ZeroVector3()
	bestPenetration := fixed.Zero
	found := false

	testAxis := func(axis geometry.Vector3) bool {
		if axis.LengthSquared().Cmp(fixed.FromFraction(1, 1000)) <= 0 {
			return true
		}

		axis = axis.Normalize()
		distance := centerDelta.Dot(axis).Abs()
		radiusA := rigidBoxProjectionRadius(bodyA, axis, axesA)
		radiusB := rigidBoxProjectionRadius(bodyB, axis, axesB)
		overlap := radiusA.Add(radiusB).Sub(distance)
		if overlap.Cmp(fixed.Zero) <= 0 {
			return false
		}

		if !found || overlap.Cmp(bestPenetration) < 0 {
			bestPenetration = overlap
			bestAxis = axis
			if centerDelta.Dot(bestAxis).Cmp(fixed.Zero) < 0 {
				bestAxis = bestAxis.Neg()
			}
			found = true
		}
		return true
	}

	for _, axis := range axesA {
		if !testAxis(axis) {
			return RigidBoxBoxContact{}
		}
	}
	for _, axis := range axesB {
		if !testAxis(axis) {
			return RigidBoxBoxContact{}
		}
	}
	for _, axisA := range axesA {
		for _, axisB := range axesB {
			if !testAxis(axisA.Cross(axisB)) {
				return RigidBoxBoxContact{}
			}
		}
	}

	if !found {
		return RigidBoxBoxContact{}
	}

	supportA := rigidBoxSupportPoint(bodyA, bestAxis, axesA)
	supportB := rigidBoxSupportPoint(bodyB, bestAxis.Neg(), axesB)
	contactPoint := supportA.Add(supportB).Scale(fixed.FromFraction(1, 2))

	return RigidBoxBoxContact{
		Hit:         true,
		Point:       contactPoint,
		Normal:      bestAxis,
		Penetration: bestPenetration,
	}
}

func ResolveRigidBoxBoxContact(bodyA, bodyB *RigidBoxBody3D, restitution fixed.Fixed) RigidBoxBoxContact {
	if bodyA == nil || bodyB == nil {
		return RigidBoxBoxContact{}
	}

	contact := FindRigidBoxBoxContact(*bodyA, *bodyB)
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

	rA := contact.Point.Sub(bodyA.Motion.Position)
	rB := contact.Point.Sub(bodyB.Motion.Position)
	relativeVelocity := bodyB.Motion.Velocity.Add(bodyB.AngularVelocity.Cross(rB)).
		Sub(bodyA.Motion.Velocity.Add(bodyA.AngularVelocity.Cross(rA)))
	normalVelocity := relativeVelocity.Dot(contact.Normal)

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		angularContributionA := applyInverseInertiaWorld(*bodyA, rA.Cross(contact.Normal)).Cross(rA)
		angularContributionB := applyInverseInertiaWorld(*bodyB, rB.Cross(contact.Normal)).Cross(rB)
		denominator := bodyA.Motion.InverseMass.Add(bodyB.Motion.InverseMass).
			Add(contact.Normal.Dot(angularContributionA.Add(angularContributionB)))
		if denominator != fixed.Zero {
			impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(restitution)).Div(denominator)
			impulse := contact.Normal.Scale(impulseMagnitude)
			bodyA.Motion.Velocity = bodyA.Motion.Velocity.Sub(impulse.Scale(bodyA.Motion.InverseMass))
			bodyB.Motion.Velocity = bodyB.Motion.Velocity.Add(impulse.Scale(bodyB.Motion.InverseMass))
			bodyA.AngularVelocity = bodyA.AngularVelocity.Sub(applyInverseInertiaWorld(*bodyA, rA.Cross(impulse)))
			bodyB.AngularVelocity = bodyB.AngularVelocity.Add(applyInverseInertiaWorld(*bodyB, rB.Cross(impulse)))
		}
	}

	return contact
}

func ConstrainRigidBoxBody3DToOpenContainer(body *RigidBoxBody3D, minX, maxX, minZ, maxZ, floorY fixed.Fixed) bool {
	if body == nil {
		return false
	}

	hadContact := false
	body.Grounded = false
	planes := []struct {
		point  geometry.Vector3
		normal geometry.Vector3
	}{
		{point: geometry.NewVector3(fixed.Zero, floorY, fixed.Zero), normal: geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)},
		{point: geometry.NewVector3(minX, fixed.Zero, fixed.Zero), normal: geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)},
		{point: geometry.NewVector3(maxX, fixed.Zero, fixed.Zero), normal: geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero)},
		{point: geometry.NewVector3(fixed.Zero, fixed.Zero, minZ), normal: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One)},
		{point: geometry.NewVector3(fixed.Zero, fixed.Zero, maxZ), normal: geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg())},
	}

	for _, plane := range planes {
		result := ResolveRigidBoxBody3DPlaneOverride(body, plane.point, plane.normal, body.Restitution)
		if result.HadContact {
			hadContact = true
		}
	}

	return hadContact
}
