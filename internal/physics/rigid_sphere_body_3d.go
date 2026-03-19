package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type RigidSphereBody3D struct {
	Motion                MotionState
	Radius                fixed.Fixed
	Restitution           fixed.Fixed
	Friction              fixed.Fixed
	UseCCD                bool
	CCDMode               CCDMode
	CCDActive             bool
	CCDVelocityThreshold  fixed.Fixed
	SleepLinearThreshold  fixed.Fixed
	SleepAngularThreshold fixed.Fixed
	SleepTickThreshold    int
	SleepTickCount        int
	Sleeping              bool
	Grounded              bool
	Orientation           Quaternion
	AngularVelocity       geometry.Vector3
	InverseInertia        fixed.Fixed
}

type RigidSphereStepResult struct {
	HadContact  bool
	LastContact SphereTriangleContact
}

func NewRigidSphereBody3D(mass, radius fixed.Fixed, position geometry.Vector3) RigidSphereBody3D {
	if radius.Cmp(fixed.Zero) <= 0 {
		panic("physics: rigid sphere radius must be > 0")
	}

	return RigidSphereBody3D{
		Motion:                NewDynamicMotionState(mass, position),
		Radius:                radius,
		CCDMode:               CCDModeDiscrete,
		CCDVelocityThreshold:  radius,
		SleepLinearThreshold:  fixed.FromFraction(1, 20),
		SleepAngularThreshold: fixed.FromFraction(1, 20),
		SleepTickThreshold:    30,
		Orientation:           IdentityQuaternion(),
		InverseInertia:        computeRigidSphereInverseInertia(mass, radius),
	}
}

func computeRigidSphereInverseInertia(mass, radius fixed.Fixed) fixed.Fixed {
	if mass == fixed.Zero || radius == fixed.Zero {
		return fixed.Zero
	}

	inertia := mass.Mul(radius.Mul(radius)).Mul(fixed.FromFraction(2, 5))
	if inertia == fixed.Zero {
		return fixed.Zero
	}
	return fixed.One.Div(inertia)
}

func integrateRigidSphereOrientation(body *RigidSphereBody3D, dt fixed.Fixed) {
	if body == nil {
		return
	}

	omega := Quaternion{
		X: body.AngularVelocity.X,
		Y: body.AngularVelocity.Y,
		Z: body.AngularVelocity.Z,
	}
	qDot := omega.Mul(body.Orientation).Scale(fixed.FromFraction(1, 2))
	body.Orientation = body.Orientation.Add(qDot.Scale(dt)).Normalize()
}

func applyRigidSphereInverseInertia(body RigidSphereBody3D, vector geometry.Vector3) geometry.Vector3 {
	return vector.Scale(body.InverseInertia)
}

func AdvanceRigidSphereBody3D(body *RigidSphereBody3D, dt fixed.Fixed) {
	if body == nil {
		return
	}

	StepLinearMotion(&body.Motion, dt)
	integrateRigidSphereOrientation(body, dt)
}

func AdvanceRigidSphereBody3DWithoutForce(body *RigidSphereBody3D, dt fixed.Fixed) {
	if body == nil {
		return
	}

	StepLinearMotion(&body.Motion, dt)
	integrateRigidSphereOrientation(body, dt)
}

func StepRigidSphereBody3DWithGravity(body *RigidSphereBody3D, dt fixed.Fixed, gravity geometry.Vector3, triangles []geometry.Triangle) RigidSphereStepResult {
	if body == nil {
		return RigidSphereStepResult{}
	}

	body.Grounded = false
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateRigidSphereOrientation(body, dt)

	if body.Restitution.Cmp(fixed.Zero) < 0 {
		body.Restitution = fixed.Zero
	}
	if body.Restitution.Cmp(fixed.One) > 0 {
		body.Restitution = fixed.One
	}
	if body.Friction.Cmp(fixed.Zero) < 0 {
		body.Friction = fixed.Zero
	}
	if body.Friction.Cmp(fixed.One) > 0 {
		body.Friction = fixed.One
	}

	result := RigidSphereStepResult{}
	for pass := 0; pass < maxSphereTriangleResolvePasses; pass++ {
		candidate, ok := selectDeepestSphereTriangleContact(body.Motion.Position, body.Radius, triangles)
		if !ok {
			break
		}

		contact := candidate.Contact
		body.Motion.Position = body.Motion.Position.Add(contact.Normal.Scale(contact.Penetration))

		contactPoint := contact.Point
		r := contactPoint.Sub(body.Motion.Position)
		relativeVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(r))
		normalVelocity := relativeVelocity.Dot(contact.Normal)
		normalImpulseMagnitude := fixed.Zero

		if normalVelocity.Cmp(fixed.Zero) < 0 {
			rCrossN := r.Cross(contact.Normal)
			angularContribution := applyRigidSphereInverseInertia(*body, rCrossN).Cross(r)
			denominator := body.Motion.InverseMass.Add(contact.Normal.Dot(angularContribution))
			if denominator != fixed.Zero {
				normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(body.Restitution)).Div(denominator)
				normalImpulse := contact.Normal.Scale(normalImpulseMagnitude)
				body.Motion.Velocity = body.Motion.Velocity.Add(normalImpulse.Scale(body.Motion.InverseMass))
				body.AngularVelocity = body.AngularVelocity.Add(applyRigidSphereInverseInertia(*body, r.Cross(normalImpulse)))
			}
		}

		updatedRelativeVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(r))
		tangentVelocity := updatedRelativeVelocity.Sub(contact.Normal.Scale(updatedRelativeVelocity.Dot(contact.Normal)))
		if body.Friction.Cmp(fixed.Zero) > 0 && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 {
			tangentDirection := tangentVelocity.Normalize().Neg()
			rCrossT := r.Cross(tangentDirection)
			angularContribution := applyRigidSphereInverseInertia(*body, rCrossT).Cross(r)
			denominator := body.Motion.InverseMass.Add(tangentDirection.Dot(angularContribution))
			if denominator != fixed.Zero {
				tangentImpulseMagnitude := tangentVelocity.Length().Div(denominator)
				maxFrictionImpulse := normalImpulseMagnitude.Mul(body.Friction)
				if tangentImpulseMagnitude.Cmp(maxFrictionImpulse) > 0 {
					tangentImpulseMagnitude = maxFrictionImpulse
				}
				if tangentImpulseMagnitude.Cmp(fixed.Zero) > 0 {
					tangentImpulse := tangentDirection.Scale(tangentImpulseMagnitude)
					body.Motion.Velocity = body.Motion.Velocity.Add(tangentImpulse.Scale(body.Motion.InverseMass))
					body.AngularVelocity = body.AngularVelocity.Add(applyRigidSphereInverseInertia(*body, r.Cross(tangentImpulse)))
				}
			}
		}

		result.HadContact = true
		result.LastContact = SphereTriangleContact{
			Hit:         true,
			Point:       contactPoint,
			Normal:      contact.Normal,
			Penetration: contact.Penetration,
		}
	}

	if result.HadContact {
		body.Grounded = result.LastContact.Normal.Y.Cmp(fixed.Zero) > 0
	}

	return result
}

func ResolveRigidSphereBody3DPlaneOverride(body *RigidSphereBody3D, planePoint, planeNormal geometry.Vector3, contactRestitution fixed.Fixed, contactFriction fixed.Fixed) RigidSphereStepResult {
	if body == nil {
		return RigidSphereStepResult{}
	}

	contactRestitution = clampUnitFixed(contactRestitution)
	contactFriction = clampUnitFixed(contactFriction)

	signedDistance := body.Motion.Position.Sub(planePoint).Dot(planeNormal).Sub(body.Radius)
	if signedDistance.Cmp(fixed.Zero) >= 0 {
		return RigidSphereStepResult{}
	}

	penetration := signedDistance.Neg()
	body.Motion.Position = body.Motion.Position.Add(planeNormal.Scale(penetration))

	contactPoint := body.Motion.Position.Sub(planeNormal.Scale(body.Radius))
	r := contactPoint.Sub(body.Motion.Position)
	relativeVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(r))
	normalVelocity := relativeVelocity.Dot(planeNormal)
	normalImpulseMagnitude := fixed.Zero

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		rCrossN := r.Cross(planeNormal)
		angularContribution := applyRigidSphereInverseInertia(*body, rCrossN).Cross(r)
		denominator := body.Motion.InverseMass.Add(planeNormal.Dot(angularContribution))
		if denominator != fixed.Zero {
			normalImpulseMagnitude = normalVelocity.Neg().Mul(fixed.One.Add(contactRestitution)).Div(denominator)
			normalImpulse := planeNormal.Scale(normalImpulseMagnitude)
			body.Motion.Velocity = body.Motion.Velocity.Add(normalImpulse.Scale(body.Motion.InverseMass))
			body.AngularVelocity = body.AngularVelocity.Add(applyRigidSphereInverseInertia(*body, r.Cross(normalImpulse)))
		}
	}

	updatedRelativeVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(r))
	tangentVelocity := updatedRelativeVelocity.Sub(planeNormal.Scale(updatedRelativeVelocity.Dot(planeNormal)))
	if contactFriction.Cmp(fixed.Zero) > 0 && tangentVelocity.LengthSquared().Cmp(fixed.Zero) > 0 {
		tangentDirection := tangentVelocity.Normalize().Neg()
		rCrossT := r.Cross(tangentDirection)
		angularContribution := applyRigidSphereInverseInertia(*body, rCrossT).Cross(r)
		denominator := body.Motion.InverseMass.Add(tangentDirection.Dot(angularContribution))
		if denominator != fixed.Zero {
			tangentImpulseMagnitude := tangentVelocity.Length().Div(denominator)
			maxFrictionImpulse := normalImpulseMagnitude.Mul(contactFriction)
			if tangentImpulseMagnitude.Cmp(maxFrictionImpulse) > 0 {
				tangentImpulseMagnitude = maxFrictionImpulse
			}
			if tangentImpulseMagnitude.Cmp(fixed.Zero) > 0 {
				tangentImpulse := tangentDirection.Scale(tangentImpulseMagnitude)
				body.Motion.Velocity = body.Motion.Velocity.Add(tangentImpulse.Scale(body.Motion.InverseMass))
				body.AngularVelocity = body.AngularVelocity.Add(applyRigidSphereInverseInertia(*body, r.Cross(tangentImpulse)))
			}
		}
	}

	body.Grounded = planeNormal.Y.Cmp(fixed.Zero) > 0

	return RigidSphereStepResult{
		HadContact: true,
		LastContact: SphereTriangleContact{
			Hit:         true,
			Point:       contactPoint,
			Normal:      planeNormal,
			Penetration: penetration,
		},
	}
}

func ConstrainRigidSphereBody3DToOpenContainer(body *RigidSphereBody3D, minX, maxX, minZ, maxZ, floorY fixed.Fixed) bool {
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
		result := ResolveRigidSphereBody3DPlaneOverride(body, plane.point, plane.normal, body.Restitution, body.Friction)
		if result.HadContact {
			hadContact = true
		}
	}

	return hadContact
}
