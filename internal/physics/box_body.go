package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

// BoxBody is a simple axis-aligned box body with linear motion only.
type BoxBody struct {
	Motion           MotionState
	HalfExtents      geometry.Vector3
	Restitution      fixed.Fixed
	Friction         fixed.Fixed
	Grounded         bool
	RotationZ        fixed.Fixed
	AngularVelocityZ fixed.Fixed
	InverseInertiaZ  fixed.Fixed
}

// BoxStepResult reports contact information produced by one box physics step.
type BoxStepResult struct {
	HadContact  bool
	LastContact SphereTriangleContact
}

// NewDynamicBoxBody creates a dynamic axis-aligned box body with linear motion state.
func NewDynamicBoxBody(mass fixed.Fixed, halfExtents, position geometry.Vector3) BoxBody {
	if halfExtents.X.Cmp(fixed.Zero) <= 0 || halfExtents.Y.Cmp(fixed.Zero) <= 0 || halfExtents.Z.Cmp(fixed.Zero) <= 0 {
		panic("physics: box half extents must be > 0")
	}

	return BoxBody{
		Motion:          NewDynamicMotionState(mass, position),
		HalfExtents:     halfExtents,
		InverseInertiaZ: computeBoxInverseInertiaZ(mass, halfExtents),
	}
}

func computeBoxInverseInertiaZ(mass fixed.Fixed, halfExtents geometry.Vector3) fixed.Fixed {
	width := halfExtents.X.Mul(fixed.FromInt(2))
	height := halfExtents.Y.Mul(fixed.FromInt(2))
	inertia := mass.Mul(width.Mul(width).Add(height.Mul(height))).Div(fixed.FromInt(12))
	if inertia == fixed.Zero {
		return fixed.Zero
	}
	return fixed.One.Div(inertia)
}

func rotateBoxOffsetAroundZ(offset geometry.Vector3, angle fixed.Fixed) geometry.Vector3 {
	sinAngle := fixed.Sin(angle)
	cosAngle := fixed.Cos(angle)

	return geometry.NewVector3(
		offset.X.Mul(cosAngle).Sub(offset.Y.Mul(sinAngle)),
		offset.X.Mul(sinAngle).Add(offset.Y.Mul(cosAngle)),
		offset.Z,
	)
}

func boxContactPointVelocity(linearVelocity geometry.Vector3, angularVelocityZ fixed.Fixed, offset geometry.Vector3) geometry.Vector3 {
	angularComponent := geometry.NewVector3(
		offset.Y.Neg().Mul(angularVelocityZ),
		offset.X.Mul(angularVelocityZ),
		fixed.Zero,
	)
	return linearVelocity.Add(angularComponent)
}

func boxPlaneCornerOffsets(halfExtents geometry.Vector3, angle fixed.Fixed) []geometry.Vector3 {
	localOffsets := []geometry.Vector3{
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y.Neg(), halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X, halfExtents.Y.Neg(), halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y.Neg(), halfExtents.Z),
		geometry.NewVector3(halfExtents.X, halfExtents.Y.Neg(), halfExtents.Z),
	}

	offsets := make([]geometry.Vector3, 0, len(localOffsets))
	for _, offset := range localOffsets {
		offsets = append(offsets, rotateBoxOffsetAroundZ(offset, angle))
	}
	return offsets
}

func integrateBoxRotation(body *BoxBody, dt fixed.Fixed) {
	if body == nil {
		return
	}

	body.RotationZ = body.RotationZ.Add(body.AngularVelocityZ.Mul(dt))
}

func clampUnitFixed(value fixed.Fixed) fixed.Fixed {
	if value.Cmp(fixed.Zero) < 0 {
		return fixed.Zero
	}
	if value.Cmp(fixed.One) > 0 {
		return fixed.One
	}
	return value
}

func applyBoxContactFriction(body *BoxBody, planeNormal geometry.Vector3) {
	if body == nil {
		return
	}

	friction := clampUnitFixed(body.Friction)
	if friction == fixed.Zero {
		return
	}

	tangentVelocity := body.Motion.Velocity.Sub(planeNormal.Scale(body.Motion.Velocity.Dot(planeNormal)))
	if tangentVelocity.LengthSquared() == fixed.Zero {
		return
	}

	body.Motion.Velocity = body.Motion.Velocity.Sub(tangentVelocity.Scale(friction))
	body.AngularVelocityZ = body.AngularVelocityZ.Mul(fixed.One.Sub(friction))
}

// Bounds returns the current axis-aligned bounds of the box body.
func (b BoxBody) Bounds() geometry.AxisAlignedBoundingBox {
	return geometry.NewAxisAlignedBoundingBox(
		b.Motion.Position.Sub(b.HalfExtents),
		b.Motion.Position.Add(b.HalfExtents),
	)
}

// StepBoxBodyWithGravityAndFloorOverride applies gravity and resolves the box against a flat floor strip.
func StepBoxBodyWithGravityAndFloorOverride(body *BoxBody, dt fixed.Fixed, gravity geometry.Vector3, floor geometry.AxisAlignedBoundingBox, contactRestitution fixed.Fixed) BoxStepResult {
	if body == nil {
		return BoxStepResult{}
	}

	body.Grounded = false
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateBoxRotation(body, dt)

	bounds := body.Bounds()
	if bounds.Max.X.Cmp(floor.Min.X) < 0 || bounds.Min.X.Cmp(floor.Max.X) > 0 ||
		bounds.Max.Z.Cmp(floor.Min.Z) < 0 || bounds.Min.Z.Cmp(floor.Max.Z) > 0 {
		return BoxStepResult{}
	}

	if bounds.Min.Y.Cmp(floor.Max.Y) >= 0 {
		return BoxStepResult{}
	}

	penetration := floor.Max.Y.Sub(bounds.Min.Y)
	if penetration.Cmp(fixed.Zero) <= 0 {
		return BoxStepResult{}
	}

	if contactRestitution.Cmp(fixed.Zero) < 0 {
		contactRestitution = fixed.Zero
	}
	if contactRestitution.Cmp(fixed.One) > 0 {
		contactRestitution = fixed.One
	}

	body.Motion.Position.Y = body.Motion.Position.Y.Add(penetration)
	if body.Motion.Velocity.Y.Cmp(fixed.Zero) < 0 {
		body.Motion.Velocity.Y = body.Motion.Velocity.Y.Neg().Mul(contactRestitution)
	}
	applyBoxContactFriction(body, geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero))
	body.Grounded = true

	return BoxStepResult{
		HadContact: true,
		LastContact: SphereTriangleContact{
			Hit:         true,
			Point:       geometry.NewVector3(body.Motion.Position.X, floor.Max.Y, body.Motion.Position.Z),
			Normal:      geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
			Penetration: penetration,
		},
	}
}

// StepBoxBodyWithGravityAndSideSlopeOverride applies gravity and resolves the box against a side slope that rises with +X.
func StepBoxBodyWithGravityAndSideSlopeOverride(body *BoxBody, dt fixed.Fixed, gravity geometry.Vector3, minX, maxX, minZ, maxZ, slopeRisePerRun, contactRestitution fixed.Fixed) BoxStepResult {
	if body == nil {
		return BoxStepResult{}
	}

	body.Grounded = false
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateBoxRotation(body, dt)

	bounds := body.Bounds()
	if bounds.Max.X.Cmp(minX) < 0 || bounds.Min.X.Cmp(maxX) > 0 ||
		bounds.Max.Z.Cmp(minZ) < 0 || bounds.Min.Z.Cmp(maxZ) > 0 {
		return BoxStepResult{}
	}

	if contactRestitution.Cmp(fixed.Zero) < 0 {
		contactRestitution = fixed.Zero
	}
	if contactRestitution.Cmp(fixed.One) > 0 {
		contactRestitution = fixed.One
	}

	normal := geometry.NewVector3(slopeRisePerRun.Neg(), fixed.One, fixed.Zero).Normalize()
	supportPoint := geometry.NewVector3(bounds.Max.X, bounds.Min.Y, body.Motion.Position.Z)
	planePoint := geometry.NewVector3(supportPoint.X, supportPoint.X.Mul(slopeRisePerRun), supportPoint.Z)
	separation := supportPoint.Sub(planePoint).Dot(normal)
	if separation.Cmp(fixed.Zero) >= 0 {
		return BoxStepResult{}
	}

	penetration := separation.Neg()
	body.Motion.Position = body.Motion.Position.Add(normal.Scale(penetration))

	vn := body.Motion.Velocity.Dot(normal)
	if vn.Cmp(fixed.Zero) < 0 {
		body.Motion.Velocity = body.Motion.Velocity.Sub(normal.Scale(vn.Mul(fixed.One.Add(contactRestitution))))
	}
	applyBoxContactFriction(body, normal)

	body.Grounded = true

	return BoxStepResult{
		HadContact: true,
		LastContact: SphereTriangleContact{
			Hit:         true,
			Point:       planePoint,
			Normal:      normal,
			Penetration: penetration,
		},
	}
}

// StepOrientedBoxBodyWithGravityAndPlaneOverride applies gravity and resolves a Z-rotated box against a static plane.
func StepOrientedBoxBodyWithGravityAndPlaneOverride(body *BoxBody, dt fixed.Fixed, gravity geometry.Vector3, planePoint, planeNormal geometry.Vector3, contactRestitution fixed.Fixed) BoxStepResult {
	if body == nil {
		return BoxStepResult{}
	}

	body.Grounded = false
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateBoxRotation(body, dt)

	if contactRestitution.Cmp(fixed.Zero) < 0 {
		contactRestitution = fixed.Zero
	}
	if contactRestitution.Cmp(fixed.One) > 0 {
		contactRestitution = fixed.One
	}

	offsets := boxPlaneCornerOffsets(body.HalfExtents, body.RotationZ)
	deepestOffset := geometry.ZeroVector3()
	minSeparation := fixed.Zero
	found := false

	for _, offset := range offsets {
		corner := body.Motion.Position.Add(offset)
		separation := corner.Sub(planePoint).Dot(planeNormal)
		if !found || separation.Cmp(minSeparation) < 0 {
			minSeparation = separation
			deepestOffset = offset
			found = true
		}
	}

	if !found || minSeparation.Cmp(fixed.Zero) >= 0 {
		return BoxStepResult{}
	}

	penetration := minSeparation.Neg()
	body.Motion.Position = body.Motion.Position.Add(planeNormal.Scale(penetration))

	contactOffset := deepestOffset
	contactPoint := body.Motion.Position.Add(contactOffset)
	relativeVelocity := boxContactPointVelocity(body.Motion.Velocity, body.AngularVelocityZ, contactOffset)
	normalVelocity := relativeVelocity.Dot(planeNormal)

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		rCrossN := contactOffset.X.Mul(planeNormal.Y).Sub(contactOffset.Y.Mul(planeNormal.X))
		angularTerm := rCrossN.Mul(rCrossN).Mul(body.InverseInertiaZ)
		denominator := body.Motion.InverseMass.Add(angularTerm)
		if denominator != fixed.Zero {
			impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(contactRestitution)).Div(denominator)
			impulse := planeNormal.Scale(impulseMagnitude)
			body.Motion.Velocity = body.Motion.Velocity.Add(impulse.Scale(body.Motion.InverseMass))
			body.AngularVelocityZ = body.AngularVelocityZ.Add(rCrossN.Mul(impulseMagnitude).Mul(body.InverseInertiaZ))
		}
	}
	applyBoxContactFriction(body, planeNormal)

	body.Grounded = planeNormal.Y.Cmp(fixed.Zero) > 0

	return BoxStepResult{
		HadContact: true,
		LastContact: SphereTriangleContact{
			Hit:         true,
			Point:       contactPoint,
			Normal:      planeNormal,
			Penetration: penetration,
		},
	}
}
