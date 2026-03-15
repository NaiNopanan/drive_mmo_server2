package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type Quaternion struct {
	W fixed.Fixed
	X fixed.Fixed
	Y fixed.Fixed
	Z fixed.Fixed
}

func IdentityQuaternion() Quaternion {
	return Quaternion{W: fixed.One}
}

func NewQuaternionFromEulerXYZ(x, y, z fixed.Fixed) Quaternion {
	half := fixed.FromFraction(1, 2)
	cx := fixed.Cos(x.Mul(half))
	sx := fixed.Sin(x.Mul(half))
	cy := fixed.Cos(y.Mul(half))
	sy := fixed.Sin(y.Mul(half))
	cz := fixed.Cos(z.Mul(half))
	sz := fixed.Sin(z.Mul(half))

	return Quaternion{
		W: cx.Mul(cy).Mul(cz).Add(sx.Mul(sy).Mul(sz)),
		X: sx.Mul(cy).Mul(cz).Sub(cx.Mul(sy).Mul(sz)),
		Y: cx.Mul(sy).Mul(cz).Add(sx.Mul(cy).Mul(sz)),
		Z: cx.Mul(cy).Mul(sz).Sub(sx.Mul(sy).Mul(cz)),
	}.Normalize()
}

func (q Quaternion) Add(o Quaternion) Quaternion {
	return Quaternion{
		W: q.W.Add(o.W),
		X: q.X.Add(o.X),
		Y: q.Y.Add(o.Y),
		Z: q.Z.Add(o.Z),
	}
}

func (q Quaternion) Scale(s fixed.Fixed) Quaternion {
	return Quaternion{
		W: q.W.Mul(s),
		X: q.X.Mul(s),
		Y: q.Y.Mul(s),
		Z: q.Z.Mul(s),
	}
}

func (q Quaternion) Mul(o Quaternion) Quaternion {
	return Quaternion{
		W: q.W.Mul(o.W).Sub(q.X.Mul(o.X)).Sub(q.Y.Mul(o.Y)).Sub(q.Z.Mul(o.Z)),
		X: q.W.Mul(o.X).Add(q.X.Mul(o.W)).Add(q.Y.Mul(o.Z)).Sub(q.Z.Mul(o.Y)),
		Y: q.W.Mul(o.Y).Sub(q.X.Mul(o.Z)).Add(q.Y.Mul(o.W)).Add(q.Z.Mul(o.X)),
		Z: q.W.Mul(o.Z).Add(q.X.Mul(o.Y)).Sub(q.Y.Mul(o.X)).Add(q.Z.Mul(o.W)),
	}
}

func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{W: q.W, X: q.X.Neg(), Y: q.Y.Neg(), Z: q.Z.Neg()}
}

func (q Quaternion) Normalize() Quaternion {
	lengthSq := q.W.Mul(q.W).Add(q.X.Mul(q.X)).Add(q.Y.Mul(q.Y)).Add(q.Z.Mul(q.Z))
	if lengthSq == fixed.Zero {
		return IdentityQuaternion()
	}
	invLength := fixed.One.Div(lengthSq.Sqrt())
	return q.Scale(invLength)
}

func (q Quaternion) RotateVector(v geometry.Vector3) geometry.Vector3 {
	p := Quaternion{X: v.X, Y: v.Y, Z: v.Z}
	rotated := q.Mul(p).Mul(q.Conjugate())
	return geometry.NewVector3(rotated.X, rotated.Y, rotated.Z)
}

type RigidBoxBody3D struct {
	Motion             MotionState
	HalfExtents        geometry.Vector3
	Restitution        fixed.Fixed
	Grounded           bool
	Orientation        Quaternion
	AngularVelocity    geometry.Vector3
	InverseInertiaBody geometry.Vector3
}

type RigidBoxStepResult struct {
	HadContact  bool
	LastContact SphereTriangleContact
}

func NewRigidBoxBody3D(mass fixed.Fixed, halfExtents, position geometry.Vector3) RigidBoxBody3D {
	if halfExtents.X.Cmp(fixed.Zero) <= 0 || halfExtents.Y.Cmp(fixed.Zero) <= 0 || halfExtents.Z.Cmp(fixed.Zero) <= 0 {
		panic("physics: rigid box half extents must be > 0")
	}

	return RigidBoxBody3D{
		Motion:             NewDynamicMotionState(mass, position),
		HalfExtents:        halfExtents,
		Orientation:        IdentityQuaternion(),
		InverseInertiaBody: computeRigidBoxInverseInertiaBody(mass, halfExtents),
	}
}

func computeRigidBoxInverseInertiaBody(mass fixed.Fixed, halfExtents geometry.Vector3) geometry.Vector3 {
	two := fixed.FromInt(2)
	x := halfExtents.X.Mul(two)
	y := halfExtents.Y.Mul(two)
	z := halfExtents.Z.Mul(two)
	twelve := fixed.FromInt(12)

	ix := mass.Mul(y.Mul(y).Add(z.Mul(z))).Div(twelve)
	iy := mass.Mul(x.Mul(x).Add(z.Mul(z))).Div(twelve)
	iz := mass.Mul(x.Mul(x).Add(y.Mul(y))).Div(twelve)

	inv := func(v fixed.Fixed) fixed.Fixed {
		if v == fixed.Zero {
			return fixed.Zero
		}
		return fixed.One.Div(v)
	}

	return geometry.NewVector3(inv(ix), inv(iy), inv(iz))
}

func integrateRigidBoxOrientation(body *RigidBoxBody3D, dt fixed.Fixed) {
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

func (b RigidBoxBody3D) WorldCorners() []geometry.Vector3 {
	localCorners := []geometry.Vector3{
		geometry.NewVector3(b.HalfExtents.X.Neg(), b.HalfExtents.Y.Neg(), b.HalfExtents.Z.Neg()),
		geometry.NewVector3(b.HalfExtents.X, b.HalfExtents.Y.Neg(), b.HalfExtents.Z.Neg()),
		geometry.NewVector3(b.HalfExtents.X, b.HalfExtents.Y, b.HalfExtents.Z.Neg()),
		geometry.NewVector3(b.HalfExtents.X.Neg(), b.HalfExtents.Y, b.HalfExtents.Z.Neg()),
		geometry.NewVector3(b.HalfExtents.X.Neg(), b.HalfExtents.Y.Neg(), b.HalfExtents.Z),
		geometry.NewVector3(b.HalfExtents.X, b.HalfExtents.Y.Neg(), b.HalfExtents.Z),
		geometry.NewVector3(b.HalfExtents.X, b.HalfExtents.Y, b.HalfExtents.Z),
		geometry.NewVector3(b.HalfExtents.X.Neg(), b.HalfExtents.Y, b.HalfExtents.Z),
	}

	corners := make([]geometry.Vector3, 0, len(localCorners))
	for _, corner := range localCorners {
		corners = append(corners, b.Motion.Position.Add(b.Orientation.RotateVector(corner)))
	}
	return corners
}

func applyInverseInertiaWorld(body RigidBoxBody3D, vector geometry.Vector3) geometry.Vector3 {
	local := body.Orientation.Conjugate().RotateVector(vector)
	local = geometry.NewVector3(
		local.X.Mul(body.InverseInertiaBody.X),
		local.Y.Mul(body.InverseInertiaBody.Y),
		local.Z.Mul(body.InverseInertiaBody.Z),
	)
	return body.Orientation.RotateVector(local)
}

func AdvanceRigidBoxBody3D(body *RigidBoxBody3D, dt fixed.Fixed, gravity geometry.Vector3) {
	if body == nil {
		return
	}

	body.Grounded = false
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	StepLinearMotion(&body.Motion, dt)
	integrateRigidBoxOrientation(body, dt)
}

func ResolveRigidBoxBody3DPlaneOverride(body *RigidBoxBody3D, planePoint, planeNormal geometry.Vector3, contactRestitution fixed.Fixed) RigidBoxStepResult {
	if body == nil {
		return RigidBoxStepResult{}
	}

	if contactRestitution.Cmp(fixed.Zero) < 0 {
		contactRestitution = fixed.Zero
	}
	if contactRestitution.Cmp(fixed.One) > 0 {
		contactRestitution = fixed.One
	}

	corners := body.WorldCorners()
	deepestPoint := geometry.ZeroVector3()
	minSeparation := fixed.Zero
	found := false

	for _, corner := range corners {
		separation := corner.Sub(planePoint).Dot(planeNormal)
		if !found || separation.Cmp(minSeparation) < 0 {
			minSeparation = separation
			deepestPoint = corner
			found = true
		}
	}

	if !found || minSeparation.Cmp(fixed.Zero) >= 0 {
		return RigidBoxStepResult{}
	}

	penetration := minSeparation.Neg()
	body.Motion.Position = body.Motion.Position.Add(planeNormal.Scale(penetration))

	contactPoint := deepestPoint.Add(planeNormal.Scale(penetration))
	r := contactPoint.Sub(body.Motion.Position)
	relativeVelocity := body.Motion.Velocity.Add(body.AngularVelocity.Cross(r))
	normalVelocity := relativeVelocity.Dot(planeNormal)

	if normalVelocity.Cmp(fixed.Zero) < 0 {
		rCrossN := r.Cross(planeNormal)
		angularContribution := applyInverseInertiaWorld(*body, rCrossN).Cross(r)
		denominator := body.Motion.InverseMass.Add(planeNormal.Dot(angularContribution))
		if denominator != fixed.Zero {
			impulseMagnitude := normalVelocity.Neg().Mul(fixed.One.Add(contactRestitution)).Div(denominator)
			impulse := planeNormal.Scale(impulseMagnitude)
			body.Motion.Velocity = body.Motion.Velocity.Add(impulse.Scale(body.Motion.InverseMass))
			body.AngularVelocity = body.AngularVelocity.Add(applyInverseInertiaWorld(*body, r.Cross(impulse)))
		}
	}

	body.Grounded = planeNormal.Y.Cmp(fixed.Zero) > 0

	return RigidBoxStepResult{
		HadContact: true,
		LastContact: SphereTriangleContact{
			Hit:         true,
			Point:       contactPoint,
			Normal:      planeNormal,
			Penetration: penetration,
		},
	}
}

func StepRigidBoxBody3DWithGravityAndPlaneOverride(body *RigidBoxBody3D, dt fixed.Fixed, gravity geometry.Vector3, planePoint, planeNormal geometry.Vector3, contactRestitution fixed.Fixed) RigidBoxStepResult {
	if body == nil {
		return RigidBoxStepResult{}
	}

	AdvanceRigidBoxBody3D(body, dt, gravity)
	return ResolveRigidBoxBody3DPlaneOverride(body, planePoint, planeNormal, contactRestitution)
}
