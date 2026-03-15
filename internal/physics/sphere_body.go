package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

const maxSphereTriangleResolvePasses = 4

// SphereBody is a reusable physics object for sphere-based motion and grounding.
type SphereBody struct {
	Motion      MotionState
	Radius      fixed.Fixed
	Restitution fixed.Fixed
	Friction    fixed.Fixed
	Grounded    bool
}

// SphereStepResult reports contact information produced by one physics step.
type SphereStepResult struct {
	HadContact  bool
	LastContact SphereTriangleContact
}

type sphereTriangleCandidate struct {
	Triangle geometry.Triangle
	Contact  SphereTriangleContact
}

// NewDynamicSphereBody creates a dynamic sphere body with linear motion state.
func NewDynamicSphereBody(mass, radius fixed.Fixed, position geometry.Vector3) SphereBody {
	if radius.Cmp(fixed.Zero) <= 0 {
		panic("physics: sphere radius must be > 0")
	}

	return SphereBody{
		Motion: NewDynamicMotionState(mass, position),
		Radius: radius,
	}
}

// StepSphereBody advances a sphere body and resolves it against triangles.
func StepSphereBody(body *SphereBody, dt fixed.Fixed, triangles []geometry.Triangle) SphereStepResult {
	if body == nil {
		return SphereStepResult{}
	}

	body.Grounded = false
	result := SphereStepResult{}

	StepLinearMotion(&body.Motion, dt)

	for pass := 0; pass < maxSphereTriangleResolvePasses; pass++ {
		candidate, ok := selectDeepestSphereTriangleContact(body.Motion.Position, body.Radius, triangles)
		if !ok {
			break
		}

		position := body.Motion.Position
		velocity := body.Motion.Velocity
		contact := SphereTriangleContact{}

		if body.Restitution.Cmp(fixed.Zero) > 0 {
			position, velocity, contact = ResolveSphereTriangleContactWithRestitution(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				candidate.Triangle,
				body.Restitution,
			)
		} else {
			position, velocity, contact = ResolveSphereTriangleContact(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				candidate.Triangle,
			)
		}

		body.Motion.Position = position
		body.Motion.Velocity = velocity
		result.HadContact = true
		result.LastContact = contact
	}

	if result.HadContact {
		body.Grounded = result.LastContact.Normal.Y.Cmp(fixed.Zero) > 0
	}

	return result
}

// StepSphereBodyWithGravity applies gravity and then advances the sphere body.
func StepSphereBodyWithGravity(body *SphereBody, dt fixed.Fixed, gravity geometry.Vector3, triangles []geometry.Triangle) SphereStepResult {
	if body == nil {
		return SphereStepResult{}
	}

	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	return StepSphereBody(body, dt, triangles)
}

// StepSphereBodyWithGravityAndRestitutionOverride applies gravity and advances the sphere body with an explicit contact restitution.
func StepSphereBodyWithGravityAndRestitutionOverride(body *SphereBody, dt fixed.Fixed, gravity geometry.Vector3, triangles []geometry.Triangle, contactRestitution fixed.Fixed) SphereStepResult {
	if body == nil {
		return SphereStepResult{}
	}

	originalRestitution := body.Restitution
	body.Restitution = contactRestitution
	ApplyForce(&body.Motion, ComputeGravityForce(body.Motion.Mass, gravity))
	result := StepSphereBody(body, dt, triangles)
	body.Restitution = originalRestitution
	return result
}

func selectDeepestSphereTriangleContact(center geometry.Vector3, radius fixed.Fixed, triangles []geometry.Triangle) (sphereTriangleCandidate, bool) {
	best := sphereTriangleCandidate{}
	found := false

	for _, triangle := range triangles {
		contact := FindSphereTriangleContact(center, radius, triangle)
		if !contact.Hit || contact.Penetration.Cmp(fixed.Zero) <= 0 {
			continue
		}

		if !found || contact.Penetration.Cmp(best.Contact.Penetration) > 0 {
			best = sphereTriangleCandidate{
				Triangle: triangle,
				Contact:  contact,
			}
			found = true
		}
	}

	return best, found
}
