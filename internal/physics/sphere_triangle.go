package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

// SphereTriangleContact describes overlap data between a sphere and one triangle.
type SphereTriangleContact struct {
	Hit         bool
	Point       geometry.Vector3
	Normal      geometry.Vector3
	Penetration fixed.Fixed
}

// FindSphereTriangleContact computes contact information for a sphere against a triangle.
func FindSphereTriangleContact(center geometry.Vector3, radius fixed.Fixed, tri geometry.Triangle) SphereTriangleContact {
	cp := tri.ClosestPointTo(center)
	delta := center.Sub(cp)

	distSq := delta.Dot(delta)
	radiusSq := radius.Mul(radius)
	if distSq.Cmp(radiusSq) > 0 {
		return SphereTriangleContact{}
	}

	var normal geometry.Vector3
	if distSq == fixed.Zero {
		normal = tri.Normal()
	} else {
		dist := distSq.Sqrt()
		if dist == fixed.Zero {
			normal = tri.Normal()
		} else {
			normal = delta.Scale(fixed.One.Div(dist))
		}
	}

	dist := fixed.Zero
	if distSq != fixed.Zero {
		dist = distSq.Sqrt()
	}

	penetration := radius.Sub(dist)
	if penetration.Cmp(fixed.Zero) < 0 {
		return SphereTriangleContact{}
	}

	return SphereTriangleContact{
		Hit:         true,
		Point:       cp,
		Normal:      normal,
		Penetration: penetration,
	}
}

// ResolveSphereTriangleContact pushes the sphere out of the triangle and removes inward normal velocity.
func ResolveSphereTriangleContact(center, velocity geometry.Vector3, radius fixed.Fixed, tri geometry.Triangle) (geometry.Vector3, geometry.Vector3, SphereTriangleContact) {
	return ResolveSphereTriangleContactWithRestitution(center, velocity, radius, tri, fixed.Zero)
}

// ResolveSphereTriangleContactWithRestitution pushes the sphere out of the triangle and applies a bounce response.
func ResolveSphereTriangleContactWithRestitution(center, velocity geometry.Vector3, radius fixed.Fixed, tri geometry.Triangle, restitution fixed.Fixed) (geometry.Vector3, geometry.Vector3, SphereTriangleContact) {
	contact := FindSphereTriangleContact(center, radius, tri)
	if !contact.Hit || contact.Penetration.Cmp(fixed.Zero) <= 0 {
		return center, velocity, contact
	}

	if restitution.Cmp(fixed.Zero) < 0 {
		restitution = fixed.Zero
	}
	if restitution.Cmp(fixed.One) > 0 {
		restitution = fixed.One
	}

	center = center.Add(contact.Normal.Scale(contact.Penetration))

	vn := velocity.Dot(contact.Normal)
	if vn.Cmp(fixed.Zero) < 0 {
		velocity = velocity.Sub(contact.Normal.Scale(vn.Mul(fixed.One.Add(restitution))))
	}

	return center, velocity, contact
}
