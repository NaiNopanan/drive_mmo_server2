package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SweepSphereTriangleMeshContact struct {
	Hit          bool
	TimeOfImpact fixed.Fixed
	Triangle     geometry.Triangle
	Normal       geometry.Vector3
	Position     geometry.Vector3
	ContactPoint geometry.Vector3
}

func SweepSphereTriangleMesh(
	startPosition geometry.Vector3,
	velocity geometry.Vector3,
	radius fixed.Fixed,
	dt fixed.Fixed,
	triangles []geometry.Triangle,
) SweepSphereTriangleMeshContact {
	delta := velocity.Scale(dt)
	best := SweepSphereTriangleMeshContact{}
	found := false
	const coarseSteps = 16
	const binaryIterations = 8
	stepSize := fixed.One.Div(fixed.FromInt(coarseSteps))
	half := fixed.FromFraction(1, 2)

	for _, triangle := range triangles {
		initialContact := FindSphereTriangleContact(startPosition, radius, triangle)
		if initialContact.Hit {
			best = SweepSphereTriangleMeshContact{
				Hit:          true,
				TimeOfImpact: fixed.Zero,
				Triangle:     triangle,
				Normal:       initialContact.Normal,
				Position:     startPosition,
				ContactPoint: initialContact.Point,
			}
			found = true
			continue
		}

		previousTime := fixed.Zero
		previousHit := false
		hitFound := false

		for step := int64(1); step <= coarseSteps; step++ {
			currentTime := stepSize.Mul(fixed.FromInt(step))
			currentPosition := startPosition.Add(delta.Scale(currentTime))
			currentHit := FindSphereTriangleContact(currentPosition, radius, triangle).Hit
			if currentHit && !previousHit {
				low := previousTime
				high := currentTime
				for iteration := 0; iteration < binaryIterations; iteration++ {
					mid := low.Add(high).Mul(half)
					midPosition := startPosition.Add(delta.Scale(mid))
					if FindSphereTriangleContact(midPosition, radius, triangle).Hit {
						high = mid
					} else {
						low = mid
					}
				}

				position := startPosition.Add(delta.Scale(high))
				contact := FindSphereTriangleContact(position, radius, triangle)
				if !contact.Hit {
					break
				}
				if !found || high.Cmp(best.TimeOfImpact) < 0 {
					best = SweepSphereTriangleMeshContact{
						Hit:          true,
						TimeOfImpact: high,
						Triangle:     triangle,
						Normal:       contact.Normal,
						Position:     position,
						ContactPoint: contact.Point,
					}
					found = true
				}
				hitFound = true
				break
			}
			previousTime = currentTime
			previousHit = currentHit
		}

		if hitFound {
			continue
		}
	}

	return best
}
