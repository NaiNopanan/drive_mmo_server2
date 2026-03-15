package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SweepRotatingOrientedBoxAxisAlignedBoundingBoxContact struct {
	Hit          bool
	TimeOfImpact fixed.Fixed
	Normal       geometry.Vector3
	Position     geometry.Vector3
	Orientation  Quaternion
	ContactPoint geometry.Vector3
}

func integrateQuaternionForSweep(orientation Quaternion, angularVelocity geometry.Vector3, dt fixed.Fixed) Quaternion {
	omega := Quaternion{
		X: angularVelocity.X,
		Y: angularVelocity.Y,
		Z: angularVelocity.Z,
	}
	qDot := omega.Mul(orientation).Scale(fixed.FromFraction(1, 2))
	return orientation.Add(qDot.Scale(dt)).Normalize()
}

func rigidBoxAABBOverlapContact(body RigidBoxBody3D, bounds geometry.AxisAlignedBoundingBox) (bool, geometry.Vector3, geometry.Vector3) {
	axesA := rigidBoxWorldAxes(body)
	centerA := body.Motion.Position
	centerB := bounds.Min.Add(bounds.Max).Scale(fixed.FromFraction(1, 2))
	extentsB := bounds.Max.Sub(bounds.Min).Scale(fixed.FromFraction(1, 2))
	worldAxesB := [3]geometry.Vector3{
		geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One),
	}
	centerDelta := centerB.Sub(centerA)

	bestAxis := geometry.ZeroVector3()
	bestPenetration := fixed.Zero
	found := false

	projectAABB := func(axis geometry.Vector3) fixed.Fixed {
		absDot := func(a fixed.Fixed) fixed.Fixed { return a.Abs() }
		return absDot(axis.Dot(worldAxesB[0])).Mul(extentsB.X).
			Add(absDot(axis.Dot(worldAxesB[1])).Mul(extentsB.Y)).
			Add(absDot(axis.Dot(worldAxesB[2])).Mul(extentsB.Z))
	}

	testAxis := func(axis geometry.Vector3) bool {
		if axis.LengthSquared().Cmp(fixed.FromFraction(1, 1000)) <= 0 {
			return true
		}
		axis = axis.Normalize()
		distance := centerDelta.Dot(axis).Abs()
		radiusA := rigidBoxProjectionRadius(body, axis, axesA)
		radiusB := projectAABB(axis)
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
			return false, geometry.ZeroVector3(), geometry.ZeroVector3()
		}
	}
	for _, axis := range worldAxesB {
		if !testAxis(axis) {
			return false, geometry.ZeroVector3(), geometry.ZeroVector3()
		}
	}
	for _, axisA := range axesA {
		for _, axisB := range worldAxesB {
			if !testAxis(axisA.Cross(axisB)) {
				return false, geometry.ZeroVector3(), geometry.ZeroVector3()
			}
		}
	}

	if !found {
		return false, geometry.ZeroVector3(), geometry.ZeroVector3()
	}

	return true, bestAxis, rigidBoxSupportPoint(body, bestAxis, axesA)
}

func SweepRotatingOrientedBoxAxisAlignedBoundingBox(
	body RigidBoxBody3D,
	dt fixed.Fixed,
	bounds geometry.AxisAlignedBoundingBox,
) SweepRotatingOrientedBoxAxisAlignedBoundingBoxContact {
	const coarseSteps = 16
	const binaryIterations = 8
	stepSize := fixed.One.Div(fixed.FromInt(coarseSteps))
	half := fixed.FromFraction(1, 2)
	startPosition := body.Motion.Position
	startOrientation := body.Orientation
	delta := body.Motion.Velocity.Scale(dt)

	positionAt := func(time fixed.Fixed) geometry.Vector3 {
		return startPosition.Add(delta.Scale(time))
	}
	orientationAt := func(time fixed.Fixed) Quaternion {
		return integrateQuaternionForSweep(startOrientation, body.AngularVelocity, dt.Mul(time))
	}
	overlapAt := func(time fixed.Fixed) (bool, geometry.Vector3, geometry.Vector3, geometry.Vector3, Quaternion) {
		position := positionAt(time)
		orientation := orientationAt(time)
		testBody := body
		testBody.Motion.Position = position
		testBody.Orientation = orientation
		hit, normal, contactPoint := rigidBoxAABBOverlapContact(testBody, bounds)
		return hit, normal, contactPoint, position, orientation
	}

	if hit, normal, contactPoint, position, orientation := overlapAt(fixed.Zero); hit {
		return SweepRotatingOrientedBoxAxisAlignedBoundingBoxContact{
			Hit:          true,
			TimeOfImpact: fixed.Zero,
			Normal:       normal,
			Position:     position,
			Orientation:  orientation,
			ContactPoint: contactPoint,
		}
	}

	previousTime := fixed.Zero
	previousHit := false
	for step := int64(1); step <= coarseSteps; step++ {
		currentTime := stepSize.Mul(fixed.FromInt(step))
		currentHit, _, _, _, _ := overlapAt(currentTime)
		if currentHit && !previousHit {
			low := previousTime
			high := currentTime
			for iteration := 0; iteration < binaryIterations; iteration++ {
				mid := low.Add(high).Mul(half)
				midHit, _, _, _, _ := overlapAt(mid)
				if midHit {
					high = mid
				} else {
					low = mid
				}
			}

			hit, normal, contactPoint, position, orientation := overlapAt(high)
			if hit {
				return SweepRotatingOrientedBoxAxisAlignedBoundingBoxContact{
					Hit:          true,
					TimeOfImpact: high,
					Normal:       normal,
					Position:     position,
					Orientation:  orientation,
					ContactPoint: contactPoint,
				}
			}
		}
		previousTime = currentTime
		previousHit = currentHit
	}

	return SweepRotatingOrientedBoxAxisAlignedBoundingBoxContact{}
}
