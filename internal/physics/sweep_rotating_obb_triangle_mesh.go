package physics

import (
	"math"

	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SweepRotatingOrientedBoxTriangleMeshContact struct {
	Hit          bool
	TimeOfImpact fixed.Fixed
	Triangle     geometry.Triangle
	Normal       geometry.Vector3
	Position     geometry.Vector3
	Orientation  Quaternion
	ContactPoint geometry.Vector3
}

func rigidBoxAtTime(body RigidBoxBody3D, dt, time fixed.Fixed) RigidBoxBody3D {
	result := body
	result.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(dt.Mul(time)))
	result.Orientation = integrateQuaternionForSweep(body.Orientation, body.AngularVelocity, dt.Mul(time))
	return result
}

func segmentTriangleIntersection(start, end geometry.Vector3, triangle geometry.Triangle) (bool, fixed.Fixed, geometry.Vector3) {
	normal := triangle.Normal()
	if normal.LengthSquared() == fixed.Zero {
		return false, fixed.Zero, geometry.ZeroVector3()
	}

	direction := end.Sub(start)
	denominator := direction.Dot(normal)
	if denominator == fixed.Zero {
		return false, fixed.Zero, geometry.ZeroVector3()
	}

	const fixedScale = float64(uint64(1) << fixed.FracBits)
	numeratorFloat := float64(triangle.A.Sub(start).Dot(normal).Raw()) / fixedScale
	denominatorFloat := float64(denominator.Raw()) / fixedScale
	if math.Abs(denominatorFloat) < 1e-9 {
		return false, fixed.Zero, geometry.ZeroVector3()
	}
	timeFloat := numeratorFloat / denominatorFloat
	if timeFloat < 0 || timeFloat > 1 {
		return false, fixed.Zero, geometry.ZeroVector3()
	}
	time := fixed.FromRaw(int64(timeFloat * fixedScale))

	point := start.Add(direction.Scale(time))
	closest := triangle.ClosestPointTo(point)
	if point.Sub(closest).LengthSquared().Cmp(fixed.FromFraction(1, 1000000)) > 0 {
		return false, fixed.Zero, geometry.ZeroVector3()
	}

	return true, time, point
}

func rotatingBoxTriangleIntervalHit(body RigidBoxBody3D, dt, startTime, endTime fixed.Fixed, triangles []geometry.Triangle) (bool, fixed.Fixed, geometry.Triangle, geometry.Vector3, geometry.Vector3) {
	startBody := rigidBoxAtTime(body, dt, startTime)
	endBody := rigidBoxAtTime(body, dt, endTime)
	startCorners := startBody.WorldCorners()
	endCorners := endBody.WorldCorners()

	bestTime := fixed.Zero
	bestTriangle := geometry.Triangle{}
	bestNormal := geometry.ZeroVector3()
	bestPoint := geometry.ZeroVector3()
	found := false

	for triangleIndex, triangle := range triangles {
		triangleNormal := triangle.Normal()
		for cornerIndex := range startCorners {
			hit, localTime, point := segmentTriangleIntersection(startCorners[cornerIndex], endCorners[cornerIndex], triangle)
			if !hit {
				continue
			}
			globalTime := startTime.Add(endTime.Sub(startTime).Mul(localTime))
			if !found || globalTime.Cmp(bestTime) < 0 {
				bestTime = globalTime
				bestTriangle = triangles[triangleIndex]
				bestNormal = triangleNormal
				bestPoint = point
				found = true
			}
		}
	}

	return found, bestTime, bestTriangle, bestNormal, bestPoint
}

func SweepRotatingOrientedBoxTriangleMesh(
	body RigidBoxBody3D,
	dt fixed.Fixed,
	triangles []geometry.Triangle,
) SweepRotatingOrientedBoxTriangleMeshContact {
	const coarseSteps = 32
	const binaryIterations = 8
	stepSize := fixed.One.Div(fixed.FromInt(coarseSteps))
	half := fixed.FromFraction(1, 2)

	low := fixed.Zero
	for step := int64(1); step <= coarseSteps; step++ {
		high := stepSize.Mul(fixed.FromInt(step))
		hit, hitTime, hitTriangle, hitNormal, hitPoint := rotatingBoxTriangleIntervalHit(body, dt, low, high, triangles)
		if hit {
			refinedLow := low
			refinedHigh := hitTime
			for iteration := 0; iteration < binaryIterations; iteration++ {
				mid := refinedLow.Add(refinedHigh).Mul(half)
				midHit, _, _, _, _ := rotatingBoxTriangleIntervalHit(body, dt, refinedLow, mid, triangles)
				if midHit {
					refinedHigh = mid
				} else {
					refinedLow = mid
				}
			}
			finalBody := rigidBoxAtTime(body, dt, refinedHigh)
			return SweepRotatingOrientedBoxTriangleMeshContact{
				Hit:          true,
				TimeOfImpact: refinedHigh,
				Triangle:     hitTriangle,
				Normal:       hitNormal,
				Position:     finalBody.Motion.Position,
				Orientation:  finalBody.Orientation,
				ContactPoint: hitPoint,
			}
		}
		low = high
	}

	return SweepRotatingOrientedBoxTriangleMeshContact{}
}
