package physics

import "math"

const bodyCapsuleCCDSkin float32 = 0.01
const bodyCapsuleCCDMaxSteps = 24
const bodyCapsuleCCDBinarySteps = 8

func (w *PhysicsWorld) queryBodyCapsuleMapCCD(previous, current VehicleBody) (bodyMapCCDHit, bool) {
	if len(w.staticMesh.Triangles) == 0 {
		return bodyMapCCDHit{}, false
	}

	startHit, startIntersects := w.queryBodyCapsuleMapHit(previous)
	if startIntersects {
		return bodyMapCCDHit{
			Time:   0,
			Normal: startHit.Normal,
		}, true
	}

	startCapsule := bodyCapsuleFromVehicle(previous)
	endCapsule := bodyCapsuleFromVehicle(current)
	maxEndpointTravel := float32(math.Sqrt(float64(maxf(
		startCapsule.start.Sub(endCapsule.start).LengthSquared(),
		startCapsule.end.Sub(endCapsule.end).LengthSquared(),
	))))
	if maxEndpointTravel <= bodyCapsuleEpsilon {
		return bodyMapCCDHit{}, false
	}

	stepSize := maxf(startCapsule.radius*0.35, 0.12)
	stepCount := int(math.Ceil(float64(maxEndpointTravel / stepSize)))
	if stepCount < 1 {
		stepCount = 1
	}
	if stepCount > bodyCapsuleCCDMaxSteps {
		stepCount = bodyCapsuleCCDMaxSteps
	}

	previousT := float32(0)
	for step := 1; step <= stepCount; step++ {
		currentT := float32(step) / float32(stepCount)
		sample := interpolateVehiclePose(previous, current, currentT)
		hit, intersects := w.queryBodyCapsuleMapHit(sample)
		if !intersects {
			previousT = currentT
			continue
		}

		low := previousT
		high := currentT
		bestHit := hit
		for iteration := 0; iteration < bodyCapsuleCCDBinarySteps; iteration++ {
			mid := (low + high) * 0.5
			midVehicle := interpolateVehiclePose(previous, current, mid)
			midHit, midIntersects := w.queryBodyCapsuleMapHit(midVehicle)
			if midIntersects {
				high = mid
				bestHit = midHit
			} else {
				low = mid
			}
		}

		return bodyMapCCDHit{
			Time:   high,
			Normal: bestHit.Normal,
		}, true
	}

	return bodyMapCCDHit{}, false
}
