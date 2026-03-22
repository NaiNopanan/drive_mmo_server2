package physics

import (
	"math"

	"server2/internal/worldmesh"
)

const bodyCapsuleCCDSkin float32 = 0.01
const bodyCapsuleCCDMaxSteps = 64
const bodyCapsuleCCDBinarySteps = 12

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

	candidates := w.collectBodyCapsuleSweepCandidates(startCapsule, endCapsule)
	if len(candidates) == 0 {
		return bodyMapCCDHit{}, false
	}

	stepSize := maxf(startCapsule.radius*0.18, 0.05)
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
		hit, intersects := queryBodyCapsuleMapHitTriangles(bodyCapsuleFromVehicle(sample), candidates)
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
			midHit, midIntersects := queryBodyCapsuleMapHitTriangles(bodyCapsuleFromVehicle(midVehicle), candidates)
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

func (w *PhysicsWorld) collectBodyCapsuleSweepCandidates(startCapsule, endCapsule bodyCapsule) []worldmesh.Triangle {
	startMin, startMax := bodyCapsuleAABB(startCapsule)
	endMin, endMax := bodyCapsuleAABB(endCapsule)
	sweptMin, sweptMax := mergeAABB(startMin, startMax, endMin, endMax)

	candidates := make([]worldmesh.Triangle, 0, len(w.staticMesh.Triangles))
	for _, triangle := range w.staticMesh.Triangles {
		if !triangleCouldHitBodyOBB(sweptMin, sweptMax, triangle) {
			continue
		}
		candidates = append(candidates, triangle)
	}
	return candidates
}
