package scenario

import (
	"sort"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func integrateScenarioRigidBoxOrientation(orientation physics.Quaternion, angularVelocity geometry.Vector3, dt fixed.Fixed) physics.Quaternion {
	omega := physics.Quaternion{
		X: angularVelocity.X,
		Y: angularVelocity.Y,
		Z: angularVelocity.Z,
	}
	qDot := omega.Mul(orientation).Scale(fixed.FromFraction(1, 2))
	return orientation.Add(qDot.Scale(dt)).Normalize()
}

type broadphaseCellKey struct {
	X int64
	Y int64
	Z int64
}

type mixedRigidBroadphaseBucket struct {
	SphereIndices []int
	BoxIndices    []int
}

type mixedRigidBroadphasePairs struct {
	SphereSphere [][2]int
	BoxBox       [][2]int
	SphereBox    [][2]int
	CellCount    int
	Cells        []geometry.AxisAlignedBoundingBox
}

func fixedFloorDiv(value, divisor fixed.Fixed) int64 {
	raw := value.Raw()
	step := divisor.Raw()
	quotient := raw / step
	remainder := raw % step
	if remainder != 0 && raw < 0 {
		quotient--
	}
	return quotient
}

func addMixedRigidPair(pairMap map[uint64]struct{}, first, second int, target *[][2]int) {
	if first > second {
		first, second = second, first
	}
	key := (uint64(uint32(first)) << 32) | uint64(uint32(second))
	if _, exists := pairMap[key]; exists {
		return
	}
	pairMap[key] = struct{}{}
	*target = append(*target, [2]int{first, second})
}

func sphereBroadphaseRadius(body physics.RigidSphereBody3D) fixed.Fixed {
	return body.Radius
}

func boxBroadphaseRadius(body physics.RigidBoxBody3D) fixed.Fixed {
	return body.HalfExtents.Length()
}

func buildMixedRigidBroadphasePairs(spheres []physics.RigidSphereBody3D, boxes []physics.RigidBoxBody3D, cellSize fixed.Fixed) mixedRigidBroadphasePairs {
	buckets := make(map[broadphaseCellKey]*mixedRigidBroadphaseBucket)

	insert := func(position geometry.Vector3, radius fixed.Fixed, isSphere bool, index int) {
		minX := fixedFloorDiv(position.X.Sub(radius), cellSize)
		maxX := fixedFloorDiv(position.X.Add(radius), cellSize)
		minY := fixedFloorDiv(position.Y.Sub(radius), cellSize)
		maxY := fixedFloorDiv(position.Y.Add(radius), cellSize)
		minZ := fixedFloorDiv(position.Z.Sub(radius), cellSize)
		maxZ := fixedFloorDiv(position.Z.Add(radius), cellSize)

		for x := minX; x <= maxX; x++ {
			for y := minY; y <= maxY; y++ {
				for z := minZ; z <= maxZ; z++ {
					key := broadphaseCellKey{X: x, Y: y, Z: z}
					bucket := buckets[key]
					if bucket == nil {
						bucket = &mixedRigidBroadphaseBucket{}
						buckets[key] = bucket
					}
					if isSphere {
						bucket.SphereIndices = append(bucket.SphereIndices, index)
					} else {
						bucket.BoxIndices = append(bucket.BoxIndices, index)
					}
				}
			}
		}
	}

	for index, sphere := range spheres {
		insert(sphere.Motion.Position, sphereBroadphaseRadius(sphere), true, index)
	}
	for index, box := range boxes {
		insert(box.Motion.Position, boxBroadphaseRadius(box), false, index)
	}

	sphereSpherePairs := make([][2]int, 0)
	boxBoxPairs := make([][2]int, 0)
	sphereBoxPairs := make([][2]int, 0)
	cells := make([]geometry.AxisAlignedBoundingBox, 0, len(buckets))
	sphereSphereSeen := make(map[uint64]struct{})
	boxBoxSeen := make(map[uint64]struct{})
	sphereBoxSeen := make(map[uint64]struct{})
	keys := make([]broadphaseCellKey, 0, len(buckets))

	for key := range buckets {
		keys = append(keys, key)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].X != keys[j].X {
			return keys[i].X < keys[j].X
		}
		if keys[i].Y != keys[j].Y {
			return keys[i].Y < keys[j].Y
		}
		return keys[i].Z < keys[j].Z
	})

	for _, key := range keys {
		bucket := buckets[key]
		min := geometry.NewVector3(
			cellSize.Mul(fixed.FromInt(key.X)),
			cellSize.Mul(fixed.FromInt(key.Y)),
			cellSize.Mul(fixed.FromInt(key.Z)),
		)
		max := min.Add(geometry.NewVector3(cellSize, cellSize, cellSize))
		cells = append(cells, geometry.NewAxisAlignedBoundingBox(min, max))

		for first := 0; first < len(bucket.SphereIndices); first++ {
			for second := first + 1; second < len(bucket.SphereIndices); second++ {
				addMixedRigidPair(sphereSphereSeen, bucket.SphereIndices[first], bucket.SphereIndices[second], &sphereSpherePairs)
			}
		}
		for first := 0; first < len(bucket.BoxIndices); first++ {
			for second := first + 1; second < len(bucket.BoxIndices); second++ {
				addMixedRigidPair(boxBoxSeen, bucket.BoxIndices[first], bucket.BoxIndices[second], &boxBoxPairs)
			}
		}
		for _, sphereIndex := range bucket.SphereIndices {
			for _, boxIndex := range bucket.BoxIndices {
				key := (uint64(uint32(sphereIndex)) << 32) | uint64(uint32(boxIndex))
				if _, exists := sphereBoxSeen[key]; exists {
					continue
				}
				sphereBoxSeen[key] = struct{}{}
				sphereBoxPairs = append(sphereBoxPairs, [2]int{sphereIndex, boxIndex})
			}
		}
	}

	return mixedRigidBroadphasePairs{
		SphereSphere: sphereSpherePairs,
		BoxBox:       boxBoxPairs,
		SphereBox:    sphereBoxPairs,
		CellCount:    len(buckets),
		Cells:        cells,
	}
}

func shouldUseRigidSphereCCD(body physics.RigidSphereBody3D) bool {
	if !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		return false
	}
	threshold := body.CCDVelocityThreshold
	if threshold == fixed.Zero {
		threshold = body.Radius
	}
	return body.Motion.Velocity.Length().Cmp(threshold) >= 0
}

func shouldUseRigidBoxCCD(body physics.RigidBoxBody3D) bool {
	if !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		return false
	}
	threshold := body.CCDVelocityThreshold
	if threshold == fixed.Zero {
		threshold = body.HalfExtents.Length()
	}
	return body.Motion.Velocity.Length().Cmp(threshold) >= 0
}

func shouldUseRigidBoxCCDWithAngularRisk(body physics.RigidBoxBody3D) (bool, bool) {
	if !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		return false, false
	}

	linearThreshold := body.CCDVelocityThreshold
	if linearThreshold == fixed.Zero {
		linearThreshold = body.HalfExtents.Length()
	}
	if body.Motion.Velocity.Length().Cmp(linearThreshold) >= 0 {
		return true, false
	}

	angularThreshold := body.CCDAngularSweepThreshold
	if angularThreshold == fixed.Zero {
		angularThreshold = body.HalfExtents.Length()
	}
	angularSweepRisk := body.AngularVelocity.Length().Mul(body.HalfExtents.Length())
	if angularSweepRisk.Cmp(angularThreshold) >= 0 {
		return true, true
	}

	return false, false
}

func cheapPrecheckRigidBoxMeshCCD(body physics.RigidBoxBody3D, triangles []geometry.Triangle) bool {
	boundingRadius := body.HalfExtents.Length()
	contact := physics.SweepSphereTriangleMesh(
		body.Motion.Position,
		body.Motion.Velocity,
		boundingRadius,
		physics.DefaultTimeStep,
		triangles,
	)
	return contact.Hit
}

func updateRigidSphereSleepState(body *physics.RigidSphereBody3D) {
	if body == nil {
		return
	}

	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	if linear.Cmp(body.SleepLinearThreshold) <= 0 && angular.Cmp(body.SleepAngularThreshold) <= 0 {
		body.SleepTickCount++
		if body.SleepTickCount >= body.SleepTickThreshold {
			body.Sleeping = true
			body.Motion.Velocity = geometry.ZeroVector3()
			body.AngularVelocity = geometry.ZeroVector3()
		}
		return
	}

	body.Sleeping = false
	body.SleepTickCount = 0
}

func updateRigidBoxSleepState(body *physics.RigidBoxBody3D) {
	if body == nil {
		return
	}

	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	if linear.Cmp(body.SleepLinearThreshold) <= 0 && angular.Cmp(body.SleepAngularThreshold) <= 0 {
		body.SleepTickCount++
		if body.SleepTickCount >= body.SleepTickThreshold {
			body.Sleeping = true
			body.Motion.Velocity = geometry.ZeroVector3()
			body.AngularVelocity = geometry.ZeroVector3()
		}
		return
	}

	body.Sleeping = false
	body.SleepTickCount = 0
}

func wakeRigidSphere(body *physics.RigidSphereBody3D) {
	if body == nil {
		return
	}
	body.Sleeping = false
	body.SleepTickCount = 0
}

func wakeRigidBox(body *physics.RigidBoxBody3D) {
	if body == nil {
		return
	}
	body.Sleeping = false
	body.SleepTickCount = 0
}

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxOptimizedScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	state.BroadphaseCellCount = 0
	state.SphereSphereCandidateCount = 0
	state.BoxBoxCandidateCount = 0
	state.SphereBoxCandidateCount = 0
	state.SphereSphereHitCount = 0
	state.BoxBoxHitCount = 0
	state.SphereBoxHitCount = 0
	state.BroadphaseDebugCells = nil

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		physics.ApplyForce(&state.RigidSpheres[index].Motion, physics.ComputeGravityForce(state.RigidSpheres[index].Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(&state.RigidSpheres[index], physics.DefaultTimeStep)
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	for index := range state.RigidBoxes {
		physics.AdvanceRigidBoxBody3D(&state.RigidBoxes[index], physics.DefaultTimeStep, physics.StandardGravity)
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	for pass := 0; pass < 2; pass++ {
		for _, pair := range pairs.SphereSphere {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			contact := physics.ResolveRigidSphereSphereContactWithFriction(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
			)
			if contact.Hit {
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
			}
		}

		for _, pair := range pairs.BoxBox {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
			if contact.Hit {
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
			}
		}

		for _, pair := range pairs.SphereBox {
			sphereIndex := pair[0]
			boxIndex := pair[1]
			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
			)
			if contact.Hit {
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
			}
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxAllCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	state.BroadphaseCellCount = 0
	state.SphereSphereCandidateCount = 0
	state.BoxBoxCandidateCount = 0
	state.SphereBoxCandidateCount = 0
	state.SphereSphereHitCount = 0
	state.BoxBoxHitCount = 0
	state.SphereBoxHitCount = 0
	state.BroadphaseDebugCells = nil

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.UseCCD && body.CCDMode == physics.CCDModeSweepTriangleMesh {
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepSphereTriangleMesh(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				physics.DefaultTimeStep,
				state.GroundTriangles,
			)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
				if normalVelocity.Cmp(fixed.Zero) < 0 {
					body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
				}
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
			} else {
				physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
			physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.UseCCD && body.CCDMode == physics.CCDModeSweepRotatingOrientedBoxTriangleMesh {
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepRotatingOrientedBoxTriangleMesh(*body, physics.DefaultTimeStep, state.GroundTriangles)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				body.Orientation = contact.Orientation
				result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
				if index < len(state.LastContacts) {
					state.LastContacts[index] = result.LastContact
				}
			} else {
				physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	for pass := 0; pass < 2; pass++ {
		for _, pair := range pairs.SphereSphere {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			contact := physics.ResolveRigidSphereSphereContactWithFriction(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
			)
			if contact.Hit {
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
			}
		}

		for _, pair := range pairs.BoxBox {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
			if contact.Hit {
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
			}
		}

		for _, pair := range pairs.SphereBox {
			sphereIndex := pair[0]
			boxIndex := pair[1]
			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
			)
			if contact.Hit {
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
			}
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxHybridCCDOptimizedScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	state.BroadphaseCellCount = 0
	state.SphereSphereCandidateCount = 0
	state.BoxBoxCandidateCount = 0
	state.SphereBoxCandidateCount = 0
	state.SphereSphereHitCount = 0
	state.BoxBoxHitCount = 0
	state.SphereBoxHitCount = 0
	state.BroadphaseDebugCells = nil
	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	state.ActiveCCDSphereCount = 0
	state.ActiveCCDBoxCount = 0
	state.ActiveDiscreteSphereCount = 0
	state.ActiveDiscreteBoxCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD := shouldUseRigidSphereCCD(*body)
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepSphereTriangleMesh(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				physics.DefaultTimeStep,
				state.GroundTriangles,
			)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
				if normalVelocity.Cmp(fixed.Zero) < 0 {
					body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
				}
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
			} else {
				physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteSphereCount++
			physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
			physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(body)
		if body.Sleeping {
			state.EverSleptSphere = true
			state.SleepingSphereCount++
		}
	}

	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}

		useCCD := shouldUseRigidBoxCCD(*body)
		if useCCD {
			state.ActiveCCDBoxCount++
			state.EverActivatedCCDBox = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepRotatingOrientedBoxTriangleMesh(*body, physics.DefaultTimeStep, state.GroundTriangles)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				body.Orientation = contact.Orientation
				result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
				if index < len(state.LastContacts) {
					state.LastContacts[index] = result.LastContact
				}
			} else {
				physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteBoxCount++
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(body)
		if body.Sleeping {
			state.EverSleptBox = true
			state.SleepingBoxCount++
		}
	}

	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	maxPasses := 2
	for pass := 0; pass < maxPasses; pass++ {
		hitCountThisPass := 0

		for _, pair := range pairs.SphereSphere {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			contact := physics.ResolveRigidSphereSphereContactWithFriction(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[first])
				wakeRigidSphere(&state.RigidSpheres[second])
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.BoxBox {
			first := pair[0]
			second := pair[1]
			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
			if contact.Hit {
				wakeRigidBox(&state.RigidBoxes[first])
				wakeRigidBox(&state.RigidBoxes[second])
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.SphereBox {
			sphereIndex := pair[0]
			boxIndex := pair[1]
			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[sphereIndex])
				wakeRigidBox(&state.RigidBoxes[boxIndex])
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
				hitCountThisPass++
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(&state.RigidSpheres[index])
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(&state.RigidBoxes[index])
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}

	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	for index := range state.RigidSpheres {
		if state.RigidSpheres[index].Sleeping {
			state.SleepingSphereCount++
		}
	}
	for index := range state.RigidBoxes {
		if state.RigidBoxes[index].Sleeping {
			state.SleepingBoxCount++
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDOptimizedScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	state.BroadphaseCellCount = 0
	state.SphereSphereCandidateCount = 0
	state.BoxBoxCandidateCount = 0
	state.SphereBoxCandidateCount = 0
	state.SphereSphereHitCount = 0
	state.BoxBoxHitCount = 0
	state.SphereBoxHitCount = 0
	state.BroadphaseDebugCells = nil
	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	state.ActiveCCDSphereCount = 0
	state.ActiveCCDBoxCount = 0
	state.ActiveAngularRiskCCDBoxCount = 0
	state.ActiveDiscreteSphereCount = 0
	state.ActiveDiscreteBoxCount = 0
	state.SleepingSphereSphereSkipCount = 0
	state.SleepingBoxBoxSkipCount = 0
	state.SleepingSphereBoxSkipCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD := shouldUseRigidSphereCCD(*body)
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepSphereTriangleMesh(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				physics.DefaultTimeStep,
				state.GroundTriangles,
			)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
				if normalVelocity.Cmp(fixed.Zero) < 0 {
					body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
				}
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
			} else {
				physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteSphereCount++
			physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
			physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(body)
		if body.Sleeping {
			state.EverSleptSphere = true
			state.SleepingSphereCount++
		}
	}

	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}

		useCCD, dueToAngularRisk := shouldUseRigidBoxCCDWithAngularRisk(*body)
		if useCCD {
			state.ActiveCCDBoxCount++
			state.EverActivatedCCDBox = true
			if dueToAngularRisk {
				state.ActiveAngularRiskCCDBoxCount++
				state.EverActivatedAngularRiskCCDBox = true
			}
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepRotatingOrientedBoxTriangleMesh(*body, physics.DefaultTimeStep, state.GroundTriangles)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				body.Orientation = contact.Orientation
				result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
				if index < len(state.LastContacts) {
					state.LastContacts[index] = result.LastContact
				}
			} else {
				physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteBoxCount++
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(body)
		if body.Sleeping {
			state.EverSleptBox = true
			state.SleepingBoxCount++
		}
	}

	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	maxPasses := 2
	for pass := 0; pass < maxPasses; pass++ {
		hitCountThisPass := 0

		for _, pair := range pairs.SphereSphere {
			first := pair[0]
			second := pair[1]
			if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
				state.SleepingSphereSphereSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			contact := physics.ResolveRigidSphereSphereContactWithFriction(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[first])
				wakeRigidSphere(&state.RigidSpheres[second])
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.BoxBox {
			first := pair[0]
			second := pair[1]
			if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
				state.SleepingBoxBoxSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
			if contact.Hit {
				wakeRigidBox(&state.RigidBoxes[first])
				wakeRigidBox(&state.RigidBoxes[second])
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.SphereBox {
			sphereIndex := pair[0]
			boxIndex := pair[1]
			if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
				state.SleepingSphereBoxSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[sphereIndex])
				wakeRigidBox(&state.RigidBoxes[boxIndex])
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
				hitCountThisPass++
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(&state.RigidSpheres[index])
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(&state.RigidBoxes[index])
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}

	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	for index := range state.RigidSpheres {
		if state.RigidSpheres[index].Sleeping {
			state.SleepingSphereCount++
		}
	}
	for index := range state.RigidBoxes {
		if state.RigidBoxes[index].Sleeping {
			state.SleepingBoxCount++
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckOptimizedScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = make([]physics.SphereTriangleContact, len(state.RigidSpheres))
	state.BroadphaseCellCount = 0
	state.SphereSphereCandidateCount = 0
	state.BoxBoxCandidateCount = 0
	state.SphereBoxCandidateCount = 0
	state.SphereSphereHitCount = 0
	state.BoxBoxHitCount = 0
	state.SphereBoxHitCount = 0
	state.BroadphaseDebugCells = nil
	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	state.ActiveCCDSphereCount = 0
	state.ActiveCCDBoxCount = 0
	state.ActiveAngularRiskCCDBoxCount = 0
	state.ActiveDiscreteSphereCount = 0
	state.ActiveDiscreteBoxCount = 0
	state.SleepingSphereSphereSkipCount = 0
	state.SleepingBoxBoxSkipCount = 0
	state.SleepingSphereBoxSkipCount = 0
	state.BoxCCDPrecheckCount = 0
	state.BoxCCDPrecheckRejectCount = 0
	state.BoxCCDMeshSweepCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD := shouldUseRigidSphereCCD(*body)
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			contact := physics.SweepSphereTriangleMesh(
				body.Motion.Position,
				body.Motion.Velocity,
				body.Radius,
				physics.DefaultTimeStep,
				state.GroundTriangles,
			)
			if contact.Hit {
				state.EverTouchedGround = true
				body.Motion.Position = contact.Position
				normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
				if normalVelocity.Cmp(fixed.Zero) < 0 {
					body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
				}
				remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
				if remainingFraction.Cmp(fixed.Zero) > 0 {
					physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
				}
			} else {
				physics.AdvanceRigidSphereBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteSphereCount++
			physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
			physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(body)
		if body.Sleeping {
			state.EverSleptSphere = true
			state.SleepingSphereCount++
		}
	}

	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}

		useCCD, dueToAngularRisk := shouldUseRigidBoxCCDWithAngularRisk(*body)
		if useCCD {
			state.ActiveCCDBoxCount++
			state.EverActivatedCCDBox = true
			if dueToAngularRisk {
				state.ActiveAngularRiskCCDBoxCount++
				state.EverActivatedAngularRiskCCDBox = true
			}
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			state.BoxCCDPrecheckCount++
			if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
				state.BoxCCDMeshSweepCount++
				state.EverExecutedBoxCCDMeshSweep = true
				contact := physics.SweepRotatingOrientedBoxTriangleMesh(*body, physics.DefaultTimeStep, state.GroundTriangles)
				if contact.Hit {
					state.EverTouchedGround = true
					body.Motion.Position = contact.Position
					body.Orientation = contact.Orientation
					result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
					remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
					if remainingFraction.Cmp(fixed.Zero) > 0 {
						physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
					}
					if index < len(state.LastContacts) {
						state.LastContacts[index] = result.LastContact
					}
				} else {
					physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep)
				}
			} else {
				state.BoxCCDPrecheckRejectCount++
				state.EverRejectedBoxCCDPrecheck = true
				physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep)
			}
		} else {
			state.ActiveDiscreteBoxCount++
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(body)
		if body.Sleeping {
			state.EverSleptBox = true
			state.SleepingBoxCount++
		}
	}

	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	maxPasses := 2
	for pass := 0; pass < maxPasses; pass++ {
		hitCountThisPass := 0

		for _, pair := range pairs.SphereSphere {
			first := pair[0]
			second := pair[1]
			if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
				state.SleepingSphereSphereSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			contact := physics.ResolveRigidSphereSphereContactWithFriction(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[first])
				wakeRigidSphere(&state.RigidSpheres[second])
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.BoxBox {
			first := pair[0]
			second := pair[1]
			if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
				state.SleepingBoxBoxSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidBoxBoxContact(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution)
			if contact.Hit {
				wakeRigidBox(&state.RigidBoxes[first])
				wakeRigidBox(&state.RigidBoxes[second])
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
				hitCountThisPass++
			}
		}

		for _, pair := range pairs.SphereBox {
			sphereIndex := pair[0]
			boxIndex := pair[1]
			if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
				state.SleepingSphereBoxSkipCount++
				state.EverSkippedSleepingPairs = true
				continue
			}

			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			contact := physics.ResolveRigidSphereRigidBoxContactWithFriction(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[sphereIndex])
				wakeRigidBox(&state.RigidBoxes[boxIndex])
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
				hitCountThisPass++
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidSphereSleepState(&state.RigidSpheres[index])
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		updateRigidBoxSleepState(&state.RigidBoxes[index])
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}

	state.SleepingSphereCount = 0
	state.SleepingBoxCount = 0
	for index := range state.RigidSpheres {
		if state.RigidSpheres[index].Sleeping {
			state.SleepingSphereCount++
		}
	}
	for index := range state.RigidBoxes {
		if state.RigidBoxes[index].Sleeping {
			state.SleepingBoxCount++
		}
	}

	if len(state.RigidSpheres) > 0 {
		state.RigidSphere = state.RigidSpheres[0]
	}
	if len(state.RigidBoxes) > 0 {
		state.RigidBox = state.RigidBoxes[0]
	}
}

func StepRigidSphereHighSpeedThinWallProjectileScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidSphereThinWallProjectile(state, &state.RigidSphere)
}

func StepRigidSphereHighSpeedThinWallProjectileCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidSphereThinWallProjectile(state, &state.RigidSphere)
}

func StepRigidSphereHighSpeedThinWallProjectileMeshCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidSphereThinWallProjectile(state, &state.RigidSphere)
}

func StepRigidBoxHighSpeedThinWallProjectileCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidBoxThinWallProjectile(state, &state.RigidBox)
}

func StepRigidBoxRotatingHighSpeedThinWallProjectileCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidBoxThinWallProjectile(state, &state.RigidBox)
}

func StepRigidBoxRotatingHighSpeedThinWallProjectileOBBCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidBoxThinWallProjectile(state, &state.RigidBox)
}

func StepRigidBoxRotatingHighSpeedThinWallProjectileOBBMeshCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	stepRigidBoxThinWallProjectile(state, &state.RigidBox)
}

func stepRigidSphereThinWallProjectile(state *SceneState, body *physics.RigidSphereBody3D) {
	if state == nil || body == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.CCDContactDetected = false
	state.CCDTimeOfImpact = fixed.Zero

	contact, usedCCD, hit, timeOfImpact := stepRigidSphereThinWallProjectileByMode(body, state.GroundBoxes, state.GroundTriangles)
	if !hit {
		return
	}

	state.LastContact = contact
	state.LastContacts = []physics.SphereTriangleContact{contact}
	state.EverTouchedGround = true
	if usedCCD {
		state.RigidSphereCCDHitDetected = true
	}
	if usedCCD {
		state.CCDContactDetected = true
		state.CCDTimeOfImpact = timeOfImpact
	}
}

func stepRigidSphereThinWallProjectileByMode(body *physics.RigidSphereBody3D, groundBoxes []geometry.AxisAlignedBoundingBox, groundTriangles []geometry.Triangle) (physics.SphereTriangleContact, bool, bool, fixed.Fixed) {
	if body == nil {
		return physics.SphereTriangleContact{}, false, false, fixed.Zero
	}

	if !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		result := physics.StepRigidSphereBody3DWithGravity(
			body,
			physics.DefaultTimeStep,
			geometry.ZeroVector3(),
			groundTriangles,
		)
		return result.LastContact, false, result.HadContact, fixed.Zero
	}

	switch body.CCDMode {
	case physics.CCDModeSweepAxisAlignedBoundingBox:
		if len(groundBoxes) == 0 {
			body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep))
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}
		contact := physics.SweepSphereAxisAlignedBoundingBox(
			body.Motion.Position,
			body.Motion.Velocity,
			body.Radius,
			physics.DefaultTimeStep,
			groundBoxes[0],
		)
		if !contact.Hit {
			body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep))
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		body.Motion.Position = contact.Position
		normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
		if normalVelocity.Cmp(fixed.Zero) < 0 {
			body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
		}
		remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
		body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep.Mul(remainingFraction)))
		return physics.SphereTriangleContact{
			Hit:         true,
			Point:       contact.ContactPoint,
			Normal:      contact.Normal,
			Penetration: fixed.Zero,
		}, true, true, contact.TimeOfImpact
	case physics.CCDModeSweepTriangleMesh:
		contact := physics.SweepSphereTriangleMesh(
			body.Motion.Position,
			body.Motion.Velocity,
			body.Radius,
			physics.DefaultTimeStep,
			groundTriangles,
		)
		if !contact.Hit {
			body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep))
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		body.Motion.Position = contact.Position
		normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
		if normalVelocity.Cmp(fixed.Zero) < 0 {
			body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
		}
		remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
		body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep.Mul(remainingFraction)))
		return physics.SphereTriangleContact{
			Hit:         true,
			Point:       contact.ContactPoint,
			Normal:      contact.Normal,
			Penetration: fixed.Zero,
		}, true, true, contact.TimeOfImpact
	default:
		result := physics.StepRigidSphereBody3DWithGravity(
			body,
			physics.DefaultTimeStep,
			geometry.ZeroVector3(),
			groundTriangles,
		)
		return result.LastContact, false, result.HadContact, fixed.Zero
	}
}

func stepRigidBoxThinWallProjectile(state *SceneState, body *physics.RigidBoxBody3D) {
	if state == nil || body == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.CCDContactDetected = false
	state.CCDTimeOfImpact = fixed.Zero

	contact, usedCCD, hit, timeOfImpact := stepRigidBoxThinWallProjectileByMode(body, state.GroundBoxes, state.GroundTriangles)
	if !hit {
		return
	}

	state.LastContact = contact
	state.LastContacts = []physics.SphereTriangleContact{contact}
	state.EverTouchedGround = true
	if usedCCD {
		state.RigidBoxCCDHitDetected = true
	}
	if usedCCD {
		state.CCDContactDetected = true
		state.CCDTimeOfImpact = timeOfImpact
	}
}

func stepRigidBoxThinWallProjectileByMode(body *physics.RigidBoxBody3D, groundBoxes []geometry.AxisAlignedBoundingBox, groundTriangles []geometry.Triangle) (physics.SphereTriangleContact, bool, bool, fixed.Fixed) {
	if body == nil {
		return physics.SphereTriangleContact{}, false, false, fixed.Zero
	}

	if !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
		return physics.SphereTriangleContact{}, false, false, fixed.Zero
	}

	switch body.CCDMode {
	case physics.CCDModeSweepAxisAlignedBoundingBox:
		if len(groundBoxes) == 0 {
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		startPosition := body.Motion.Position
		startVelocity := body.Motion.Velocity
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())

		contact := physics.SweepAxisAlignedBoxAxisAlignedBoundingBox(
			startPosition,
			startVelocity,
			body.HalfExtents,
			physics.DefaultTimeStep,
			groundBoxes[0],
		)
		if !contact.Hit {
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		body.Motion.Position = contact.Position
		body.Motion.Velocity = startVelocity
		result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
		return result.LastContact, true, true, contact.TimeOfImpact
	case physics.CCDModeSweepRotatingOrientedBoxAxisAlignedBoundingBox:
		if len(groundBoxes) == 0 {
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}
		contact := physics.SweepRotatingOrientedBoxAxisAlignedBoundingBox(*body, physics.DefaultTimeStep, groundBoxes[0])
		if !contact.Hit {
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		body.Motion.Position = contact.Position
		body.Orientation = contact.Orientation
		result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
		return result.LastContact, true, true, contact.TimeOfImpact
	case physics.CCDModeSweepRotatingOrientedBoxTriangleMesh:
		contact := physics.SweepRotatingOrientedBoxTriangleMesh(*body, physics.DefaultTimeStep, groundTriangles)
		if !contact.Hit {
			physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
			return physics.SphereTriangleContact{}, true, false, fixed.Zero
		}

		body.Motion.Position = contact.Position
		body.Orientation = contact.Orientation
		result := physics.ResolveRigidBoxBody3DContact(body, contact.ContactPoint, contact.Normal, fixed.Zero, body.Restitution)
		remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
		if remainingFraction.Cmp(fixed.Zero) > 0 {
			physics.AdvanceRigidBoxBody3DWithoutForce(body, physics.DefaultTimeStep.Mul(remainingFraction))
		}
		return result.LastContact, true, true, contact.TimeOfImpact
	default:
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, geometry.ZeroVector3())
		return physics.SphereTriangleContact{}, false, false, fixed.Zero
	}
}

func StepRigidSphereAndBoxThinWallProjectileObjectCCDScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	state.LastContacts = nil
	state.CCDContactDetected = false
	state.CCDTimeOfImpact = fixed.Zero

	sphereContact, sphereUsedCCD, sphereHit, sphereTOI := stepRigidSphereThinWallProjectileByMode(&state.RigidSphere, state.GroundBoxes, state.GroundTriangles)
	boxContact, boxUsedCCD, boxHit, boxTOI := stepRigidBoxThinWallProjectileByMode(&state.RigidBox, state.GroundBoxes, state.GroundTriangles)

	if sphereHit {
		state.LastContacts = append(state.LastContacts, sphereContact)
		if sphereUsedCCD {
			state.RigidSphereCCDHitDetected = true
		}
		state.EverTouchedGround = true
	}
	if boxHit {
		state.LastContacts = append(state.LastContacts, boxContact)
		if boxUsedCCD {
			state.RigidBoxCCDHitDetected = true
		}
		state.EverTouchedGround = true
	}
	if len(state.LastContacts) > 0 {
		state.LastContact = state.LastContacts[0]
	}

	if sphereUsedCCD && sphereHit {
		state.CCDContactDetected = true
		state.CCDTimeOfImpact = sphereTOI
	}
	if boxUsedCCD && boxHit && (!state.CCDContactDetected || boxTOI.Cmp(state.CCDTimeOfImpact) < 0) {
		state.CCDContactDetected = true
		state.CCDTimeOfImpact = boxTOI
		if boxHit {
			state.LastContact = boxContact
		}
	}
}
