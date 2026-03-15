package scenario

import (
	"sort"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

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

func StepRigidSphereHighSpeedThinWallProjectileScene(state *SceneState) {
	if state == nil {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	result := physics.StepRigidSphereBody3DWithGravity(
		&state.RigidSphere,
		physics.DefaultTimeStep,
		geometry.ZeroVector3(),
		state.GroundTriangles,
	)
	if result.HadContact {
		state.LastContact = result.LastContact
		state.EverTouchedGround = true
	}
}

func StepRigidSphereHighSpeedThinWallProjectileCCDScene(state *SceneState) {
	if state == nil || len(state.GroundBoxes) == 0 {
		return
	}

	state.LastContact = physics.SphereTriangleContact{}
	body := &state.RigidSphere
	contact := physics.SweepSphereAxisAlignedBoundingBox(
		body.Motion.Position,
		body.Motion.Velocity,
		body.Radius,
		physics.DefaultTimeStep,
		state.GroundBoxes[0],
	)
	if !contact.Hit {
		body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep))
		return
	}

	state.EverTouchedGround = true
	body.Motion.Position = contact.Position
	normalVelocity := body.Motion.Velocity.Dot(contact.Normal)
	if normalVelocity.Cmp(fixed.Zero) < 0 {
		body.Motion.Velocity = body.Motion.Velocity.Sub(contact.Normal.Scale(normalVelocity.Mul(fixed.One.Add(body.Restitution))))
	}

	remainingFraction := fixed.One.Sub(contact.TimeOfImpact)
	body.Motion.Position = body.Motion.Position.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep.Mul(remainingFraction)))
	state.LastContact = physics.SphereTriangleContact{
		Hit:         true,
		Point:       contact.ContactPoint,
		Normal:      contact.Normal,
		Penetration: fixed.Zero,
	}
}
