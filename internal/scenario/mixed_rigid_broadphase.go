package scenario

import (
	"sort"
	"time"

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

type scoredSphereSpherePair struct {
	Pair  [2]int
	Score fixed.Fixed
}

type scoredBoxBoxPair struct {
	Pair  [2]int
	Score fixed.Fixed
}

type scoredSphereBoxPair struct {
	Pair  [2]int
	Score fixed.Fixed
}

const (
	persistentPairKindSphereSphere uint64 = 1
	persistentPairKindBoxBox       uint64 = 2
	persistentPairKindSphereBox    uint64 = 3
	persistentPairKindShift               = 56
)

type islandUnionFind struct {
	parent []int
	size   []int
}

func newIslandUnionFind(count int) *islandUnionFind {
	parent := make([]int, count)
	size := make([]int, count)
	for i := range parent {
		parent[i] = i
		size[i] = 1
	}
	return &islandUnionFind{parent: parent, size: size}
}

func (u *islandUnionFind) find(index int) int {
	if u.parent[index] != index {
		u.parent[index] = u.find(u.parent[index])
	}
	return u.parent[index]
}

func (u *islandUnionFind) union(first, second int) {
	rootA := u.find(first)
	rootB := u.find(second)
	if rootA == rootB {
		return
	}
	if u.size[rootA] < u.size[rootB] {
		rootA, rootB = rootB, rootA
	}
	u.parent[rootB] = rootA
	u.size[rootA] += u.size[rootB]
}

func encodePersistentPairKey(kind uint64, first, second int) uint64 {
	if first > second && kind != persistentPairKindSphereBox {
		first, second = second, first
	}
	return (kind << persistentPairKindShift) | (uint64(uint32(first)) << 28) | uint64(uint32(second))
}

func makeSphereSpherePersistentKey(first, second int) uint64 {
	return encodePersistentPairKey(persistentPairKindSphereSphere, first, second)
}

func makeBoxBoxPersistentKey(first, second int) uint64 {
	return encodePersistentPairKey(persistentPairKindBoxBox, first, second)
}

func makeSphereBoxPersistentKey(sphereIndex, boxIndex int) uint64 {
	return encodePersistentPairKey(persistentPairKindSphereBox, sphereIndex, boxIndex)
}

func sortPersistentContactCache(entries []PersistentContactCacheEntry) {
	sort.Slice(entries, func(i, j int) bool {
		return entries[i].Key < entries[j].Key
	})
}

func persistentContactCacheMap(entries []PersistentContactCacheEntry) map[uint64]PersistentContactCacheEntry {
	cache := make(map[uint64]PersistentContactCacheEntry, len(entries))
	for _, entry := range entries {
		cache[entry.Key] = entry
	}
	return cache
}

func persistentEntryToWarmStart(entry PersistentContactCacheEntry) physics.ContactWarmStartManifold {
	return physics.ContactWarmStartManifold{
		Point:          entry.Point,
		Normal:         entry.Normal,
		NormalImpulse:  entry.NormalImpulse,
		TangentImpulse: entry.TangentImpulse,
	}
}

func hasUsefulWarmStart(entry PersistentContactCacheEntry) bool {
	return entry.NormalImpulse.Cmp(fixed.Zero) > 0 || entry.TangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0
}

func ensurePositiveDurationNanos(value int64) int64 {
	if value <= 0 {
		return 1
	}
	return value
}

func contactScoreFromManifold(manifold physics.ContactWarmStartManifold) fixed.Fixed {
	score := manifold.NormalImpulse
	if manifold.TangentImpulse.LengthSquared().Cmp(fixed.Zero) > 0 {
		score = score.Add(manifold.TangentImpulse.Length())
	}
	if score == fixed.Zero {
		score = fixed.FromFraction(1, 1000)
	}
	return score
}

func trimScoredSphereSpherePairs(pairs []scoredSphereSpherePair, budget int) [][2]int {
	if budget <= 0 || len(pairs) <= budget {
		out := make([][2]int, 0, len(pairs))
		for _, pair := range pairs {
			out = append(out, pair.Pair)
		}
		return out
	}
	sort.Slice(pairs, func(i, j int) bool {
		return pairs[i].Score.Cmp(pairs[j].Score) > 0
	})
	out := make([][2]int, 0, budget)
	for _, pair := range pairs[:budget] {
		out = append(out, pair.Pair)
	}
	return out
}

func trimScoredBoxBoxPairs(pairs []scoredBoxBoxPair, budget int) [][2]int {
	if budget <= 0 || len(pairs) <= budget {
		out := make([][2]int, 0, len(pairs))
		for _, pair := range pairs {
			out = append(out, pair.Pair)
		}
		return out
	}
	sort.Slice(pairs, func(i, j int) bool {
		return pairs[i].Score.Cmp(pairs[j].Score) > 0
	})
	out := make([][2]int, 0, budget)
	for _, pair := range pairs[:budget] {
		out = append(out, pair.Pair)
	}
	return out
}

func trimScoredSphereBoxPairs(pairs []scoredSphereBoxPair, budget int) [][2]int {
	if budget <= 0 || len(pairs) <= budget {
		out := make([][2]int, 0, len(pairs))
		for _, pair := range pairs {
			out = append(out, pair.Pair)
		}
		return out
	}
	sort.Slice(pairs, func(i, j int) bool {
		return pairs[i].Score.Cmp(pairs[j].Score) > 0
	})
	out := make([][2]int, 0, budget)
	for _, pair := range pairs[:budget] {
		out = append(out, pair.Pair)
	}
	return out
}

func reorderPersistentPairs(pairs [][2]int, keyFunc func([2]int) uint64, persistent map[uint64]PersistentContactCacheEntry) ([][2]int, int) {
	if len(pairs) == 0 || len(persistent) == 0 {
		return pairs, 0
	}

	persistentPairs := make([][2]int, 0, len(pairs))
	freshPairs := make([][2]int, 0, len(pairs))
	reused := 0
	for _, pair := range pairs {
		if _, exists := persistent[keyFunc(pair)]; exists {
			persistentPairs = append(persistentPairs, pair)
			reused++
		} else {
			freshPairs = append(freshPairs, pair)
		}
	}

	ordered := make([][2]int, 0, len(pairs))
	ordered = append(ordered, persistentPairs...)
	ordered = append(ordered, freshPairs...)
	return ordered, reused
}

func islandSleepEligibleSphere(body physics.RigidSphereBody3D) bool {
	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	return linear.Cmp(sleepExitThreshold(body.SleepLinearThreshold)) <= 0 &&
		angular.Cmp(sleepExitThreshold(body.SleepAngularThreshold)) <= 0
}

func islandSleepEligibleBox(body physics.RigidBoxBody3D) bool {
	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	return linear.Cmp(sleepExitThreshold(body.SleepLinearThreshold)) <= 0 &&
		angular.Cmp(sleepExitThreshold(body.SleepAngularThreshold)) <= 0
}

func nodeIndexForBox(sphereCount, boxIndex int) int {
	return sphereCount + boxIndex
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

func cheapPrecheckRigidSphereContainerCCD(body physics.RigidSphereBody3D, minX, maxX, minZ, maxZ fixed.Fixed) bool {
	start := body.Motion.Position
	end := start.Add(body.Motion.Velocity.Scale(physics.DefaultTimeStep))
	radius := body.Radius

	if start.Y.Sub(radius).Cmp(fixed.Zero) <= 0 || end.Y.Sub(radius).Cmp(fixed.Zero) <= 0 {
		return true
	}
	if start.X.Sub(radius).Cmp(minX) <= 0 || end.X.Sub(radius).Cmp(minX) <= 0 {
		return true
	}
	if start.X.Add(radius).Cmp(maxX) >= 0 || end.X.Add(radius).Cmp(maxX) >= 0 {
		return true
	}
	if start.Z.Sub(radius).Cmp(minZ) <= 0 || end.Z.Sub(radius).Cmp(minZ) <= 0 {
		return true
	}
	if start.Z.Add(radius).Cmp(maxZ) >= 0 || end.Z.Add(radius).Cmp(maxZ) >= 0 {
		return true
	}

	return false
}

func ccdExitThreshold(threshold fixed.Fixed) fixed.Fixed {
	if threshold == fixed.Zero {
		return fixed.Zero
	}
	return threshold.Mul(fixed.FromFraction(3, 4))
}

func sleepExitThreshold(threshold fixed.Fixed) fixed.Fixed {
	if threshold == fixed.Zero {
		return fixed.Zero
	}
	return threshold.Mul(fixed.FromFraction(3, 2))
}

func shouldUseRigidSphereCCDHysteresis(body *physics.RigidSphereBody3D) (bool, bool) {
	if body == nil || !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		if body != nil {
			body.CCDActive = false
		}
		return false, false
	}

	enterThreshold := body.CCDVelocityThreshold
	if enterThreshold == fixed.Zero {
		enterThreshold = body.Radius
	}
	exitThreshold := ccdExitThreshold(enterThreshold)
	speed := body.Motion.Velocity.Length()

	if body.CCDActive {
		if speed.Cmp(exitThreshold) >= 0 {
			heldByHysteresis := speed.Cmp(enterThreshold) < 0
			body.CCDActive = true
			return true, heldByHysteresis
		}
		body.CCDActive = false
		return false, false
	}

	if speed.Cmp(enterThreshold) >= 0 {
		body.CCDActive = true
		return true, false
	}

	return false, false
}

func shouldUseRigidBoxCCDHysteresis(body *physics.RigidBoxBody3D) (bool, bool, bool) {
	if body == nil || !body.UseCCD || body.CCDMode == physics.CCDModeDiscrete {
		if body != nil {
			body.CCDActive = false
		}
		return false, false, false
	}

	enterLinearThreshold := body.CCDVelocityThreshold
	if enterLinearThreshold == fixed.Zero {
		enterLinearThreshold = body.HalfExtents.Length()
	}
	exitLinearThreshold := ccdExitThreshold(enterLinearThreshold)
	enterAngularThreshold := body.CCDAngularSweepThreshold
	if enterAngularThreshold == fixed.Zero {
		enterAngularThreshold = body.HalfExtents.Length()
	}
	exitAngularThreshold := ccdExitThreshold(enterAngularThreshold)

	speed := body.Motion.Velocity.Length()
	angularRisk := body.AngularVelocity.Length().Mul(body.HalfExtents.Length())
	enterLinear := speed.Cmp(exitLinearThreshold) >= 0 && speed.Cmp(enterLinearThreshold) >= 0
	enterAngular := angularRisk.Cmp(exitAngularThreshold) >= 0 && angularRisk.Cmp(enterAngularThreshold) >= 0

	if body.CCDActive {
		keepLinear := speed.Cmp(exitLinearThreshold) >= 0
		keepAngular := angularRisk.Cmp(exitAngularThreshold) >= 0
		if keepLinear || keepAngular {
			heldByHysteresis := (!enterLinear && keepLinear) || (!enterAngular && keepAngular)
			body.CCDActive = true
			return true, !keepLinear && keepAngular, heldByHysteresis
		}
		body.CCDActive = false
		return false, false, false
	}

	if enterLinear || enterAngular {
		body.CCDActive = true
		return true, !enterLinear && enterAngular, false
	}

	return false, false, false
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

func updateRigidSphereSleepStateWithHysteresis(body *physics.RigidSphereBody3D) bool {
	if body == nil {
		return false
	}

	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	enterLinear := body.SleepLinearThreshold
	enterAngular := body.SleepAngularThreshold
	exitLinear := sleepExitThreshold(enterLinear)
	exitAngular := sleepExitThreshold(enterAngular)

	if body.Sleeping {
		if linear.Cmp(exitLinear) > 0 || angular.Cmp(exitAngular) > 0 {
			body.Sleeping = false
			body.SleepTickCount = 0
			return false
		}
		body.Motion.Velocity = geometry.ZeroVector3()
		body.AngularVelocity = geometry.ZeroVector3()
		return false
	}

	if linear.Cmp(enterLinear) <= 0 && angular.Cmp(enterAngular) <= 0 {
		body.SleepTickCount++
		if body.SleepTickCount >= body.SleepTickThreshold {
			body.Sleeping = true
			body.Motion.Velocity = geometry.ZeroVector3()
			body.AngularVelocity = geometry.ZeroVector3()
		}
		return false
	}

	if linear.Cmp(exitLinear) <= 0 && angular.Cmp(exitAngular) <= 0 && body.SleepTickCount > 0 {
		return true
	}

	body.Sleeping = false
	body.SleepTickCount = 0
	return false
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

func updateRigidBoxSleepStateWithHysteresis(body *physics.RigidBoxBody3D) bool {
	if body == nil {
		return false
	}

	linear := body.Motion.Velocity.Length()
	angular := body.AngularVelocity.Length()
	enterLinear := body.SleepLinearThreshold
	enterAngular := body.SleepAngularThreshold
	exitLinear := sleepExitThreshold(enterLinear)
	exitAngular := sleepExitThreshold(enterAngular)

	if body.Sleeping {
		if linear.Cmp(exitLinear) > 0 || angular.Cmp(exitAngular) > 0 {
			body.Sleeping = false
			body.SleepTickCount = 0
			return false
		}
		body.Motion.Velocity = geometry.ZeroVector3()
		body.AngularVelocity = geometry.ZeroVector3()
		return false
	}

	if linear.Cmp(enterLinear) <= 0 && angular.Cmp(enterAngular) <= 0 {
		body.SleepTickCount++
		if body.SleepTickCount >= body.SleepTickThreshold {
			body.Sleeping = true
			body.Motion.Velocity = geometry.ZeroVector3()
			body.AngularVelocity = geometry.ZeroVector3()
		}
		return false
	}

	if linear.Cmp(exitLinear) <= 0 && angular.Cmp(exitAngular) <= 0 && body.SleepTickCount > 0 {
		return true
	}

	body.Sleeping = false
	body.SleepTickCount = 0
	return false
}

func wakeRigidSphere(body *physics.RigidSphereBody3D) {
	if body == nil {
		return
	}
	body.Sleeping = false
	body.SleepTickCount = 0
	body.CCDActive = false
}

func wakeRigidBox(body *physics.RigidBoxBody3D) {
	if body == nil {
		return
	}
	body.Sleeping = false
	body.SleepTickCount = 0
	body.CCDActive = false
}

func appendPersistentContact(cache map[uint64]PersistentContactCacheEntry, key uint64, point, normal geometry.Vector3) {
	appendPersistentContactManifold(cache, key, point, normal, fixed.Zero, geometry.ZeroVector3())
}

func appendPersistentContactManifold(cache map[uint64]PersistentContactCacheEntry, key uint64, point, normal geometry.Vector3, normalImpulse fixed.Fixed, tangentImpulse geometry.Vector3) {
	cache[key] = PersistentContactCacheEntry{
		Key:            key,
		Point:          point,
		Normal:         normal,
		NormalImpulse:  normalImpulse,
		TangentImpulse: tangentImpulse,
	}
}

func applySleepingIslands(state *SceneState, hitPairs []PersistentContactCacheEntry) {
	if state == nil {
		return
	}

	totalBodies := len(state.RigidSpheres) + len(state.RigidBoxes)
	if totalBodies == 0 || len(hitPairs) == 0 {
		return
	}

	uf := newIslandUnionFind(totalBodies)
	for _, hit := range hitPairs {
		switch hit.Key >> persistentPairKindShift {
		case persistentPairKindSphereSphere:
			first := int((hit.Key >> 28) & ((1 << 28) - 1))
			second := int(hit.Key & ((1 << 28) - 1))
			if first < len(state.RigidSpheres) && second < len(state.RigidSpheres) {
				uf.union(first, second)
			}
		case persistentPairKindBoxBox:
			first := int((hit.Key >> 28) & ((1 << 28) - 1))
			second := int(hit.Key & ((1 << 28) - 1))
			if first < len(state.RigidBoxes) && second < len(state.RigidBoxes) {
				uf.union(nodeIndexForBox(len(state.RigidSpheres), first), nodeIndexForBox(len(state.RigidSpheres), second))
			}
		case persistentPairKindSphereBox:
			sphereIndex := int((hit.Key >> 28) & ((1 << 28) - 1))
			boxIndex := int(hit.Key & ((1 << 28) - 1))
			if sphereIndex < len(state.RigidSpheres) && boxIndex < len(state.RigidBoxes) {
				uf.union(sphereIndex, nodeIndexForBox(len(state.RigidSpheres), boxIndex))
			}
		}
	}

	groups := make(map[int][]int)
	for index := 0; index < totalBodies; index++ {
		root := uf.find(index)
		groups[root] = append(groups[root], index)
	}

	for _, group := range groups {
		if len(group) < 2 {
			continue
		}

		state.SleepingIslandCount++
		state.EverBuiltSleepingIsland = true
		if len(group) > state.LargestSleepingIslandSize {
			state.LargestSleepingIslandSize = len(group)
		}

		allEligible := true
		allSleeping := true
		for _, node := range group {
			if node < len(state.RigidSpheres) {
				body := state.RigidSpheres[node]
				if !body.Sleeping {
					allSleeping = false
				}
				if !body.Sleeping && !islandSleepEligibleSphere(body) {
					allEligible = false
					break
				}
			} else {
				boxIndex := node - len(state.RigidSpheres)
				body := state.RigidBoxes[boxIndex]
				if !body.Sleeping {
					allSleeping = false
				}
				if !body.Sleeping && !islandSleepEligibleBox(body) {
					allEligible = false
					break
				}
			}
		}

		if allSleeping {
			state.EverSleptIsland = true
			continue
		}

		if !allEligible {
			for _, node := range group {
				if node < len(state.RigidSpheres) && !state.RigidSpheres[node].Sleeping {
					state.RigidSpheres[node].SleepTickCount = 0
				} else if node >= len(state.RigidSpheres) {
					boxIndex := node - len(state.RigidSpheres)
					if !state.RigidBoxes[boxIndex].Sleeping {
						state.RigidBoxes[boxIndex].SleepTickCount = 0
					}
				}
			}
			continue
		}

		for _, node := range group {
			if node < len(state.RigidSpheres) {
				body := &state.RigidSpheres[node]
				if !body.Sleeping {
					body.SleepTickCount++
				}
			} else {
				boxIndex := node - len(state.RigidSpheres)
				body := &state.RigidBoxes[boxIndex]
				if !body.Sleeping {
					body.SleepTickCount++
				}
			}
		}

		state.EverSleptIsland = true
		for _, node := range group {
			if node < len(state.RigidSpheres) {
				body := &state.RigidSpheres[node]
				body.Sleeping = true
				body.Motion.Velocity = geometry.ZeroVector3()
				body.AngularVelocity = geometry.ZeroVector3()
			} else {
				boxIndex := node - len(state.RigidSpheres)
				body := &state.RigidBoxes[boxIndex]
				body.Sleeping = true
				body.Motion.Velocity = geometry.ZeroVector3()
				body.AngularVelocity = geometry.ZeroVector3()
			}
		}
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
			state.EverRanBoxCCDPrecheck = true
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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckHysteresisOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			state.SphereCCDPrecheckCount++
			state.EverRanSphereCCDPrecheck = true
			if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
				state.SphereCCDMeshSweepCount++
				state.EverExecutedSphereCCDMeshSweep = true
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
				state.SphereCCDPrecheckRejectCount++
				state.EverRejectedSphereCCDPrecheck = true
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
		if updateRigidSphereSleepStateWithHysteresis(body) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
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

		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
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
			state.EverRanBoxCCDPrecheck = true
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
		if updateRigidBoxSleepStateWithHysteresis(body) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
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
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxPersistentPairsIslandsWarmStartOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			state.SphereCCDPrecheckCount++
			state.EverRanSphereCCDPrecheck = true
			if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
				state.SphereCCDMeshSweepCount++
				state.EverExecutedSphereCCDMeshSweep = true
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
				state.SphereCCDPrecheckRejectCount++
				state.EverRejectedSphereCCDPrecheck = true
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
		if updateRigidSphereSleepStateWithHysteresis(body) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
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

		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
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
			state.EverRanBoxCCDPrecheck = true
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
		if updateRigidBoxSleepStateWithHysteresis(body) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if body.Sleeping {
			state.EverSleptBox = true
			state.SleepingBoxCount++
		}
	}

	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(
		pairs.SphereSphere,
		func(pair [2]int) uint64 { return makeSphereSpherePersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(
		pairs.BoxBox,
		func(pair [2]int) uint64 { return makeBoxBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(
		pairs.SphereBox,
		func(pair [2]int) uint64 { return makeSphereBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}

	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
	warmSpherePairs := state.PersistentPairReuseCount
	if warmSpherePairs > len(pairs.SphereSphere) {
		warmSpherePairs = len(pairs.SphereSphere)
	}
	warmBoxPairs := reused
	_ = warmBoxPairs

	sphereSpherePersistentCount := 0
	for sphereSpherePersistentCount < len(pairs.SphereSphere) {
		key := makeSphereSpherePersistentKey(pairs.SphereSphere[sphereSpherePersistentCount][0], pairs.SphereSphere[sphereSpherePersistentCount][1])
		if _, exists := prevPersistent[key]; !exists {
			break
		}
		sphereSpherePersistentCount++
	}
	boxBoxPersistentCount := 0
	for boxBoxPersistentCount < len(pairs.BoxBox) {
		key := makeBoxBoxPersistentKey(pairs.BoxBox[boxBoxPersistentCount][0], pairs.BoxBox[boxBoxPersistentCount][1])
		if _, exists := prevPersistent[key]; !exists {
			break
		}
		boxBoxPersistentCount++
	}
	sphereBoxPersistentCount := 0
	for sphereBoxPersistentCount < len(pairs.SphereBox) {
		key := makeSphereBoxPersistentKey(pairs.SphereBox[sphereBoxPersistentCount][0], pairs.SphereBox[sphereBoxPersistentCount][1])
		if _, exists := prevPersistent[key]; !exists {
			break
		}
		sphereBoxPersistentCount++
	}

	for index := 0; index < sphereSpherePersistentCount; index++ {
		pair := pairs.SphereSphere[index]
		first := pair[0]
		second := pair[1]
		if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
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
			state.WarmStartedPairCount++
			state.EverWarmStartedPairs = true
			appendPersistentContact(
				currentPersistentHits,
				makeSphereSpherePersistentKey(first, second),
				state.RigidSpheres[first].Motion.Position.Add(state.RigidSpheres[second].Motion.Position).Scale(fixed.FromFraction(1, 2)),
				contact.Normal,
			)
		}
	}
	for index := 0; index < boxBoxPersistentCount; index++ {
		pair := pairs.BoxBox[index]
		first := pair[0]
		second := pair[1]
		if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
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
			state.WarmStartedPairCount++
			state.EverWarmStartedPairs = true
			appendPersistentContact(currentPersistentHits, makeBoxBoxPersistentKey(first, second), contact.Point, contact.Normal)
		}
	}
	for index := 0; index < sphereBoxPersistentCount; index++ {
		pair := pairs.SphereBox[index]
		sphereIndex := pair[0]
		boxIndex := pair[1]
		if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
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
			state.WarmStartedPairCount++
			state.EverWarmStartedPairs = true
			appendPersistentContact(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), contact.Point, contact.Normal)
		}
	}

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
				appendPersistentContact(
					currentPersistentHits,
					makeSphereSpherePersistentKey(first, second),
					state.RigidSpheres[first].Motion.Position.Add(state.RigidSpheres[second].Motion.Position).Scale(fixed.FromFraction(1, 2)),
					contact.Normal,
				)
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
				appendPersistentContact(currentPersistentHits, makeBoxBoxPersistentKey(first, second), contact.Point, contact.Normal)
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
				appendPersistentContact(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), contact.Point, contact.Normal)
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}

	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxSolverManifoldWarmStartOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}

		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if useCCD {
			state.ActiveCCDSphereCount++
			state.EverActivatedCCDSphere = true
			body.Grounded = false
			body.Motion.Velocity = body.Motion.Velocity.Add(physics.StandardGravity.Scale(physics.DefaultTimeStep))
			state.SphereCCDPrecheckCount++
			state.EverRanSphereCCDPrecheck = true
			if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
				state.SphereCCDMeshSweepCount++
				state.EverExecutedSphereCCDMeshSweep = true
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
				state.SphereCCDPrecheckRejectCount++
				state.EverRejectedSphereCCDPrecheck = true
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
		if updateRigidSphereSleepStateWithHysteresis(body) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
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

		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
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
			state.EverRanBoxCCDPrecheck = true
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
		if updateRigidBoxSleepStateWithHysteresis(body) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if body.Sleeping {
			state.EverSleptBox = true
			state.SleepingBoxCount++
		}
	}

	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(
		pairs.SphereSphere,
		func(pair [2]int) uint64 { return makeSphereSpherePersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(
		pairs.BoxBox,
		func(pair [2]int) uint64 { return makeBoxBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(
		pairs.SphereBox,
		func(pair [2]int) uint64 { return makeSphereBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}

	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells

	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)

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
			key := makeSphereSpherePersistentKey(first, second)
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if entry, exists := prevPersistent[key]; exists {
				manifold = persistentEntryToWarmStart(entry)
				usedWarmStart = true
			}
			contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(
				&state.RigidSpheres[first],
				&state.RigidSpheres[second],
				combinedRestitution,
				combinedFriction,
				manifold,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[first])
				wakeRigidSphere(&state.RigidSpheres[second])
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
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
			key := makeBoxBoxPersistentKey(first, second)
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if entry, exists := prevPersistent[key]; exists {
				manifold = persistentEntryToWarmStart(entry)
				usedWarmStart = true
			}
			contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(
				&state.RigidBoxes[first],
				&state.RigidBoxes[second],
				combinedRestitution,
				manifold,
			)
			if contact.Hit {
				wakeRigidBox(&state.RigidBoxes[first])
				wakeRigidBox(&state.RigidBoxes[second])
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
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
			key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if entry, exists := prevPersistent[key]; exists {
				manifold = persistentEntryToWarmStart(entry)
				usedWarmStart = true
			}
			contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(
				&state.RigidSpheres[sphereIndex],
				&state.RigidBoxes[boxIndex],
				combinedRestitution,
				state.RigidSpheres[sphereIndex].Friction,
				manifold,
			)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[sphereIndex])
				wakeRigidBox(&state.RigidBoxes[boxIndex])
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}

	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)

	for index := range state.RigidSpheres {
		if physics.ConstrainRigidSphereBody3DToOpenContainer(&state.RigidSpheres[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if physics.ConstrainRigidBoxBody3DToOpenContainer(&state.RigidBoxes[index], minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverWarmStartOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0
	state.PhaseIntegrationNanos = 0
	state.PhaseCCDNanos = 0
	state.PhaseBroadphaseNanos = 0
	state.PhaseSolverNanos = 0
	state.PhaseSleepingNanos = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	integrationStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}
		physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
	}
	state.PhaseIntegrationNanos = ensurePositiveDurationNanos(time.Since(integrationStart).Nanoseconds())

	ccdStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			continue
		}
		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteSphereCount++
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDSphereCount++
		state.EverActivatedCCDSphere = true
		state.SphereCCDPrecheckCount++
		state.EverRanSphereCCDPrecheck = true
		if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
			state.SphereCCDMeshSweepCount++
			state.EverExecutedSphereCCDMeshSweep = true
		} else {
			state.SphereCCDPrecheckRejectCount++
			state.EverRejectedSphereCCDPrecheck = true
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			continue
		}
		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteBoxCount++
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDBoxCount++
		state.EverActivatedCCDBox = true
		if dueToAngularRisk {
			state.ActiveAngularRiskCCDBoxCount++
			state.EverActivatedAngularRiskCCDBox = true
		}
		state.BoxCCDPrecheckCount++
		state.EverRanBoxCCDPrecheck = true
		if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
			state.BoxCCDMeshSweepCount++
			state.EverExecutedBoxCCDMeshSweep = true
		} else {
			state.BoxCCDPrecheckRejectCount++
			state.EverRejectedBoxCCDPrecheck = true
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	state.PhaseCCDNanos = ensurePositiveDurationNanos(time.Since(ccdStart).Nanoseconds())

	broadphaseStart := time.Now()
	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	broadphaseCellSize := fixed.FromInt(2)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, broadphaseCellSize)
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(
		pairs.SphereSphere,
		func(pair [2]int) uint64 { return makeSphereSpherePersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(
		pairs.BoxBox,
		func(pair [2]int) uint64 { return makeBoxBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(
		pairs.SphereBox,
		func(pair [2]int) uint64 { return makeSphereBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells
	state.PhaseBroadphaseNanos = ensurePositiveDurationNanos(time.Since(broadphaseStart).Nanoseconds())

	solverStart := time.Now()
	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
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
			key := makeSphereSpherePersistentKey(first, second)
			combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
			if combinedFriction.Cmp(fixed.One) > 0 {
				combinedFriction = fixed.One
			}
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if pass == 0 {
				if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
					manifold = persistentEntryToWarmStart(entry)
					usedWarmStart = true
				}
			}
			contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, manifold)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[first])
				wakeRigidSphere(&state.RigidSpheres[second])
				state.RigidSphereSphereCollisionDetected = true
				state.SphereSphereHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
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
			key := makeBoxBoxPersistentKey(first, second)
			combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if pass == 0 {
				if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
					manifold = persistentEntryToWarmStart(entry)
					usedWarmStart = true
				}
			}
			contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, manifold)
			if contact.Hit {
				wakeRigidBox(&state.RigidBoxes[first])
				wakeRigidBox(&state.RigidBoxes[second])
				state.RigidBoxBoxCollisionDetected = true
				state.BoxBoxHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
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
			key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
			combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
			if combinedRestitution.Cmp(fixed.One) > 0 {
				combinedRestitution = fixed.One
			}
			manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
			if pass == 0 {
				if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
					manifold = persistentEntryToWarmStart(entry)
					usedWarmStart = true
				}
			}
			contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, manifold)
			if contact.Hit {
				wakeRigidSphere(&state.RigidSpheres[sphereIndex])
				wakeRigidBox(&state.RigidBoxes[boxIndex])
				state.SphereBoxCollisionDetected = true
				state.SphereBoxHitCount++
				hitCountThisPass++
				if usedWarmStart {
					state.WarmStartedPairCount++
					state.EverWarmStartedPairs = true
				}
				appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			}
		}

		if hitCountThisPass == 0 {
			break
		}
	}
	state.PhaseSolverNanos = ensurePositiveDurationNanos(time.Since(solverStart).Nanoseconds())

	sleepStart := time.Now()
	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)
	for index := range state.RigidSpheres {
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}
	state.PhaseSleepingNanos = ensurePositiveDurationNanos(time.Since(sleepStart).Nanoseconds())

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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxProfiledSolverFrontierOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0
	state.PhaseIntegrationNanos = 0
	state.PhaseCCDNanos = 0
	state.PhaseBroadphaseNanos = 0
	state.PhaseSolverNanos = 0
	state.PhaseSleepingNanos = 0
	state.SolverPass1PairCount = 0
	state.SolverPass2PairCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	integrationStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}
		physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
	}
	state.PhaseIntegrationNanos = ensurePositiveDurationNanos(time.Since(integrationStart).Nanoseconds())

	ccdStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			continue
		}
		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteSphereCount++
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDSphereCount++
		state.EverActivatedCCDSphere = true
		state.SphereCCDPrecheckCount++
		state.EverRanSphereCCDPrecheck = true
		if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
			state.SphereCCDMeshSweepCount++
			state.EverExecutedSphereCCDMeshSweep = true
		} else {
			state.SphereCCDPrecheckRejectCount++
			state.EverRejectedSphereCCDPrecheck = true
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			continue
		}
		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteBoxCount++
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDBoxCount++
		state.EverActivatedCCDBox = true
		if dueToAngularRisk {
			state.ActiveAngularRiskCCDBoxCount++
			state.EverActivatedAngularRiskCCDBox = true
		}
		state.BoxCCDPrecheckCount++
		state.EverRanBoxCCDPrecheck = true
		if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
			state.BoxCCDMeshSweepCount++
			state.EverExecutedBoxCCDMeshSweep = true
		} else {
			state.BoxCCDPrecheckRejectCount++
			state.EverRejectedBoxCCDPrecheck = true
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	state.PhaseCCDNanos = ensurePositiveDurationNanos(time.Since(ccdStart).Nanoseconds())

	broadphaseStart := time.Now()
	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, fixed.FromInt(2))
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(
		pairs.SphereSphere,
		func(pair [2]int) uint64 { return makeSphereSpherePersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(
		pairs.BoxBox,
		func(pair [2]int) uint64 { return makeBoxBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(
		pairs.SphereBox,
		func(pair [2]int) uint64 { return makeSphereBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells
	state.PhaseBroadphaseNanos = ensurePositiveDurationNanos(time.Since(broadphaseStart).Nanoseconds())

	solverStart := time.Now()
	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
	pass2SphereSphere := make([][2]int, 0, len(pairs.SphereSphere)/2)
	pass2BoxBox := make([][2]int, 0, len(pairs.BoxBox)/2)
	pass2SphereBox := make([][2]int, 0, len(pairs.SphereBox)/2)

	for _, pair := range pairs.SphereSphere {
		state.SolverPass1PairCount++
		first := pair[0]
		second := pair[1]
		if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
			state.SleepingSphereSphereSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeSphereSpherePersistentKey(first, second)
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[first])
			wakeRigidSphere(&state.RigidSpheres[second])
			state.RigidSphereSphereCollisionDetected = true
			state.SphereSphereHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereSphere = append(pass2SphereSphere, pair)
		}
	}
	for _, pair := range pairs.BoxBox {
		state.SolverPass1PairCount++
		first := pair[0]
		second := pair[1]
		if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
			state.SleepingBoxBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeBoxBoxPersistentKey(first, second)
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, manifold)
		if contact.Hit {
			wakeRigidBox(&state.RigidBoxes[first])
			wakeRigidBox(&state.RigidBoxes[second])
			state.RigidBoxBoxCollisionDetected = true
			state.BoxBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2BoxBox = append(pass2BoxBox, pair)
		}
	}
	for _, pair := range pairs.SphereBox {
		state.SolverPass1PairCount++
		sphereIndex := pair[0]
		boxIndex := pair[1]
		if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
			state.SleepingSphereBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[sphereIndex])
			wakeRigidBox(&state.RigidBoxes[boxIndex])
			state.SphereBoxCollisionDetected = true
			state.SphereBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereBox = append(pass2SphereBox, pair)
		}
	}

	state.SolverPass2PairCount = len(pass2SphereSphere) + len(pass2BoxBox) + len(pass2SphereBox)
	if state.SolverPass2PairCount < state.SolverPass1PairCount {
		state.EverReducedSolverPass2 = true
	}

	for _, pair := range pass2SphereSphere {
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
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereSpherePersistentKey(first, second), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	for _, pair := range pass2BoxBox {
		first := pair[0]
		second := pair[1]
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeBoxBoxPersistentKey(first, second), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	for _, pair := range pass2SphereBox {
		sphereIndex := pair[0]
		boxIndex := pair[1]
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	state.PhaseSolverNanos = ensurePositiveDurationNanos(time.Since(solverStart).Nanoseconds())

	sleepStart := time.Now()
	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)
	for index := range state.RigidSpheres {
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}
	state.PhaseSleepingNanos = ensurePositiveDurationNanos(time.Since(sleepStart).Nanoseconds())

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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveBudgetOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0
	state.PhaseIntegrationNanos = 0
	state.PhaseCCDNanos = 0
	state.PhaseBroadphaseNanos = 0
	state.PhaseSolverNanos = 0
	state.PhaseSleepingNanos = 0
	state.SolverPass1PairCount = 0
	state.SolverPass2PairCount = 0
	state.CCDBudgetSkipCount = 0
	state.SolverFrontierBudgetCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()

	integrationStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}
		physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
	}
	state.PhaseIntegrationNanos = ensurePositiveDurationNanos(time.Since(integrationStart).Nanoseconds())

	ccdStart := time.Now()
	sphereCCDSeen := 0
	boxCCDSeen := 0
	sphereCCDBudget := 1
	boxCCDBudget := 1
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			continue
		}
		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteSphereCount++
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDSphereCount++
		state.EverActivatedCCDSphere = true
		sphereCCDSeen++
		if sphereCCDSeen > sphereCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.SphereCCDPrecheckCount++
		state.EverRanSphereCCDPrecheck = true
		if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
			state.SphereCCDMeshSweepCount++
			state.EverExecutedSphereCCDMeshSweep = true
		} else {
			state.SphereCCDPrecheckRejectCount++
			state.EverRejectedSphereCCDPrecheck = true
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			continue
		}
		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteBoxCount++
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDBoxCount++
		state.EverActivatedCCDBox = true
		if dueToAngularRisk {
			state.ActiveAngularRiskCCDBoxCount++
			state.EverActivatedAngularRiskCCDBox = true
		}
		boxCCDSeen++
		if boxCCDSeen > boxCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.BoxCCDPrecheckCount++
		state.EverRanBoxCCDPrecheck = true
		if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
			state.BoxCCDMeshSweepCount++
			state.EverExecutedBoxCCDMeshSweep = true
		} else {
			state.BoxCCDPrecheckRejectCount++
			state.EverRejectedBoxCCDPrecheck = true
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	state.PhaseCCDNanos = ensurePositiveDurationNanos(time.Since(ccdStart).Nanoseconds())

	broadphaseStart := time.Now()
	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, fixed.FromInt(2))
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(
		pairs.SphereSphere,
		func(pair [2]int) uint64 { return makeSphereSpherePersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(
		pairs.BoxBox,
		func(pair [2]int) uint64 { return makeBoxBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(
		pairs.SphereBox,
		func(pair [2]int) uint64 { return makeSphereBoxPersistentKey(pair[0], pair[1]) },
		prevPersistent,
	)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells
	state.PhaseBroadphaseNanos = ensurePositiveDurationNanos(time.Since(broadphaseStart).Nanoseconds())

	solverStart := time.Now()
	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
	pass2SphereSphereScored := make([]scoredSphereSpherePair, 0, len(pairs.SphereSphere)/2)
	pass2BoxBoxScored := make([]scoredBoxBoxPair, 0, len(pairs.BoxBox)/2)
	pass2SphereBoxScored := make([]scoredSphereBoxPair, 0, len(pairs.SphereBox)/2)

	for _, pair := range pairs.SphereSphere {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
			state.SleepingSphereSphereSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeSphereSpherePersistentKey(first, second)
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[first])
			wakeRigidSphere(&state.RigidSpheres[second])
			state.RigidSphereSphereCollisionDetected = true
			state.SphereSphereHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereSphereScored = append(pass2SphereSphereScored, scoredSphereSpherePair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	for _, pair := range pairs.BoxBox {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
			state.SleepingBoxBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeBoxBoxPersistentKey(first, second)
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, manifold)
		if contact.Hit {
			wakeRigidBox(&state.RigidBoxes[first])
			wakeRigidBox(&state.RigidBoxes[second])
			state.RigidBoxBoxCollisionDetected = true
			state.BoxBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2BoxBoxScored = append(pass2BoxBoxScored, scoredBoxBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	for _, pair := range pairs.SphereBox {
		state.SolverPass1PairCount++
		sphereIndex, boxIndex := pair[0], pair[1]
		if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
			state.SleepingSphereBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			continue
		}
		key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[sphereIndex])
			wakeRigidBox(&state.RigidBoxes[boxIndex])
			state.SphereBoxCollisionDetected = true
			state.SphereBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereBoxScored = append(pass2SphereBoxScored, scoredSphereBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}

	totalPass2Candidates := len(pass2SphereSphereScored) + len(pass2BoxBoxScored) + len(pass2SphereBoxScored)
	solverBudget := totalPass2Candidates
	if totalPass2Candidates > 32 {
		solverBudget = (totalPass2Candidates * 2) / 3
	}
	if solverBudget < 1 && totalPass2Candidates > 0 {
		solverBudget = 1
	}
	if totalPass2Candidates > solverBudget {
		state.EverAppliedSolverBudget = true
	}
	state.SolverFrontierBudgetCount = solverBudget

	typeCounts := []int{len(pass2SphereSphereScored), len(pass2BoxBoxScored), len(pass2SphereBoxScored)}
	totalCandidates := typeCounts[0] + typeCounts[1] + typeCounts[2]
	sphereBudget, boxBudget, sphereBoxBudget := typeCounts[0], typeCounts[1], typeCounts[2]
	if totalCandidates > solverBudget {
		sphereBudget = solverBudget * typeCounts[0] / totalCandidates
		boxBudget = solverBudget * typeCounts[1] / totalCandidates
		sphereBoxBudget = solverBudget * typeCounts[2] / totalCandidates
		for sphereBudget+boxBudget+sphereBoxBudget < solverBudget {
			if sphereBudget < typeCounts[0] {
				sphereBudget++
			} else if boxBudget < typeCounts[1] {
				boxBudget++
			} else if sphereBoxBudget < typeCounts[2] {
				sphereBoxBudget++
			} else {
				break
			}
		}
	}
	pass2SphereSphere := trimScoredSphereSpherePairs(pass2SphereSphereScored, sphereBudget)
	pass2BoxBox := trimScoredBoxBoxPairs(pass2BoxBoxScored, boxBudget)
	pass2SphereBox := trimScoredSphereBoxPairs(pass2SphereBoxScored, sphereBoxBudget)

	state.SolverPass2PairCount = len(pass2SphereSphere) + len(pass2BoxBox) + len(pass2SphereBox)
	if state.SolverPass2PairCount < state.SolverPass1PairCount {
		state.EverReducedSolverPass2 = true
	}

	for _, pair := range pass2SphereSphere {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereSpherePersistentKey(first, second), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	for _, pair := range pass2BoxBox {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeBoxBoxPersistentKey(first, second), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	for _, pair := range pass2SphereBox {
		sphereIndex, boxIndex := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, physics.ContactWarmStartManifold{})
		if contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
		}
	}
	state.PhaseSolverNanos = ensurePositiveDurationNanos(time.Since(solverStart).Nanoseconds())

	sleepStart := time.Now()
	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)
	for index := range state.RigidSpheres {
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}
	state.PhaseSleepingNanos = ensurePositiveDurationNanos(time.Since(sleepStart).Nanoseconds())

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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxSmartAdaptiveBudgetOptimizedScene(state *SceneState) {
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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0
	state.PhaseIntegrationNanos = 0
	state.PhaseCCDNanos = 0
	state.PhaseBroadphaseNanos = 0
	state.PhaseSolverNanos = 0
	state.PhaseSleepingNanos = 0
	state.SolverPass1PairCount = 0
	state.SolverPass2PairCount = 0
	state.CCDBudgetSkipCount = 0
	state.SolverFrontierBudgetCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
	contactLoad := len(state.PersistentContactCache)

	integrationStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}
		physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
	}
	state.PhaseIntegrationNanos = ensurePositiveDurationNanos(time.Since(integrationStart).Nanoseconds())

	ccdStart := time.Now()
	sphereCCDSeen := 0
	boxCCDSeen := 0
	sphereCCDBudget := 8
	boxCCDBudget := 8
	switch {
	case contactLoad > 180:
		sphereCCDBudget, boxCCDBudget = 1, 1
	case contactLoad > 120:
		sphereCCDBudget, boxCCDBudget = 2, 2
	case contactLoad > 64:
		sphereCCDBudget, boxCCDBudget = 4, 4
	}
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			continue
		}
		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteSphereCount++
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDSphereCount++
		state.EverActivatedCCDSphere = true
		sphereCCDSeen++
		if sphereCCDSeen > sphereCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.SphereCCDPrecheckCount++
		state.EverRanSphereCCDPrecheck = true
		if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
			state.SphereCCDMeshSweepCount++
			state.EverExecutedSphereCCDMeshSweep = true
		} else {
			state.SphereCCDPrecheckRejectCount++
			state.EverRejectedSphereCCDPrecheck = true
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			continue
		}
		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteBoxCount++
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDBoxCount++
		state.EverActivatedCCDBox = true
		if dueToAngularRisk {
			state.ActiveAngularRiskCCDBoxCount++
			state.EverActivatedAngularRiskCCDBox = true
		}
		boxCCDSeen++
		if boxCCDSeen > boxCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.BoxCCDPrecheckCount++
		state.EverRanBoxCCDPrecheck = true
		if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
			state.BoxCCDMeshSweepCount++
			state.EverExecutedBoxCCDMeshSweep = true
		} else {
			state.BoxCCDPrecheckRejectCount++
			state.EverRejectedBoxCCDPrecheck = true
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	state.PhaseCCDNanos = ensurePositiveDurationNanos(time.Since(ccdStart).Nanoseconds())

	broadphaseStart := time.Now()
	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, fixed.FromInt(2))
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(pairs.SphereSphere, func(pair [2]int) uint64 {
		return makeSphereSpherePersistentKey(pair[0], pair[1])
	}, prevPersistent)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(pairs.BoxBox, func(pair [2]int) uint64 {
		return makeBoxBoxPersistentKey(pair[0], pair[1])
	}, prevPersistent)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(pairs.SphereBox, func(pair [2]int) uint64 {
		return makeSphereBoxPersistentKey(pair[0], pair[1])
	}, prevPersistent)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells
	state.PhaseBroadphaseNanos = ensurePositiveDurationNanos(time.Since(broadphaseStart).Nanoseconds())

	solverStart := time.Now()
	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
	pass2SphereSphereScored := make([]scoredSphereSpherePair, 0, len(pairs.SphereSphere)/2)
	pass2BoxBoxScored := make([]scoredBoxBoxPair, 0, len(pairs.BoxBox)/2)
	pass2SphereBoxScored := make([]scoredSphereBoxPair, 0, len(pairs.SphereBox)/2)

	resolveSphereSpherePass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
			state.SleepingSphereSphereSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeSphereSpherePersistentKey(first, second)
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[first])
			wakeRigidSphere(&state.RigidSpheres[second])
			state.RigidSphereSphereCollisionDetected = true
			state.SphereSphereHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereSphereScored = append(pass2SphereSphereScored, scoredSphereSpherePair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	resolveBoxBoxPass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
			state.SleepingBoxBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeBoxBoxPersistentKey(first, second)
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, manifold)
		if contact.Hit {
			wakeRigidBox(&state.RigidBoxes[first])
			wakeRigidBox(&state.RigidBoxes[second])
			state.RigidBoxBoxCollisionDetected = true
			state.BoxBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2BoxBoxScored = append(pass2BoxBoxScored, scoredBoxBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	resolveSphereBoxPass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		sphereIndex, boxIndex := pair[0], pair[1]
		if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
			state.SleepingSphereBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[sphereIndex])
			wakeRigidBox(&state.RigidBoxes[boxIndex])
			state.SphereBoxCollisionDetected = true
			state.SphereBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereBoxScored = append(pass2SphereBoxScored, scoredSphereBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	for _, pair := range pairs.SphereSphere {
		resolveSphereSpherePass1(pair)
	}
	for _, pair := range pairs.BoxBox {
		resolveBoxBoxPass1(pair)
	}
	for _, pair := range pairs.SphereBox {
		resolveSphereBoxPass1(pair)
	}

	totalPass2Candidates := len(pass2SphereSphereScored) + len(pass2BoxBoxScored) + len(pass2SphereBoxScored)
	solverBudget := totalPass2Candidates
	switch {
	case totalPass2Candidates > 192:
		solverBudget = totalPass2Candidates / 2
	case totalPass2Candidates > 96:
		solverBudget = (totalPass2Candidates * 3) / 5
	case totalPass2Candidates > 32:
		solverBudget = (totalPass2Candidates * 3) / 4
	}
	if solverBudget < 1 && totalPass2Candidates > 0 {
		solverBudget = 1
	}
	if totalPass2Candidates > solverBudget {
		state.EverAppliedSolverBudget = true
	}
	state.SolverFrontierBudgetCount = solverBudget

	typeCounts := []int{len(pass2SphereSphereScored), len(pass2BoxBoxScored), len(pass2SphereBoxScored)}
	totalCandidates := typeCounts[0] + typeCounts[1] + typeCounts[2]
	sphereBudget, boxBudget, sphereBoxBudget := typeCounts[0], typeCounts[1], typeCounts[2]
	if totalCandidates > solverBudget {
		sphereBudget = solverBudget * typeCounts[0] / totalCandidates
		boxBudget = solverBudget * typeCounts[1] / totalCandidates
		sphereBoxBudget = solverBudget * typeCounts[2] / totalCandidates
		for sphereBudget+boxBudget+sphereBoxBudget < solverBudget {
			if sphereBudget < typeCounts[0] {
				sphereBudget++
			} else if boxBudget < typeCounts[1] {
				boxBudget++
			} else if sphereBoxBudget < typeCounts[2] {
				sphereBoxBudget++
			} else {
				break
			}
		}
	}
	pass2SphereSphere := trimScoredSphereSpherePairs(pass2SphereSphereScored, sphereBudget)
	pass2BoxBox := trimScoredBoxBoxPairs(pass2BoxBoxScored, boxBudget)
	pass2SphereBox := trimScoredSphereBoxPairs(pass2SphereBoxScored, sphereBoxBudget)
	state.SolverPass2PairCount = len(pass2SphereSphere) + len(pass2BoxBox) + len(pass2SphereBox)
	if state.SolverPass2PairCount < state.SolverPass1PairCount {
		state.EverReducedSolverPass2 = true
	}

	for _, pair := range pass2SphereSphere {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		if contact, updated := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereSpherePersistentKey(first, second), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	for _, pair := range pass2BoxBox {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		if contact, updated := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeBoxBoxPersistentKey(first, second), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	for _, pair := range pass2SphereBox {
		sphereIndex, boxIndex := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		if contact, updated := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	state.PhaseSolverNanos = ensurePositiveDurationNanos(time.Since(solverStart).Nanoseconds())

	sleepStart := time.Now()
	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)
	for index := range state.RigidSpheres {
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}
	state.PhaseSleepingNanos = ensurePositiveDurationNanos(time.Since(sleepStart).Nanoseconds())

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

func StepHundredRigidSpheresAndHundredRigidBoxesInBoxTimingAwareAdaptiveBudgetOptimizedScene(state *SceneState) {
	if state == nil {
		return
	}

	prevSphereCCDLoad := state.SphereCCDMeshSweepCount + state.BoxCCDMeshSweepCount + state.ActiveCCDSphereCount + state.ActiveCCDBoxCount
	prevBroadphaseLoad := state.SphereSphereCandidateCount + state.BoxBoxCandidateCount + state.SphereBoxCandidateCount
	prevSolverLoad := state.SolverPass1PairCount + state.SolverPass2PairCount + state.SphereSphereHitCount + state.BoxBoxHitCount + state.SphereBoxHitCount

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
	state.SphereCCDPrecheckCount = 0
	state.SphereCCDPrecheckRejectCount = 0
	state.SphereCCDMeshSweepCount = 0
	state.SphereCCDHysteresisHoldCount = 0
	state.BoxCCDHysteresisHoldCount = 0
	state.SphereSleepHysteresisHoldCount = 0
	state.BoxSleepHysteresisHoldCount = 0
	state.PersistentPairReuseCount = 0
	state.WarmStartedPairCount = 0
	state.SleepingIslandCount = 0
	state.LargestSleepingIslandSize = 0
	state.PhaseIntegrationNanos = 0
	state.PhaseCCDNanos = 0
	state.PhaseBroadphaseNanos = 0
	state.PhaseSolverNanos = 0
	state.PhaseSleepingNanos = 0
	state.SolverPass1PairCount = 0
	state.SolverPass2PairCount = 0
	state.CCDBudgetSkipCount = 0
	state.SolverFrontierBudgetCount = 0

	minX, maxX, minZ, maxZ, _ := openBoxContainerParameters()
	contactLoad := len(state.PersistentContactCache)
	minInt := func(a, b int) int {
		if a < b {
			return a
		}
		return b
	}
	maxInt := func(a, b int) int {
		if a > b {
			return a
		}
		return b
	}

	integrationStart := time.Now()
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			state.SleepingSphereCount++
			continue
		}
		physics.ApplyForce(&body.Motion, physics.ComputeGravityForce(body.Motion.Mass, physics.StandardGravity))
		physics.AdvanceRigidSphereBody3D(body, physics.DefaultTimeStep)
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			state.SleepingBoxCount++
			continue
		}
		physics.AdvanceRigidBoxBody3D(body, physics.DefaultTimeStep, physics.StandardGravity)
	}
	state.PhaseIntegrationNanos = ensurePositiveDurationNanos(time.Since(integrationStart).Nanoseconds())

	ccdStart := time.Now()
	sphereCCDSeen := 0
	boxCCDSeen := 0
	sphereCCDBudget := 8
	boxCCDBudget := 8
	switch {
	case contactLoad > 180:
		sphereCCDBudget, boxCCDBudget = 1, 1
	case contactLoad > 120:
		sphereCCDBudget, boxCCDBudget = 2, 2
	case contactLoad > 64:
		sphereCCDBudget, boxCCDBudget = 4, 4
	}
	switch {
	case prevSphereCCDLoad > 140 || prevBroadphaseLoad > 1200 || prevSolverLoad > 700:
		sphereCCDBudget = minInt(sphereCCDBudget, 1)
		boxCCDBudget = minInt(boxCCDBudget, 1)
	case prevSphereCCDLoad > 90 || prevBroadphaseLoad > 800 || prevSolverLoad > 450:
		sphereCCDBudget = minInt(sphereCCDBudget, 2)
		boxCCDBudget = minInt(boxCCDBudget, 2)
	case prevSphereCCDLoad > 45 || prevBroadphaseLoad > 500 || prevSolverLoad > 250:
		sphereCCDBudget = minInt(sphereCCDBudget, 4)
		boxCCDBudget = minInt(boxCCDBudget, 4)
	}
	for index := range state.RigidSpheres {
		body := &state.RigidSpheres[index]
		if body.Sleeping {
			continue
		}
		useCCD, heldByHysteresis := shouldUseRigidSphereCCDHysteresis(body)
		if heldByHysteresis {
			state.SphereCCDHysteresisHoldCount++
			state.EverHeldSphereCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteSphereCount++
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDSphereCount++
		state.EverActivatedCCDSphere = true
		sphereCCDSeen++
		if sphereCCDSeen > sphereCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.SphereCCDPrecheckCount++
		state.EverRanSphereCCDPrecheck = true
		if cheapPrecheckRigidSphereContainerCCD(*body, minX, maxX, minZ, maxZ) {
			state.SphereCCDMeshSweepCount++
			state.EverExecutedSphereCCDMeshSweep = true
		} else {
			state.SphereCCDPrecheckRejectCount++
			state.EverRejectedSphereCCDPrecheck = true
		}
		if physics.ConstrainRigidSphereBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	for index := range state.RigidBoxes {
		body := &state.RigidBoxes[index]
		if body.Sleeping {
			continue
		}
		useCCD, dueToAngularRisk, heldByHysteresis := shouldUseRigidBoxCCDHysteresis(body)
		if heldByHysteresis {
			state.BoxCCDHysteresisHoldCount++
			state.EverHeldBoxCCDHysteresis = true
		}
		if !useCCD {
			state.ActiveDiscreteBoxCount++
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.ActiveCCDBoxCount++
		state.EverActivatedCCDBox = true
		if dueToAngularRisk {
			state.ActiveAngularRiskCCDBoxCount++
			state.EverActivatedAngularRiskCCDBox = true
		}
		boxCCDSeen++
		if boxCCDSeen > boxCCDBudget {
			state.CCDBudgetSkipCount++
			state.EverAppliedCCDBudget = true
			if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
				state.EverTouchedGround = true
			}
			continue
		}
		state.BoxCCDPrecheckCount++
		state.EverRanBoxCCDPrecheck = true
		if cheapPrecheckRigidBoxMeshCCD(*body, state.GroundTriangles) {
			state.BoxCCDMeshSweepCount++
			state.EverExecutedBoxCCDMeshSweep = true
		} else {
			state.BoxCCDPrecheckRejectCount++
			state.EverRejectedBoxCCDPrecheck = true
		}
		if physics.ConstrainRigidBoxBody3DToOpenContainer(body, minX, maxX, minZ, maxZ, fixed.Zero) {
			state.EverTouchedGround = true
		}
	}
	state.PhaseCCDNanos = ensurePositiveDurationNanos(time.Since(ccdStart).Nanoseconds())

	broadphaseStart := time.Now()
	prevPersistent := persistentContactCacheMap(state.PersistentContactCache)
	pairs := buildMixedRigidBroadphasePairs(state.RigidSpheres, state.RigidBoxes, fixed.FromInt(2))
	pairs.SphereSphere, state.PersistentPairReuseCount = reorderPersistentPairs(pairs.SphereSphere, func(pair [2]int) uint64 {
		return makeSphereSpherePersistentKey(pair[0], pair[1])
	}, prevPersistent)
	var reused int
	pairs.BoxBox, reused = reorderPersistentPairs(pairs.BoxBox, func(pair [2]int) uint64 {
		return makeBoxBoxPersistentKey(pair[0], pair[1])
	}, prevPersistent)
	state.PersistentPairReuseCount += reused
	pairs.SphereBox, reused = reorderPersistentPairs(pairs.SphereBox, func(pair [2]int) uint64 {
		return makeSphereBoxPersistentKey(pair[0], pair[1])
	}, prevPersistent)
	state.PersistentPairReuseCount += reused
	if state.PersistentPairReuseCount > 0 {
		state.EverReusedPersistentPairs = true
	}
	state.BroadphaseCellCount = pairs.CellCount
	state.SphereSphereCandidateCount = len(pairs.SphereSphere)
	state.BoxBoxCandidateCount = len(pairs.BoxBox)
	state.SphereBoxCandidateCount = len(pairs.SphereBox)
	state.BroadphaseDebugCells = pairs.Cells
	state.PhaseBroadphaseNanos = ensurePositiveDurationNanos(time.Since(broadphaseStart).Nanoseconds())

	solverStart := time.Now()
	currentPersistentHits := make(map[uint64]PersistentContactCacheEntry)
	pass2SphereSphereScored := make([]scoredSphereSpherePair, 0, len(pairs.SphereSphere)/2)
	pass2BoxBoxScored := make([]scoredBoxBoxPair, 0, len(pairs.BoxBox)/2)
	pass2SphereBoxScored := make([]scoredSphereBoxPair, 0, len(pairs.SphereBox)/2)

	resolveSphereSpherePass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidSpheres[first].Sleeping && state.RigidSpheres[second].Sleeping {
			state.SleepingSphereSphereSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeSphereSpherePersistentKey(first, second)
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[first])
			wakeRigidSphere(&state.RigidSpheres[second])
			state.RigidSphereSphereCollisionDetected = true
			state.SphereSphereHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereSphereScored = append(pass2SphereSphereScored, scoredSphereSpherePair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	resolveBoxBoxPass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		first, second := pair[0], pair[1]
		if state.RigidBoxes[first].Sleeping && state.RigidBoxes[second].Sleeping {
			state.SleepingBoxBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeBoxBoxPersistentKey(first, second)
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, manifold)
		if contact.Hit {
			wakeRigidBox(&state.RigidBoxes[first])
			wakeRigidBox(&state.RigidBoxes[second])
			state.RigidBoxBoxCollisionDetected = true
			state.BoxBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2BoxBoxScored = append(pass2BoxBoxScored, scoredBoxBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	resolveSphereBoxPass1 := func(pair [2]int) {
		state.SolverPass1PairCount++
		sphereIndex, boxIndex := pair[0], pair[1]
		if state.RigidSpheres[sphereIndex].Sleeping && state.RigidBoxes[boxIndex].Sleeping {
			state.SleepingSphereBoxSkipCount++
			state.EverSkippedSleepingPairs = true
			return
		}
		key := makeSphereBoxPersistentKey(sphereIndex, boxIndex)
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		manifold, usedWarmStart := physics.ContactWarmStartManifold{}, false
		if entry, exists := prevPersistent[key]; exists && hasUsefulWarmStart(entry) {
			manifold = persistentEntryToWarmStart(entry)
			usedWarmStart = true
		}
		contact, updatedManifold := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, manifold)
		if contact.Hit {
			wakeRigidSphere(&state.RigidSpheres[sphereIndex])
			wakeRigidBox(&state.RigidBoxes[boxIndex])
			state.SphereBoxCollisionDetected = true
			state.SphereBoxHitCount++
			if usedWarmStart {
				state.WarmStartedPairCount++
				state.EverWarmStartedPairs = true
			}
			appendPersistentContactManifold(currentPersistentHits, key, updatedManifold.Point, updatedManifold.Normal, updatedManifold.NormalImpulse, updatedManifold.TangentImpulse)
			pass2SphereBoxScored = append(pass2SphereBoxScored, scoredSphereBoxPair{Pair: pair, Score: contactScoreFromManifold(updatedManifold)})
		}
	}
	for _, pair := range pairs.SphereSphere {
		resolveSphereSpherePass1(pair)
	}
	for _, pair := range pairs.BoxBox {
		resolveBoxBoxPass1(pair)
	}
	for _, pair := range pairs.SphereBox {
		resolveSphereBoxPass1(pair)
	}

	totalPass2Candidates := len(pass2SphereSphereScored) + len(pass2BoxBoxScored) + len(pass2SphereBoxScored)
	solverBudget := totalPass2Candidates
	switch {
	case totalPass2Candidates > 192:
		solverBudget = totalPass2Candidates / 2
	case totalPass2Candidates > 96:
		solverBudget = (totalPass2Candidates * 3) / 5
	case totalPass2Candidates > 32:
		solverBudget = (totalPass2Candidates * 3) / 4
	}
	switch {
	case prevSolverLoad > 700 || prevBroadphaseLoad > 1200:
		solverBudget = minInt(solverBudget, maxInt(1, totalPass2Candidates/2))
	case prevSolverLoad > 450 || prevBroadphaseLoad > 800:
		solverBudget = minInt(solverBudget, maxInt(1, (totalPass2Candidates*3)/5))
	case prevSolverLoad > 250 || prevBroadphaseLoad > 500:
		solverBudget = minInt(solverBudget, maxInt(1, (totalPass2Candidates*3)/4))
	}
	if solverBudget < 1 && totalPass2Candidates > 0 {
		solverBudget = 1
	}
	if totalPass2Candidates > solverBudget {
		state.EverAppliedSolverBudget = true
	}
	state.SolverFrontierBudgetCount = solverBudget

	typeCounts := []int{len(pass2SphereSphereScored), len(pass2BoxBoxScored), len(pass2SphereBoxScored)}
	totalCandidates := typeCounts[0] + typeCounts[1] + typeCounts[2]
	sphereBudget, boxBudget, sphereBoxBudget := typeCounts[0], typeCounts[1], typeCounts[2]
	if totalCandidates > solverBudget {
		sphereBudget = solverBudget * typeCounts[0] / totalCandidates
		boxBudget = solverBudget * typeCounts[1] / totalCandidates
		sphereBoxBudget = solverBudget * typeCounts[2] / totalCandidates
		for sphereBudget+boxBudget+sphereBoxBudget < solverBudget {
			if sphereBudget < typeCounts[0] {
				sphereBudget++
			} else if boxBudget < typeCounts[1] {
				boxBudget++
			} else if sphereBoxBudget < typeCounts[2] {
				sphereBoxBudget++
			} else {
				break
			}
		}
	}
	pass2SphereSphere := trimScoredSphereSpherePairs(pass2SphereSphereScored, sphereBudget)
	pass2BoxBox := trimScoredBoxBoxPairs(pass2BoxBoxScored, boxBudget)
	pass2SphereBox := trimScoredSphereBoxPairs(pass2SphereBoxScored, sphereBoxBudget)
	state.SolverPass2PairCount = len(pass2SphereSphere) + len(pass2BoxBox) + len(pass2SphereBox)
	if state.SolverPass2PairCount < state.SolverPass1PairCount {
		state.EverReducedSolverPass2 = true
	}

	for _, pair := range pass2SphereSphere {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[first].Restitution.Add(state.RigidSpheres[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		combinedFriction := state.RigidSpheres[first].Friction.Add(state.RigidSpheres[second].Friction)
		if combinedFriction.Cmp(fixed.One) > 0 {
			combinedFriction = fixed.One
		}
		if contact, updated := physics.ResolveRigidSphereSphereContactWithFrictionWarmStart(&state.RigidSpheres[first], &state.RigidSpheres[second], combinedRestitution, combinedFriction, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereSpherePersistentKey(first, second), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	for _, pair := range pass2BoxBox {
		first, second := pair[0], pair[1]
		combinedRestitution := state.RigidBoxes[first].Restitution.Add(state.RigidBoxes[second].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		if contact, updated := physics.ResolveRigidBoxBoxContactWarmStart(&state.RigidBoxes[first], &state.RigidBoxes[second], combinedRestitution, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeBoxBoxPersistentKey(first, second), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	for _, pair := range pass2SphereBox {
		sphereIndex, boxIndex := pair[0], pair[1]
		combinedRestitution := state.RigidSpheres[sphereIndex].Restitution.Add(state.RigidBoxes[boxIndex].Restitution)
		if combinedRestitution.Cmp(fixed.One) > 0 {
			combinedRestitution = fixed.One
		}
		if contact, updated := physics.ResolveRigidSphereRigidBoxContactWithFrictionWarmStart(&state.RigidSpheres[sphereIndex], &state.RigidBoxes[boxIndex], combinedRestitution, state.RigidSpheres[sphereIndex].Friction, physics.ContactWarmStartManifold{}); contact.Hit {
			appendPersistentContactManifold(currentPersistentHits, makeSphereBoxPersistentKey(sphereIndex, boxIndex), updated.Point, updated.Normal, updated.NormalImpulse, updated.TangentImpulse)
		}
	}
	state.PhaseSolverNanos = ensurePositiveDurationNanos(time.Since(solverStart).Nanoseconds())

	sleepStart := time.Now()
	nextPersistentContacts := make([]PersistentContactCacheEntry, 0, len(currentPersistentHits))
	for _, entry := range currentPersistentHits {
		nextPersistentContacts = append(nextPersistentContacts, entry)
	}
	sortPersistentContactCache(nextPersistentContacts)
	state.PersistentContactCache = nextPersistentContacts
	applySleepingIslands(state, nextPersistentContacts)
	for index := range state.RigidSpheres {
		if updateRigidSphereSleepStateWithHysteresis(&state.RigidSpheres[index]) {
			state.SphereSleepHysteresisHoldCount++
			state.EverHeldSphereSleepHysteresis = true
		}
		if state.RigidSpheres[index].Sleeping {
			state.EverSleptSphere = true
		}
	}
	for index := range state.RigidBoxes {
		if updateRigidBoxSleepStateWithHysteresis(&state.RigidBoxes[index]) {
			state.BoxSleepHysteresisHoldCount++
			state.EverHeldBoxSleepHysteresis = true
		}
		if state.RigidBoxes[index].Sleeping {
			state.EverSleptBox = true
		}
	}
	state.PhaseSleepingNanos = ensurePositiveDurationNanos(time.Since(sleepStart).Nanoseconds())

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
