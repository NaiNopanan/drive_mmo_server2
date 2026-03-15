package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type CityObstacle struct {
	MinX, MaxX  fixed.Fixed
	MinZ, MaxZ  fixed.Fixed
	BaseY       fixed.Fixed
	Height      fixed.Fixed
	ForwardXZ   geom.Vec3
	HalfForward fixed.Fixed
	HalfRight   fixed.Fixed
}

func (o CityObstacle) TopY() fixed.Fixed {
	return o.BaseY.Add(o.Height)
}

func (o CityObstacle) Center() geom.Vec3 {
	two := fixed.FromInt(2)
	return geom.V3(
		o.MinX.Add(o.MaxX).Div(two),
		o.BaseY.Add(o.Height.Div(two)),
		o.MinZ.Add(o.MaxZ).Div(two),
	)
}

func (o CityObstacle) Size() geom.Vec3 {
	return geom.V3(
		o.MaxX.Sub(o.MinX),
		o.Height,
		o.MaxZ.Sub(o.MinZ),
	)
}

func (o CityObstacle) OrientedAxes() (forward geom.Vec3, right geom.Vec3, halfForward fixed.Fixed, halfRight fixed.Fixed) {
	if o.ForwardXZ.LengthSq().Cmp(fixed.Zero) != 0 &&
		o.HalfForward.Cmp(fixed.Zero) > 0 &&
		o.HalfRight.Cmp(fixed.Zero) > 0 {
		forward = geom.V3(o.ForwardXZ.X, fixed.Zero, o.ForwardXZ.Z).Normalize()
		right = geom.V3(forward.Z, fixed.Zero, forward.X.Neg())
		return forward, right, o.HalfForward, o.HalfRight
	}

	return geom.V3(fixed.Zero, fixed.Zero, fixed.One),
		geom.V3(fixed.One, fixed.Zero, fixed.Zero),
		o.MaxZ.Sub(o.MinZ).Div(fixed.FromInt(2)),
		o.MaxX.Sub(o.MinX).Div(fixed.FromInt(2))
}

type CityRoadPath struct {
	Points []geom.Vec3
}

type CityMap struct {
	Ground        WorldGroundQuery
	Obstacles     []CityObstacle
	Bounds        WallBounds
	SpawnPosition geom.Vec3
	SpawnYaw      fixed.Fixed
	Buildings     []CityObstacle
	GuardRails    []CityObstacle
	RoadSurfaces  []geom.Triangle
	LanePaths     []CityRoadPath
	Sidewalks     []CityObstacle
}

func BuildCurvedOverpassCity() CityMap {
	worldMin := fixed.FromInt(-80)
	worldMax := fixed.FromInt(80)

	boulevardCenter := []geom.Vec3{
		cityV3(-70, 0, -42),
		cityV3(-56, 0, -36),
		cityV3(-42, 0, -28),
		cityV3(-34, 0, -22),
		cityV3(-26, 0, -16),
		cityV3(-20, 0, -12),
		cityV3(-16, 0, -9),
		cityV3(-13, 1, -7),
		cityV3(-10, 2, -5),
		cityV3(-7, 3, -3),
		cityV3(-4, 4, -2),
		cityV3(-2, 5, -1),
		cityV3(0, 6, 0),
		cityV3(2, 6, 1),
		cityV3(5, 5, 2),
		cityV3(8, 4, 4),
		cityV3(11, 3, 6),
		cityV3(14, 2, 8),
		cityV3(17, 1, 10),
		cityV3(22, 0, 13),
		cityV3(30, 0, 17),
		cityV3(40, 0, 23),
		cityV3(52, 0, 31),
		cityV3(64, 0, 38),
	}

	boulevardWidth := fixed.FromInt(12)
	boulevardRoad, leftEdge, rightEdge := buildRoadStrip(boulevardCenter, boulevardWidth)

	groundTriangles := make([]geom.Triangle, 0, 2+len(boulevardRoad))
	groundTriangles = appendRectTriangles(
		groundTriangles,
		worldMin, worldMax,
		worldMin, worldMax,
		fixed.Zero,
	)
	groundTriangles = append(groundTriangles, boulevardRoad...)

	roadSurfaces := make([]geom.Triangle, 0, len(boulevardRoad)+10)
	roadSurfaces = append(roadSurfaces, boulevardRoad...)

	lanePaths := []CityRoadPath{
		{Points: clonePoints(boulevardCenter)},
	}

	roadSurfaces = appendRectTriangles(
		roadSurfaces,
		fixed.FromInt(-5), fixed.FromInt(5),
		worldMin, worldMax,
		fixed.Zero,
	)
	lanePaths = append(lanePaths, CityRoadPath{
		Points: []geom.Vec3{
			cityV3(0, 0, -78),
			cityV3(0, 0, 78),
		},
	})

	secondaryRoads := []struct {
		minX, maxX fixed.Fixed
		minZ, maxZ fixed.Fixed
		path       []geom.Vec3
	}{
		{
			minX: fixed.FromInt(-80), maxX: fixed.FromInt(80),
			minZ: fixed.FromInt(-36), maxZ: fixed.FromInt(-28),
			path: []geom.Vec3{cityV3(-78, 0, -32), cityV3(78, 0, -32)},
		},
		{
			minX: fixed.FromInt(-80), maxX: fixed.FromInt(80),
			minZ: fixed.FromInt(28), maxZ: fixed.FromInt(36),
			path: []geom.Vec3{cityV3(-78, 0, 32), cityV3(78, 0, 32)},
		},
		{
			minX: fixed.FromInt(-48), maxX: fixed.FromInt(-40),
			minZ: fixed.FromInt(-80), maxZ: fixed.FromInt(80),
			path: []geom.Vec3{cityV3(-44, 0, -78), cityV3(-44, 0, 78)},
		},
		{
			minX: fixed.FromInt(40), maxX: fixed.FromInt(48),
			minZ: fixed.FromInt(-80), maxZ: fixed.FromInt(80),
			path: []geom.Vec3{cityV3(44, 0, -78), cityV3(44, 0, 78)},
		},
	}
	for _, r := range secondaryRoads {
		roadSurfaces = appendRectTriangles(roadSurfaces, r.minX, r.maxX, r.minZ, r.maxZ, fixed.Zero)
		lanePaths = append(lanePaths, CityRoadPath{Points: clonePoints(r.path)})
	}

	buildings := []CityObstacle{
		cityObstacle(-72, -54, -72, -52, 0, 18),
		cityObstacle(-52, -28, -72, -50, 0, 14),
		cityObstacle(-72, -56, -20, 20, 0, 22),
		cityObstacle(-72, -52, 46, 70, 0, 20),
		cityObstacle(-30, -16, 44, 64, 0, 16),
		cityObstacle(18, 36, -72, -48, 0, 17),
		cityObstacle(54, 72, -68, -46, 0, 21),
		cityObstacle(56, 72, -18, 18, 0, 24),
		cityObstacle(18, 38, 48, 70, 0, 19),
		cityObstacle(54, 72, 50, 72, 0, 15),
	}

	sidewalks := make([]CityObstacle, 0, len(buildings))
	for _, building := range buildings {
		sidewalks = append(sidewalks, inflateObstacle(building, fixed.FromInt(2), fixed.FromFraction(3, 20)))
	}

	guardRails := buildGuardRailObstacles(boulevardCenter, leftEdge, rightEdge)

	obstacles := make([]CityObstacle, 0, len(buildings)+len(guardRails))
	obstacles = append(obstacles, buildings...)
	obstacles = append(obstacles, guardRails...)

	return CityMap{
		Ground: WorldGroundQuery{
			Triangles: groundTriangles,
		},
		Obstacles: obstacles,
		Bounds: WallBounds{
			MinX: worldMin,
			MaxX: worldMax,
			MinZ: worldMin,
			MaxZ: worldMax,
		},
		SpawnPosition: geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-68)),
		SpawnYaw:      fixed.Zero,
		Buildings:     buildings,
		GuardRails:    guardRails,
		RoadSurfaces:  roadSurfaces,
		LanePaths:     lanePaths,
		Sidewalks:     sidewalks,
	}
}

func buildRoadStrip(points []geom.Vec3, width fixed.Fixed) ([]geom.Triangle, []geom.Vec3, []geom.Vec3) {
	if len(points) < 2 {
		return nil, nil, nil
	}

	halfWidth := width.Div(fixed.FromInt(2))
	left := make([]geom.Vec3, len(points))
	right := make([]geom.Vec3, len(points))

	for i := range points {
		prev := points[i]
		next := points[i]
		if i > 0 {
			prev = points[i-1]
		}
		if i+1 < len(points) {
			next = points[i+1]
		}

		tangent := geom.V3(next.X.Sub(prev.X), fixed.Zero, next.Z.Sub(prev.Z))
		if tangent.LengthSq().Cmp(fixed.Zero) == 0 {
			tangent = geom.V3(fixed.Zero, fixed.Zero, fixed.One)
		}
		tangent = tangent.Normalize()
		rightDir := geom.V3(tangent.Z, fixed.Zero, tangent.X.Neg())
		offset := rightDir.Scale(halfWidth)

		left[i] = points[i].Sub(offset)
		right[i] = points[i].Add(offset)
	}

	tris := make([]geom.Triangle, 0, (len(points)-1)*2)
	for i := 0; i+1 < len(points); i++ {
		tris = appendStripQuad(tris, left[i], left[i+1], right[i], right[i+1])
	}

	return tris, left, right
}

func buildGuardRailObstacles(center, leftEdge, rightEdge []geom.Vec3) []CityObstacle {
	thickness := fixed.FromFraction(3, 5)
	railTop := fixed.FromFraction(6, 5)
	rails := make([]CityObstacle, 0, 12)

	appendRailSegments := func(edge []geom.Vec3) {
		for i := 0; i+1 < len(edge); i++ {
			if center[i].Y.Cmp(fixed.FromInt(2)) < 0 || center[i+1].Y.Cmp(fixed.FromInt(2)) < 0 {
				continue
			}

			minX, maxX := minMaxFixed(edge[i].X, edge[i+1].X)
			minZ, maxZ := minMaxFixed(edge[i].Z, edge[i+1].Z)
			baseY, topY := minMaxFixed(edge[i].Y, edge[i+1].Y)
			delta := geom.V3(edge[i+1].X.Sub(edge[i].X), fixed.Zero, edge[i+1].Z.Sub(edge[i].Z))
			forward := delta.Normalize()
			if forward.LengthSq().Cmp(fixed.Zero) == 0 {
				forward = geom.V3(fixed.Zero, fixed.Zero, fixed.One)
			}
			right := geom.V3(forward.Z, fixed.Zero, forward.X.Neg())
			centerXZ := geom.V3(
				edge[i].X.Add(edge[i+1].X).Div(fixed.FromInt(2)),
				fixed.Zero,
				edge[i].Z.Add(edge[i+1].Z).Div(fixed.FromInt(2)),
			)
			halfForward := delta.Length().Div(fixed.FromInt(2)).Add(thickness)
			halfRight := thickness
			corners := [4]geom.Vec3{
				centerXZ.Add(forward.Scale(halfForward)).Add(right.Scale(halfRight)),
				centerXZ.Add(forward.Scale(halfForward)).Add(right.Scale(halfRight.Neg())),
				centerXZ.Add(forward.Scale(halfForward.Neg())).Add(right.Scale(halfRight)),
				centerXZ.Add(forward.Scale(halfForward.Neg())).Add(right.Scale(halfRight.Neg())),
			}
			minX = corners[0].X
			maxX = corners[0].X
			minZ = corners[0].Z
			maxZ = corners[0].Z
			for j := 1; j < len(corners); j++ {
				minX, maxX = minMaxFixed3(minX, maxX, corners[j].X)
				minZ, maxZ = minMaxFixed3(minZ, maxZ, corners[j].Z)
			}

			rails = append(rails, CityObstacle{
				MinX:        minX,
				MaxX:        maxX,
				MinZ:        minZ,
				MaxZ:        maxZ,
				BaseY:       baseY,
				Height:      topY.Sub(baseY).Add(railTop),
				ForwardXZ:   forward,
				HalfForward: halfForward,
				HalfRight:   halfRight,
			})
		}
	}

	appendRailSegments(leftEdge)
	appendRailSegments(rightEdge)

	return rails
}

func appendRectTriangles(dst []geom.Triangle, minX, maxX, minZ, maxZ, y fixed.Fixed) []geom.Triangle {
	a := geom.V3(minX, y, minZ)
	b := geom.V3(maxX, y, minZ)
	c := geom.V3(minX, y, maxZ)
	d := geom.V3(maxX, y, maxZ)
	return appendStripQuad(dst, a, c, b, d)
}

func appendStripQuad(dst []geom.Triangle, left0, left1, right0, right1 geom.Vec3) []geom.Triangle {
	dst = append(dst, geom.NewTriangle(left0, left1, right0))
	dst = append(dst, geom.NewTriangle(left1, right1, right0))
	return dst
}

func inflateObstacle(o CityObstacle, border, height fixed.Fixed) CityObstacle {
	return CityObstacle{
		MinX:   o.MinX.Sub(border),
		MaxX:   o.MaxX.Add(border),
		MinZ:   o.MinZ.Sub(border),
		MaxZ:   o.MaxZ.Add(border),
		BaseY:  o.BaseY,
		Height: height,
	}
}

func cityObstacle(minX, maxX, minZ, maxZ, baseY, height int64) CityObstacle {
	return CityObstacle{
		MinX:   fixed.FromInt(minX),
		MaxX:   fixed.FromInt(maxX),
		MinZ:   fixed.FromInt(minZ),
		MaxZ:   fixed.FromInt(maxZ),
		BaseY:  fixed.FromInt(baseY),
		Height: fixed.FromInt(height),
	}
}

func cityV3(x, y, z int64) geom.Vec3 {
	return geom.V3(fixed.FromInt(x), fixed.FromInt(y), fixed.FromInt(z))
}

func clonePoints(points []geom.Vec3) []geom.Vec3 {
	out := make([]geom.Vec3, len(points))
	copy(out, points)
	return out
}

func minMaxFixed(a, b fixed.Fixed) (fixed.Fixed, fixed.Fixed) {
	if a.Cmp(b) <= 0 {
		return a, b
	}
	return b, a
}

func minMaxFixed3(minv, maxv, candidate fixed.Fixed) (fixed.Fixed, fixed.Fixed) {
	if candidate.Cmp(minv) < 0 {
		minv = candidate
	}
	if candidate.Cmp(maxv) > 0 {
		maxv = candidate
	}
	return minv, maxv
}
