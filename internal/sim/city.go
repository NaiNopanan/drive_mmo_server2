package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type CityObstacle struct {
	MinX, MaxX fixed.Fixed
	MinZ, MaxZ fixed.Fixed
	BaseY      fixed.Fixed
	Height     fixed.Fixed
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
		cityV3(-28, 0, -18),
		cityV3(-18, 0, -10),
		cityV3(-14, 2, -6),
		cityV3(-8, 6, -1),
		cityV3(0, 6, 0),
		cityV3(8, 6, 1),
		cityV3(14, 2, 6),
		cityV3(18, 0, 10),
		cityV3(32, 0, 18),
		cityV3(48, 0, 28),
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

			rails = append(rails, CityObstacle{
				MinX:   minX.Sub(thickness),
				MaxX:   maxX.Add(thickness),
				MinZ:   minZ.Sub(thickness),
				MaxZ:   maxZ.Add(thickness),
				BaseY:  baseY,
				Height: topY.Sub(baseY).Add(railTop),
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
