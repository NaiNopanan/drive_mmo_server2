package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func TestBuildCurvedOverpassCity_DeterministicCounts(t *testing.T) {
	city := BuildCurvedOverpassCity()

	if got := len(city.Ground.Triangles); got != 28 {
		t.Fatalf("ground triangle count mismatch: got=%d want=28", got)
	}
	if got := len(city.RoadSurfaces); got != 36 {
		t.Fatalf("road surface count mismatch: got=%d want=36", got)
	}
	if got := len(city.Buildings); got != 10 {
		t.Fatalf("building count mismatch: got=%d want=10", got)
	}
	if got := len(city.GuardRails); got != 8 {
		t.Fatalf("guard rail count mismatch: got=%d want=8", got)
	}
	if got := len(city.Obstacles); got != 18 {
		t.Fatalf("obstacle count mismatch: got=%d want=18", got)
	}
	if got := len(city.LanePaths); got != 6 {
		t.Fatalf("lane path count mismatch: got=%d want=6", got)
	}
}

func TestBuildCurvedOverpassCity_SpawnHitsDrivableGround(t *testing.T) {
	city := BuildCurvedOverpassCity()
	hit := city.Ground.Raycast(city.SpawnPosition, downDir(), fixed.FromInt(10))

	if !hit.Hit {
		t.Fatalf("expected spawn raycast to hit ground")
	}
	if hit.Point.Y != fixed.Zero {
		t.Fatalf("spawn ground Y mismatch: got=%v want=0", hit.Point.Y)
	}
}

func TestBuildCurvedOverpassCity_UnderpassStaysOnGroundLevel(t *testing.T) {
	city := BuildCurvedOverpassCity()
	hit := city.Ground.Raycast(geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero), downDir(), fixed.FromInt(10))

	if !hit.Hit {
		t.Fatalf("expected underpass raycast to hit")
	}
	if hit.Point.Y != fixed.Zero {
		t.Fatalf("underpass should stay on ground level: got=%v want=0", hit.Point.Y)
	}
}

func TestBuildCurvedOverpassCity_OverpassWinsWhenOriginIsAboveDeck(t *testing.T) {
	city := BuildCurvedOverpassCity()
	hit := city.Ground.Raycast(geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero), downDir(), fixed.FromInt(12))

	if !hit.Hit {
		t.Fatalf("expected overpass raycast to hit")
	}
	if hit.Point.Y != fixed.FromInt(6) {
		t.Fatalf("overpass height mismatch: got=%v want=6", hit.Point.Y)
	}
}

func TestCollideVehicleWithObstacles_PreventsEntryIntoBuilding(t *testing.T) {
	city := BuildCurvedOverpassCity()
	building := city.Buildings[0]
	center := building.Center()

	v := NewVehicle(1, geom.V3(center.X, fixed.FromInt(1), center.Z))
	CollideVehicleWithObstacles(&v, city.Obstacles)

	if vehicleOverlapsObstacle(v, building) {
		t.Fatalf("vehicle still overlaps building after collision resolution: pos=%v building=%+v", v.Position, building)
	}
}

func TestCityWorld_PerimeterContainsVehicle(t *testing.T) {
	world := NewCityWorld()
	world.Vehicle.Position = geom.V3(fixed.FromInt(79), fixed.FromInt(2), fixed.Zero)
	world.Vehicle.Velocity = geom.V3(fixed.FromInt(40), fixed.Zero, fixed.Zero)
	world.Step(fixed.FromFraction(1, 60))

	halfWidth := world.Vehicle.Tuning.TrackWidth.Div(fixed.FromInt(2))
	maxCenterX := world.Map.Bounds.MaxX.Sub(halfWidth)
	if world.Vehicle.Position.X.Cmp(maxCenterX) > 0 {
		t.Fatalf("vehicle escaped city perimeter: posX=%v limit=%v", world.Vehicle.Position.X, maxCenterX)
	}
}

func TestCollideVehicleWithObstacles_ContainsOverpassGuardRail(t *testing.T) {
	city := BuildCurvedOverpassCity()
	rail := city.GuardRails[0]
	center := rail.Center()

	v := NewVehicle(1, geom.V3(center.X, rail.BaseY.Add(fixed.FromFraction(3, 5)), center.Z))
	v.Velocity = geom.V3(fixed.FromInt(10), fixed.Zero, fixed.Zero)

	CollideVehicleWithObstacles(&v, city.Obstacles)

	if vehicleOverlapsObstacle(v, rail) {
		t.Fatalf("vehicle still overlaps guard rail after resolution: pos=%v rail=%+v", v.Position, rail)
	}
}

func TestCityWorld_SmokeDriveStaysGroundedAndOutsideObstacles(t *testing.T) {
	world := NewCityWorld()
	dt := fixed.FromFraction(1, 60)
	groundedTicks := 0
	everGrounded := false

	for tick := 0; tick < 420; tick++ {
		world.Vehicle.Input.Throttle = fixed.FromFraction(4, 5)
		world.Vehicle.Input.Brake = fixed.Zero
		world.Vehicle.Input.Steer = fixed.Zero
		if tick > 300 {
			world.Vehicle.Input.Throttle = fixed.Zero
			world.Vehicle.Input.Brake = fixed.One
		}

		world.Step(dt)

		if world.Vehicle.OnGround {
			groundedTicks++
			everGrounded = true
		}
		if tick > 90 && !world.Vehicle.OnGround {
			t.Fatalf("vehicle lost ground contact at tick=%d pos=%v vel=%v", tick, world.Vehicle.Position, world.Vehicle.Velocity)
		}
		if overlapsAnyObstacle(world.Vehicle, world.Map.Obstacles) {
			t.Fatalf("vehicle overlapped city obstacle at tick=%d pos=%v", tick, world.Vehicle.Position)
		}
		assertVehicleInsideBounds(t, world.Vehicle, world.Map.Bounds)
	}

	if !everGrounded {
		t.Fatalf("vehicle never settled onto the city ground")
	}
	if groundedTicks < 380 {
		t.Fatalf("vehicle grounded too infrequently: groundedTicks=%d", groundedTicks)
	}
}

func TestCityWorld_DrivingUnderBridgeStaysOnLowerRoad(t *testing.T) {
	world := NewCityWorld()
	world.Vehicle.Position = geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-28))
	world.Vehicle.Yaw = fixed.Zero
	world.Vehicle.UpdateBasisFromYaw()

	dt := fixed.FromFraction(1, 60)
	maxYUnderBridge := fixed.Zero
	seenUnderBridge := false

	for tick := 0; tick < 300; tick++ {
		world.Vehicle.Input.Brake = fixed.Zero
		world.Vehicle.Input.Steer = fixed.Zero
		if tick < 45 {
			world.Vehicle.Input.Throttle = fixed.Zero
		} else {
			world.Vehicle.Input.Throttle = fixed.FromFraction(7, 10)
		}

		world.Step(dt)

		if world.Vehicle.Position.Z.Cmp(fixed.FromInt(-8)) >= 0 && world.Vehicle.Position.Z.Cmp(fixed.FromInt(8)) <= 0 {
			seenUnderBridge = true
			if world.Vehicle.Position.Y.Cmp(maxYUnderBridge) > 0 {
				maxYUnderBridge = world.Vehicle.Position.Y
			}
		}
	}

	if !seenUnderBridge {
		t.Fatalf("vehicle never reached under-bridge section")
	}
	if maxYUnderBridge.Cmp(fixed.FromInt(2)) > 0 {
		t.Fatalf("vehicle was lifted toward overpass while on lower road: maxY=%v", maxYUnderBridge)
	}
}

func downDir() geom.Vec3 {
	return geom.V3(fixed.Zero, fixed.One.Neg(), fixed.Zero)
}

func overlapsAnyObstacle(v Vehicle, obstacles []CityObstacle) bool {
	for i := range obstacles {
		if vehicleOverlapsObstacle(v, obstacles[i]) {
			return true
		}
	}
	return false
}

func vehicleOverlapsObstacle(v Vehicle, obstacle CityObstacle) bool {
	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	halfHeight := fixed.FromFraction(1, 4)

	vehicleMinX := v.Position.X.Sub(halfWidth)
	vehicleMaxX := v.Position.X.Add(halfWidth)
	vehicleMinZ := v.Position.Z.Sub(halfLen)
	vehicleMaxZ := v.Position.Z.Add(halfLen)
	vehicleMinY := v.Position.Y.Sub(halfHeight)
	vehicleMaxY := v.Position.Y.Add(halfHeight)

	if vehicleMaxX.Cmp(obstacle.MinX) <= 0 || vehicleMinX.Cmp(obstacle.MaxX) >= 0 {
		return false
	}
	if vehicleMaxZ.Cmp(obstacle.MinZ) <= 0 || vehicleMinZ.Cmp(obstacle.MaxZ) >= 0 {
		return false
	}
	if vehicleMinY.Cmp(obstacle.TopY()) >= 0 || vehicleMaxY.Cmp(obstacle.BaseY) <= 0 {
		return false
	}

	return true
}

func assertVehicleInsideBounds(t *testing.T, v Vehicle, bounds WallBounds) {
	t.Helper()

	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))

	if v.Position.X.Cmp(bounds.MinX.Add(halfWidth)) < 0 {
		t.Fatalf("vehicle escaped minX bound: pos=%v min=%v", v.Position.X, bounds.MinX.Add(halfWidth))
	}
	if v.Position.X.Cmp(bounds.MaxX.Sub(halfWidth)) > 0 {
		t.Fatalf("vehicle escaped maxX bound: pos=%v max=%v", v.Position.X, bounds.MaxX.Sub(halfWidth))
	}
	if v.Position.Z.Cmp(bounds.MinZ.Add(halfLen)) < 0 {
		t.Fatalf("vehicle escaped minZ bound: pos=%v min=%v", v.Position.Z, bounds.MinZ.Add(halfLen))
	}
	if v.Position.Z.Cmp(bounds.MaxZ.Sub(halfLen)) > 0 {
		t.Fatalf("vehicle escaped maxZ bound: pos=%v max=%v", v.Position.Z, bounds.MaxZ.Sub(halfLen))
	}
}
