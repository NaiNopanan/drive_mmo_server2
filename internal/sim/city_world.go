package sim

import "server2/internal/fixed"

type CityWorld struct {
	Tick    uint64
	Map     CityMap
	Vehicle Vehicle
}

func NewCityWorld() CityWorld {
	city := BuildCurvedOverpassCity()
	v := NewVehicleWithTuning(1, city.SpawnPosition, EasyDriveTuning())
	v.Yaw = city.SpawnYaw
	v.UpdateBasisFromYaw()

	return CityWorld{
		Map:     city,
		Vehicle: v,
	}
}

func (w *CityWorld) Reset() {
	if w == nil {
		return
	}

	reset := NewCityWorld()
	*w = reset
}

func (w *CityWorld) Step(dt fixed.Fixed) {
	if w == nil {
		return
	}

	w.Vehicle.Step(dt, w.Map.Ground)
	CollideVehicleWithObstacles(&w.Vehicle, w.Map.Obstacles)
	CollideVehicleWithWalls(&w.Vehicle, w.Map.Bounds)
	w.Tick++
}
