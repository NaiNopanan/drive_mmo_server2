package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func TestVehicleRestsOnFlatGround(t *testing.T) {
	vehicles := []Vehicle{
		NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero)),
	}
	w := VehicleWorldDay6{
		Ground:   WorldGroundQuery{Triangles: GroundFlatSmall()},
		Vehicles:        vehicles,
	}

	dt := fixed.FromFraction(1, 60)
	for i := 0; i < 600; i++ {
		w.Step(dt)
	}

	v := w.Vehicles[0]

	if !v.OnGround {
		t.Fatalf("vehicle should be grounded")
	}

	if v.GroundedWheels < 2 {
		t.Fatalf("expected at least 2 wheel contacts, got %d", v.GroundedWheels)
	}

	if v.Position.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("vehicle body Y should remain above ground, got %v", v.Position.Y)
	}
}

func TestVehicleAcceleratesForward(t *testing.T) {
	vehicles := []Vehicle{
		NewVehicle(1, geom.V3(fixed.Zero, fixed.FromFraction(12, 10), fixed.Zero)),
	}
	w := VehicleWorldDay6{
		Ground:   WorldGroundQuery{Triangles: GroundFlatSmall()},
		Vehicles:        vehicles,
	}

	startZ := w.Vehicles[0].Position.Z
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 180; i++ {
		w.Vehicles[0].Input = VehicleInput{
			Throttle: fixed.One,
		}
		w.Step(dt)
	}

	endZ := w.Vehicles[0].Position.Z
	if endZ.Cmp(startZ) <= 0 {
		t.Fatalf("vehicle did not move forward: start=%v end=%v", startZ, endZ)
	}
}

func TestVehicleSteeringChangesYaw(t *testing.T) {
	vehicles := []Vehicle{
		NewVehicle(1, geom.V3(fixed.Zero, fixed.FromFraction(12, 10), fixed.Zero)),
	}
	w := VehicleWorldDay6{
		Ground:   WorldGroundQuery{Triangles: GroundFlatSmall()},
		Vehicles:        vehicles,
	}

	startYaw := w.Vehicles[0].Yaw
	dt := fixed.FromFraction(1, 60)

	for i := 0; i < 180; i++ {
		w.Vehicles[0].Input = VehicleInput{
			Throttle: fixed.One,
			Steer:    fixed.FromFraction(1, 2), // 0.5
		}
		w.Step(dt)
	}

	endYaw := w.Vehicles[0].Yaw

	if endYaw.Cmp(startYaw) == 0 {
		t.Fatalf("vehicle yaw did not change: start=%v end=%v", startYaw, endYaw)
	}
}

func TestVehicleReplayDeterministicMany(t *testing.T) {
	va := spawnGrid(4, 4)
	vb := spawnGrid(4, 4)

	a := VehicleWorldDay6{Ground: WorldGroundQuery{Triangles: GroundFlatSmall()}, Vehicles: va}
	b := VehicleWorldDay6{Ground: WorldGroundQuery{Triangles: GroundFlatSmall()}, Vehicles: vb}

	dt := fixed.FromFraction(1, 60)
	for tick := 0; tick < 300; tick++ {
		for i := range a.Vehicles {
			in := deterministicVehicleInput(tick, i)
			a.Vehicles[i].Input = in
			b.Vehicles[i].Input = in
		}

		a.Step(dt)
		b.Step(dt)
	}

	for i := range a.Vehicles {
		ha := HashVehicle(a.Vehicles[i])
		hb := HashVehicle(b.Vehicles[i])
		if ha != hb {
			t.Fatalf("vehicle %d hash mismatch: %d != %d", i, ha, hb)
		}
	}
}

func spawnGrid(rows, cols int) []Vehicle {
	var vs []Vehicle
	id := uint32(1)
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			vs = append(vs, NewVehicle(id, geom.V3(
				fixed.FromInt(int64(c*4)),
				fixed.FromInt(3),
				fixed.FromInt(int64(r*6)),
			)))
			id++
		}
	}
	return vs
}

func deterministicVehicleInput(tick int, index int) VehicleInput {
	switch index % 4 {
	case 0:
		return VehicleInput{Throttle: fixed.One}
	case 1:
		return VehicleInput{Throttle: fixed.FromFraction(3, 4), Steer: fixed.FromFraction(1, 3)}
	case 2:
		return VehicleInput{Throttle: fixed.FromFraction(1, 2), Steer: fixed.FromFraction(-1, 3)}
	default:
		if tick%60 < 30 {
			return VehicleInput{Brake: fixed.FromFraction(1, 2)}
		}
		return VehicleInput{Throttle: fixed.FromFraction(1, 2)}
	}
}
