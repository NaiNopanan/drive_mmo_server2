package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

func TestVehicleRestsOnFlatGround(t *testing.T) {
	vehicles := []Vehicle{
		NewDefaultVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero)),
	}
	w := NewVehicleWorld(GroundFlatSmall(), vehicles)

	for i := 0; i < 600; i++ {
		StepVehicleWorld(&w)
	}

	v := w.Vehicles[0]

	if !v.Body.OnGround {
		t.Fatalf("vehicle should be grounded")
	}

	contactCount := 0
	for i := range v.Wheels {
		if v.Wheels[i].Contact {
			contactCount++
		}
	}

	if contactCount < 2 {
		t.Fatalf("expected at least 2 wheel contacts, got %d", contactCount)
	}

	if v.Body.Pos.Y.Cmp(fixed.Zero) <= 0 {
		t.Fatalf("vehicle body Y should remain above ground, got %v", v.Body.Pos.Y)
	}
}

func TestVehicleAcceleratesForward(t *testing.T) {
	vehicles := []Vehicle{
		NewDefaultVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero)),
	}
	w := NewVehicleWorld(GroundFlatSmall(), vehicles)

	startZ := w.Vehicles[0].Body.Pos.Z

	for i := 0; i < 180; i++ {
		w.Vehicles[0].Input = VehicleInput{
			Throttle: fixed.One,
		}
		StepVehicleWorld(&w)
	}

	endZ := w.Vehicles[0].Body.Pos.Z
	if endZ.Cmp(startZ) <= 0 {
		t.Fatalf("vehicle did not move forward: start=%v end=%v", startZ, endZ)
	}
}

func TestVehicleSteeringChangesHeading(t *testing.T) {
	vehicles := []Vehicle{
		NewDefaultVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero)),
	}
	w := NewVehicleWorld(GroundFlatSmall(), vehicles)

	startForward := w.Vehicles[0].Body.Forward

	for i := 0; i < 180; i++ {
		w.Vehicles[0].Input = VehicleInput{
			Throttle: fixed.One,
			Steer:    fixed.FromFraction(1, 2), // 0.5
		}
		StepVehicleWorld(&w)
	}

	endForward := w.Vehicles[0].Body.Forward

	if endForward.X == startForward.X && endForward.Z == startForward.Z {
		t.Fatalf("vehicle heading did not change")
	}
}

func TestVehicleReplayDeterministicSingle(t *testing.T) {
	a := NewVehicleWorld(
		GroundSlopeSmall(),
		[]Vehicle{NewDefaultVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(4), fixed.Zero))},
	)
	b := NewVehicleWorld(
		GroundSlopeSmall(),
		[]Vehicle{NewDefaultVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(4), fixed.Zero))},
	)

	for i := 0; i < 1000; i++ {
		in := VehicleInput{}
		switch i % 5 {
		case 0:
			in.Throttle = fixed.One
		case 1:
			in.Throttle = fixed.One
			in.Steer = fixed.FromFraction(1, 3)
		case 2:
			in.Brake = fixed.FromFraction(1, 2)
		case 3:
			in.Steer = fixed.FromFraction(-1, 2)
		}

		a.Vehicles[0].Input = in
		b.Vehicles[0].Input = in

		StepVehicleWorld(&a)
		StepVehicleWorld(&b)
	}

	ha := HashVehicleWorld(a)
	hb := HashVehicleWorld(b)

	if ha != hb {
		t.Fatalf("hash mismatch: %d != %d", ha, hb)
	}
}

func TestVehicleReplayDeterministicMany(t *testing.T) {
	va := SpawnVehicleGrid(4, 4, 8, 4)
	vb := SpawnVehicleGrid(4, 4, 8, 4)

	a := NewVehicleWorld(GroundFlatSmall(), va)
	b := NewVehicleWorld(GroundFlatSmall(), vb)

	for tick := 0; tick < 600; tick++ {
		for i := range a.Vehicles {
			in := deterministicVehicleInput(tick, i)
			a.Vehicles[i].Input = in
			b.Vehicles[i].Input = in
		}

		StepVehicleWorld(&a)
		StepVehicleWorld(&b)
	}

	ha := HashVehicleWorld(a)
	hb := HashVehicleWorld(b)

	if ha != hb {
		t.Fatalf("many-vehicle hash mismatch: %d != %d", ha, hb)
	}
}

func deterministicVehicleInput(tick int, index int) VehicleInput {
	switch index % 4 {
	case 0:
		return VehicleInput{
			Throttle: fixed.One,
		}
	case 1:
		return VehicleInput{
			Throttle: fixed.FromFraction(3, 4),
			Steer:    fixed.FromFraction(1, 3),
		}
	case 2:
		return VehicleInput{
			Throttle: fixed.FromFraction(1, 2),
			Steer:    fixed.FromFraction(-1, 3),
		}
	default:
		if tick%60 < 30 {
			return VehicleInput{Brake: fixed.FromFraction(1, 2)}
		}
		return VehicleInput{Throttle: fixed.FromFraction(1, 2)}
	}
}
