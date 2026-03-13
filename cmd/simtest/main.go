package main

import (
	"fmt"
	"server2/internal/sim"
)

func main() {
	println("--- server2-perfect Day 3 SimTest Runner ---")
	
	w := sim.NewWorld()

	sequence := []sim.Input{
		{},
		{},
		{Throttle: 1},
		{Throttle: 1},
		{Throttle: 1, Right: 1},
		{Right: 1},
		{},
		{},
		{Left: 1},
		{Brake: 1},
		{},
	}

	for i := 0; i < 240; i++ {
		in := sequence[i%len(sequence)]
		sim.Step(&w, in)

		if i%30 == 0 {
			fmt.Printf(
				"tick=%3d pos=%v vel=%v onGround=%v\n",
				w.Tick,
				w.Car.Pos,
				w.Car.Vel,
				w.Car.OnGround,
			)
		}
	}

	fmt.Println("---- final ----")
	fmt.Println("tick    :", w.Tick)
	fmt.Println("pos     :", w.Car.Pos)
	fmt.Println("vel     :", w.Car.Vel)
	fmt.Println("onGround:", w.Car.OnGround)
	fmt.Println("hash    :", sim.HashWorld(w))
}
