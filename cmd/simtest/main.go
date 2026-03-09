package main

import (
	"fmt"

	"server2/internal/sim"
)

func main() {
	println("--- server2-perfect Day 2 SimTest Runner ---")
	
	var w sim.World

	sequence := []sim.Input{
		{Throttle: 1},
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

	for i := 0; i < 600; i++ {
		in := sequence[i%len(sequence)]
		sim.Step(&w, in)
	}

	fmt.Printf("Final Simulation State (Tick %d):\n", w.Tick)
	fmt.Println("  Position :", w.Car.Pos)
	fmt.Println("  Velocity :", w.Car.Vel)
	fmt.Printf("  Final Hash: %d\n", sim.HashWorld(w))
}
