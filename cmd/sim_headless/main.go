package main

import (
	"fmt"

	"server2/internal/body"
	"server2/internal/fixed"
	"server2/internal/geom"
	"server2/internal/world"
)

func main() {
	println("--- server2-perfect Day 1 Headless Simulation ---")
	
	w := world.NewWorld(world.DefaultConfig())

	b := body.NewDynamicBody(
		1,
		fixed.FromInt(1),
		geom.V3(
			fixed.Zero,
			fixed.FromInt(10),
			fixed.Zero,
		),
	)

	w.AddBody(b)
	w.SaveInitialState()

	// Capture first run
	fmt.Printf("Initial Checksum: %q\n", w.Checksum())
	
	var lastChecksum string
	for i := 0; i < 10; i++ {
		w.Step()
		lastChecksum = w.Checksum()
		fmt.Printf(
			"tick=%d pos=%s vel=%s checksum=%q\n",
			w.Tick,
			b.Position.String(),
			b.LinearVelocity.String(),
			lastChecksum,
		)
	}

	fmt.Printf("\nFinal Checksum (Run 1): %q\n", lastChecksum)

	// Reset and run again for verification
	println("\n--- Resetting for Run 2 Verification ---")
	w.Reset()
	
	for i := 0; i < 10; i++ {
		w.Step()
	}
	
	verifyChecksum := w.Checksum()
	fmt.Printf("Final Checksum (Run 2): %q\n", verifyChecksum)

	if lastChecksum == verifyChecksum {
		println("\nSUCCESS: Bit-perfect determinism verified across resets!")
	} else {
		println("\nFAILURE: Determinism mismatch between runs.")
	}
}
