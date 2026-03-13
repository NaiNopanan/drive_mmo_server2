package sim

import (
	"testing"

	"server2/internal/fixed"
)

func TestBodyFallsToPlaneAndRests(t *testing.T) {
	w := NewWorld()

	// Initial state: Y=10, Vel.Y=0
	for i := 0; i < 300; i++ {
		Step(&w, Input{})
	}

	// Body should rest at Radius (1.0)
	if got, want := w.Car.Pos.Y, w.Car.Radius; got != want {
		t.Fatalf("final Y mismatch: got=%v want=%v", got, want)
	}

	// Vertical velocity should be zeroed
	if w.Car.Vel.Y != fixed.Zero {
		t.Fatalf("final vel.Y should be zero, got=%v", w.Car.Vel.Y)
	}

	// Should be marked as on ground
	if !w.Car.OnGround {
		t.Fatalf("body should be on ground")
	}
}

func TestBodyNeverGoesBelowPlaneAfterResolution(t *testing.T) {
	w := NewWorld()

	for i := 0; i < 600; i++ {
		Step(&w, Input{})

		if w.Car.Pos.Y.Cmp(w.Car.Radius) < 0 {
			t.Fatalf("body penetrated plane at tick=%d posY=%v radius=%v", w.Tick, w.Car.Pos.Y, w.Car.Radius)
		}
	}
}

func TestHorizontalMoveWhileGravityRuns(t *testing.T) {
	w := NewWorld()

	// Run long enough to hit ground and keep moving
	for i := 0; i < 180; i++ {
		Step(&w, Input{Throttle: 1, Right: 1})
	}

	if w.Car.Pos.X == fixed.Zero {
		t.Fatalf("expected X movement")
	}
	if w.Car.Pos.Z == fixed.Zero {
		t.Fatalf("expected Z movement")
	}
	
	// Check if still on ground as expected
	if !w.Car.OnGround {
		t.Fatalf("expected to be on ground during movement")
	}
}
