package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// makeTestFlatGround makes a larger flat ground for tests that need more movement room.
// ±48 keeps dot products within Q32 safe range (max ~18k << Q32 limit ~2B).
func makeTestFlatGround() []geom.Triangle {
	min := fixed.FromInt(-48)
	max := fixed.FromInt(48)
	y := fixed.Zero
	a := geom.V3(min, y, min)
	b := geom.V3(max, y, min)
	c := geom.V3(min, y, max)
	d := geom.V3(max, y, max)
	return []geom.Triangle{
		geom.NewTriangle(a, c, b),
		geom.NewTriangle(c, d, b),
	}
}

func TestBodyFallsToPlaneAndRests(t *testing.T) {
	w := NewWorld()

	for i := 0; i < 300; i++ {
		Step(&w, Input{})
	}

	diff := w.Car.Pos.Y.Sub(w.Car.Radius).Abs()
	if diff.Cmp(1024) > 0 {
		t.Fatalf("final Y mismatch: got=%v want=%v diff_raw=%d", w.Car.Pos.Y, w.Car.Radius, diff.Raw())
	}

	if w.Car.Vel.Y != fixed.Zero {
		t.Fatalf("final vel.Y should be zero, got=%v", w.Car.Vel.Y)
	}

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
	// Use a larger local ground so body doesn't escape during movement.
	w.GroundTriangles = makeTestFlatGround()

	// Phase 1: let body fall from Y=10 and settle onto ground (~120 ticks).
	for i := 0; i < 120; i++ {
		Step(&w, Input{})
	}

	if !w.Car.OnGround {
		t.Fatalf("body should be grounded before movement test. OnGround=%v Y=%v", w.Car.OnGround, w.Car.Pos.Y)
	}

	startX := w.Car.Pos.X
	startZ := w.Car.Pos.Z

	// Phase 2: apply movement for 60 ticks (1 second), stay on ±48 grid.
	for i := 0; i < 60; i++ {
		Step(&w, Input{Throttle: 1, Right: 1})
	}

	if w.Car.Pos.X == startX {
		t.Fatalf("expected X movement")
	}
	if w.Car.Pos.Z == startZ {
		t.Fatalf("expected Z movement")
	}

	if !w.Car.OnGround {
		t.Fatalf("expected body to remain on ground during movement. OnGround=%v, Y=%v, Radius=%v",
			w.Car.OnGround, w.Car.Pos.Y, w.Car.Radius)
	}
}
