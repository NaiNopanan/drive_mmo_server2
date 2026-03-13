package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

// makeTestSlopeTriangles creates a slope within Q32-safe range.
// ±40 keeps dot products (edge*vec) well within Q32 max.
func makeTestSlopeTriangles() []geom.Triangle {
	min := fixed.FromInt(-40)
	max := fixed.FromInt(40)
	yLow := fixed.Zero
	yHigh := fixed.FromInt(8) // slope: 8/80 rise/run ratio

	a := geom.V3(min, yLow, min)
	b := geom.V3(max, yLow, min)
	c := geom.V3(min, yHigh, max)
	d := geom.V3(max, yHigh, max)

	return []geom.Triangle{
		geom.NewTriangle(a, b, c),
		geom.NewTriangle(c, b, d),
	}
}

func TestBodyFallsOntoSlopeAndGetsGrounded(t *testing.T) {
	w := NewWorld()
	w.GroundTriangles = makeTestSlopeTriangles()
	w.Car.Pos = geom.V3(fixed.Zero, fixed.FromInt(12), fixed.Zero)

	for i := 0; i < 300; i++ {
		Step(&w, Input{})
	}

	if !w.Car.OnGround {
		t.Fatalf("expected body to be grounded on slope")
	}

	if !w.LastContact.HasContact {
		t.Fatalf("expected contact debug to be set")
	}

	if w.LastContact.Normal.Y.Cmp(GroundNormalMinY) < 0 {
		t.Fatalf("expected walkable contact normal, got %v", w.LastContact.Normal)
	}
}

func TestBodySlidesDownSlopeWithoutPenetration(t *testing.T) {
	oldGroundDrag := GroundDrag
	GroundDrag = fixed.Zero
	defer func() { GroundDrag = oldGroundDrag }()

	w := NewWorld()
	w.GroundTriangles = makeTestSlopeTriangles()
	w.Car.Pos = geom.V3(fixed.Zero, fixed.FromInt(12), fixed.FromInt(5))

	startZ := w.Car.Pos.Z

	// Run 120 ticks (2 seconds) — enough to slide but not off the ±40 slope
	for i := 0; i < 120; i++ {
		Step(&w, Input{})
	}

	// Body should have moved downhill (negative Z because slope rises from -Z to +Z)
	if w.Car.Pos.Z.Cmp(startZ) >= 0 {
		t.Fatalf("expected body to slide downhill (-Z direction), start=%v end=%v", startZ, w.Car.Pos.Z)
	}

	if !w.Car.OnGround {
		t.Fatalf("expected body to remain grounded while sliding. OnGround=%v, Pos=%v, Radius=%v",
			w.Car.OnGround, w.Car.Pos, w.Car.Radius)
	}
}
