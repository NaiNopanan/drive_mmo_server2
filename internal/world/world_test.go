package world

import (
	"testing"

	"server2/internal/body"
	"server2/internal/fixed"
	"server2/internal/fixed3d"
)

func TestWorldStepGravityMovesDown(t *testing.T) {
	w := NewWorld(DefaultConfig())

	b := body.NewDynamicBody(
		1,
		fixed.FromInt(1),
		fixed3d.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
	)

	w.AddBody(b)
	w.Step()

	if b.Position.Y.Cmp(fixed.FromInt(10)) >= 0 {
		t.Fatalf("expected Y to decrease, got %s", b.Position.Y.String())
	}
}

func TestWorldDeterministicChecksum(t *testing.T) {
	run := func() string {
		w := NewWorld(DefaultConfig())

		b := body.NewDynamicBody(
			1,
			fixed.FromInt(1),
			fixed3d.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
		)

		w.AddBody(b)

		for i := 0; i < 300; i++ {
			w.Step()
		}

		return w.Checksum()
	}

	a := run()
	b := run()

	if a != b {
		t.Fatalf("checksum mismatch\nA:\n%s\nB:\n%s", a, b)
	}
}

func TestWorldReset(t *testing.T) {
	w := NewWorld(DefaultConfig())

	b := body.NewDynamicBody(
		1,
		fixed.FromInt(1),
		fixed3d.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
	)

	w.AddBody(b)
	w.SaveInitialState()

	for i := 0; i < 60; i++ {
		w.Step()
	}

	w.Reset()

	if b.Position != fixed3d.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero) {
		t.Fatalf("reset position mismatch: got=%v", b.Position)
	}
	if b.LinearVelocity != fixed3d.Zero() {
		t.Fatalf("reset velocity mismatch: got=%v", b.LinearVelocity)
	}
	if w.Tick != 0 {
		t.Fatalf("expected tick=0 got=%d", w.Tick)
	}
}
