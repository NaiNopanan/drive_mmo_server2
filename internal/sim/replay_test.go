package sim

import "testing"

func TestReplayIsDeterministic(t *testing.T) {
	inputs := []Input{
		{Throttle: 1},
		{Throttle: 1},
		{Throttle: 1, Right: 1},
		{Right: 1},
		{},
		{},
		{Left: 1},
		{Brake: 1},
		{},
		{Throttle: 1, Left: 1},
	}

	var a World
	var b World

	for i := 0; i < 5000; i++ {
		in := inputs[i%len(inputs)]
		Step(&a, in)
		Step(&b, in)
	}

	// Direct Comparison
	if a != b {
		t.Fatalf("world mismatch\na=%+v\nb=%+v", a, b)
	}

	// Hash Comparison
	ha := HashWorld(a)
	hb := HashWorld(b)

	if ha != hb {
		t.Fatalf("hash mismatch: %d != %d", ha, hb)
	}
}
