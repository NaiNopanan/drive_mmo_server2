package sim

import "testing"

func TestReplayIsDeterministic(t *testing.T) {
	inputs := []Input{
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
		{Throttle: 1, Left: 1},
	}

	a := NewWorld()
	b := NewWorld()

	for i := 0; i < 300; i++ {
		in := inputs[i%len(inputs)]
		Step(&a, in)
		Step(&b, in)
	}

	ha := HashWorld(a)
	hb := HashWorld(b)

	if ha != hb {
		t.Fatalf("hash mismatch: %x != %x", ha, hb)
	}
}
