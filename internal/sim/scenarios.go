package sim

import (
	"fmt"
	"server2/internal/fixed"
	"server2/internal/geom"
)

type ScenarioStatus int

const (
	ScenarioRunning ScenarioStatus = iota
	ScenarioPassed
	ScenarioFailed
)

type ScenarioResult struct {
	Status  ScenarioStatus
	Message string
}

func Pass(msg string) ScenarioResult {
	return ScenarioResult{Status: ScenarioPassed, Message: msg}
}

func Fail(msg string) ScenarioResult {
	return ScenarioResult{Status: ScenarioFailed, Message: msg}
}

func Running(msg string) ScenarioResult {
	return ScenarioResult{Status: ScenarioRunning, Message: msg}
}

type Scenario struct {
	Name        string
	Description string
	MaxTicks    int
	Setup       func() World
	InputAtTick func(tick int, w World) Input
	Check       func(w World) ScenarioResult
}

type ScenarioRunner struct {
	Scenario   Scenario
	World      World
	Tick       int
	Finished   bool
	LastResult ScenarioResult
}

func NewScenarioRunner(s Scenario) *ScenarioRunner {
	return &ScenarioRunner{
		Scenario:   s,
		World:      s.Setup(),
		Tick:       0,
		Finished:   false,
		LastResult: Running("Initializing"),
	}
}

func (r *ScenarioRunner) Reset() {
	r.World = r.Scenario.Setup()
	r.Tick = 0
	r.Finished = false
	r.LastResult = Running("Reset")
}

func (r *ScenarioRunner) Step() {
	if r.Finished {
		return
	}

	if r.Tick < r.Scenario.MaxTicks {
		in := r.Scenario.InputAtTick(r.Tick, r.World)
		Step(&r.World, in)
		r.Tick++
		r.LastResult = Running(fmt.Sprintf("Step %d/%d", r.Tick, r.Scenario.MaxTicks))
		
		// Optional: Early check if desired
		return
	}

	r.LastResult = r.Scenario.Check(r.World)
	r.Finished = true
}

func DefaultScenarios() []Scenario {
	return []Scenario{
		ScenarioFreeFall(),
		ScenarioRestOnFlatGround(),
		ScenarioMoveOnFlatGround(),
		ScenarioFallOntoSlope(),
		ScenarioSlideDownSlope(),
		ScenarioDeterminismReplay(),
	}
}

func ScenarioFreeFall() Scenario {
	return Scenario{
		Name:        "S01: Free Fall",
		Description: "Body falls from Y=10. Verifies gravity and Euler integration.",
		MaxTicks:    120,
		Setup: func() World {
			w := NewWorld()
			w.GroundTriangles = nil // No ground
			w.Car.Pos = geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero)
			return w
		},
		InputAtTick: func(tick int, w World) Input { return Input{} },
		Check: func(w World) ScenarioResult {
			if w.Car.Pos.Y.Cmp(fixed.FromInt(10)) >= 0 {
				return Fail("Body did not fall")
			}
			if w.Car.Vel.Y.Cmp(fixed.Zero) >= 0 {
				return Fail("Velocity not negative")
			}
			return Pass("Falling correctly")
		},
	}
}

func ScenarioRestOnFlatGround() Scenario {
	return Scenario{
		Name:        "S02: Flat Ground Rest",
		Description: "Body hits ground and stays at Y=Radius, Vel.Y=0.",
		MaxTicks:    300,
		Setup: func() World {
			return NewWorld()
		},
		InputAtTick: func(tick int, w World) Input { return Input{} },
		Check: func(w World) ScenarioResult {
			if !w.Car.OnGround {
				return Fail("Not grounded")
			}
			diffY := w.Car.Pos.Y.Sub(w.Car.Radius).Abs()
			if diffY.Cmp(fixed.FromRaw(1024)) > 0 {
				return Fail(fmt.Sprintf("Pos.Y mismatch: %v", w.Car.Pos.Y))
			}
			if w.Car.Vel.Y != fixed.Zero {
				return Fail("Vel.Y not zeroed")
			}
			return Pass("Rests perfectly")
		},
	}
}

func ScenarioMoveOnFlatGround() Scenario {
	return Scenario{
		Name:        "S03: Flat Ground Move",
		Description: "Move while grounded. Verifies horizontal friction and stability.",
		MaxTicks:    200,
		Setup: func() World {
			w := NewWorld()
			// Let body start at Y=10, fall and settle onto flat ground naturally.
			// Ground is ±50 (default), body won't escape in 200 ticks at MaxSpeed=40.
			return w
		},
		InputAtTick: func(tick int, w World) Input {
			// First 120 ticks: no input, let body settle on ground.
			// Next 80 ticks: apply throttle.
			if tick < 120 {
				return Input{}
			}
			return Input{Throttle: 1}
		},
		Check: func(w World) ScenarioResult {
			if w.Car.Pos.Z == fixed.Zero {
				return Fail("No movement detected")
			}
			if !w.Car.OnGround {
				return Fail("Lost grounding during movement")
			}
			return Pass("Movement stable")
		},
	}
}

func ScenarioFallOntoSlope() Scenario {
	return Scenario{
		Name:        "S04: Falling onto Slope",
		Description: "Verify collision with sloped triangles.",
		MaxTicks:    240,
		Setup: func() World {
			w := NewWorld()
			w.Car.Pos = geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero)
			w.GroundTriangles = makeSlopeTriangles()
			return w
		},
		InputAtTick: func(tick int, w World) Input { return Input{} },
		Check: func(w World) ScenarioResult {
			if !w.Car.OnGround {
				return Fail("Not grounded on slope")
			}
			// Normal should have horizontal component
			if w.LastContact.Normal.X == fixed.Zero && w.LastContact.Normal.Z == fixed.Zero {
				return Fail("Normal is perfectly vertical, expected slope")
			}
			return Pass("Slope contact detected")
		},
	}
}

func ScenarioSlideDownSlope() Scenario {
	return Scenario{
		Name:        "S05: Slide Down Slope",
		Description: "Gravity should pull body down sloped surface.",
		MaxTicks:    400,
		Setup: func() World {
			w := NewWorld()
			w.Car.Pos = geom.V3(fixed.Zero, fixed.FromInt(5), fixed.Zero)
			w.GroundTriangles = makeSlopeTriangles()
			return w
		},
		InputAtTick: func(tick int, w World) Input { return Input{} },
		Check: func(w World) ScenarioResult {
			if !w.Car.OnGround {
				return Fail("Lost contact while sliding")
			}
			// In our makeSlopeTriangles, downhill should be -Z
			if w.Car.Pos.Z.Cmp(fixed.Zero) >= 0 {
				return Fail("Body did not slide downhill (-Z)")
			}
			return Pass("Sliding correctly")
		},
	}
}

func ScenarioDeterminismReplay() Scenario {
	return Scenario{
		Name:        "S06: Determinism Replay",
		Description: "Runs two simulations with same inputs. Hashes must match.",
		MaxTicks:    500,
		Setup: func() World {
			return NewWorld()
		},
		InputAtTick: func(tick int, w World) Input {
			// Some varied inputs
			inputs := []Input{{}, {Throttle: 1}, {Right: 1}, {Throttle: 1, Left: 1}, {Brake: 1}}
			return inputs[tick%len(inputs)]
		},
		Check: func(w World) ScenarioResult {
			// We simulate a second one here to compare
			w2 := NewWorld()
			for t := 0; t < 500; t++ {
				in := []Input{{}, {Throttle: 1}, {Right: 1}, {Throttle: 1, Left: 1}, {Brake: 1}}[t%5]
				Step(&w2, in)
			}
			h1 := HashWorld(w)
			h2 := HashWorld(w2)
			if h1 != h2 {
				return Fail(fmt.Sprintf("Hash mismatch: %x vs %x", h1, h2))
			}
			return Pass(fmt.Sprintf("Bit-perfect match (Hash: %x)", h1))
		},
	}
}

// makeSlopeTriangles creates slope geometry safe for Q32 fixed-point.
// ±40 keeps dot products (edge ~80 * vec ~10+) well within Q32 limit (~2B).
func makeSlopeTriangles() []geom.Triangle {
	min := fixed.FromInt(-40)
	max := fixed.FromInt(40)
	yLow := fixed.Zero
	yHigh := fixed.FromInt(8) // gentle slope: 8/80 rise/run

	a := geom.V3(min, yLow, min)
	b := geom.V3(max, yLow, min)
	c := geom.V3(min, yHigh, max)
	d := geom.V3(max, yHigh, max)

	return []geom.Triangle{
		geom.NewTriangle(a, b, c),
		geom.NewTriangle(c, b, d),
	}
}
