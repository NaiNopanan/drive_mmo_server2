package world

import (
	"fmt"
	"strings"

	"server2/internal/body"
	"server2/internal/fixed3d"
)

type BodySnapshot struct {
	ID             uint64
	Position       fixed3d.Vec3
	LinearVelocity fixed3d.Vec3
}

type World struct {
	Tick uint64

	Config Config
	Bodies []*body.RigidBody

	initialState []BodySnapshot
}

func NewWorld(cfg Config) *World {
	return &World{
		Config: cfg,
		Bodies: make([]*body.RigidBody, 0),
	}
}

func (w *World) AddBody(b *body.RigidBody) {
	w.Bodies = append(w.Bodies, b)
}

func (w *World) SaveInitialState() {
	w.initialState = make([]BodySnapshot, len(w.Bodies))
	for i, b := range w.Bodies {
		w.initialState[i] = BodySnapshot{
			ID:             b.ID,
			Position:       b.Position,
			LinearVelocity: b.LinearVelocity,
		}
	}
}

func (w *World) Reset() {
	for i, s := range w.initialState {
		if i >= len(w.Bodies) {
			break
		}
		b := w.Bodies[i]
		if b.ID != s.ID {
			// Find body by ID if order changed (though it shouldn't in this version)
			continue 
		}
		b.Position = s.Position
		b.LinearVelocity = s.LinearVelocity
		b.ClearAccumulators()
	}
	w.Tick = 0
}

func (w *World) Step() {
	dt := w.Config.FixedDt

	for _, b := range w.Bodies {
		if b.Type != body.BodyDynamic {
			continue
		}

		// Apply gravity
		gravityForce := w.Config.Gravity.MulScalar(b.Mass)
		b.ApplyForce(gravityForce)

		// Simple Euler integration
		accel := b.ForceAccum.MulScalar(b.InvMass)
		b.LinearVelocity = b.LinearVelocity.Add(accel.MulScalar(dt))
		b.Position = b.Position.Add(b.LinearVelocity.MulScalar(dt))

		b.ClearAccumulators()
	}

	w.Tick++
}

func (w *World) Checksum() string {
	var sb strings.Builder

	sb.WriteString(fmt.Sprintf("tick=%d\n", w.Tick))
	for _, b := range w.Bodies {
		sb.WriteString(fmt.Sprintf(
			"id=%d|px=%d|py=%d|pz=%d|vx=%d|vy=%d|vz=%d\n",
			b.ID,
			b.Position.X.Raw(), b.Position.Y.Raw(), b.Position.Z.Raw(),
			b.LinearVelocity.X.Raw(), b.LinearVelocity.Y.Raw(), b.LinearVelocity.Z.Raw(),
		))
	}

	return sb.String()
}
