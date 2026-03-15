package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

const DefaultStepRateHz = 60

var (
	DefaultTimeStep = fixed.FromFraction(1, DefaultStepRateHz)
	StandardGravity = geometry.NewVector3(fixed.Zero, fixed.FromFraction(-981, 100), fixed.Zero)
)

// MotionState stores the minimum state needed for linear force integration.
type MotionState struct {
	Position         geometry.Vector3
	Velocity         geometry.Vector3
	AccumulatedForce geometry.Vector3
	Mass             fixed.Fixed
	InverseMass      fixed.Fixed
}

// NewDynamicMotionState creates a state that responds to forces and impulses.
func NewDynamicMotionState(mass fixed.Fixed, position geometry.Vector3) MotionState {
	if mass.Cmp(fixed.Zero) <= 0 {
		panic("physics: dynamic mass must be > 0")
	}

	return MotionState{
		Position:    position,
		Mass:        mass,
		InverseMass: fixed.One.Div(mass),
	}
}

// NewStaticMotionState creates a state that ignores forces and impulses.
func NewStaticMotionState(position geometry.Vector3) MotionState {
	return MotionState{
		Position: position,
	}
}

// ComputeGravityForce returns the force produced by gravity for a given mass.
func ComputeGravityForce(mass fixed.Fixed, gravity geometry.Vector3) geometry.Vector3 {
	return gravity.Scale(mass)
}

func ApplyForce(s *MotionState, force geometry.Vector3) {
	if s == nil || s.InverseMass == fixed.Zero {
		return
	}

	s.AccumulatedForce = s.AccumulatedForce.Add(force)
}

func ApplyImpulse(s *MotionState, impulse geometry.Vector3) {
	if s == nil || s.InverseMass == fixed.Zero {
		return
	}

	s.Velocity = s.Velocity.Add(impulse.Scale(s.InverseMass))
}

func ClearAccumulatedForce(s *MotionState) {
	if s == nil {
		return
	}

	s.AccumulatedForce = geometry.ZeroVector3()
}

// StepLinearMotion advances the state by one fixed step using semi-implicit Euler integration.
func StepLinearMotion(s *MotionState, dt fixed.Fixed) {
	if s == nil || s.InverseMass == fixed.Zero {
		return
	}

	acceleration := s.AccumulatedForce.Scale(s.InverseMass)
	s.Velocity = s.Velocity.Add(acceleration.Scale(dt))
	s.Position = s.Position.Add(s.Velocity.Scale(dt))
	ClearAccumulatedForce(s)
}

// StepLinearMotionWithGravity applies gravity and then advances the state by one fixed step.
func StepLinearMotionWithGravity(s *MotionState, dt fixed.Fixed, gravity geometry.Vector3) {
	if s == nil || s.InverseMass == fixed.Zero {
		return
	}

	ApplyForce(s, ComputeGravityForce(s.Mass, gravity))
	StepLinearMotion(s, dt)
}
