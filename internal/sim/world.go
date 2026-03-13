package sim

import (
	"encoding/binary"
	"hash/fnv"

	"server2/internal/fixed"
	"server2/internal/geom"
)

const TickHz = 60

var (
	Dt = fixed.FromFraction(1, TickHz)

	// gravity = -9.8
	GravityY = fixed.FromFraction(-98, 10)

	// horizontal control
	MoveAccel = fixed.FromInt(12)
	MaxSpeed  = fixed.FromInt(40)

	// drag
	AirDrag    = fixed.FromFraction(1, 2) // 0.5
	GroundDrag = fixed.FromInt(10)

	// infinite plane at y = 0
	GroundY = fixed.Zero
)

type Input struct {
	Throttle int8
	Brake    int8
	Left     int8
	Right    int8
}

type Body struct {
	Pos      geom.Vec3
	Vel      geom.Vec3
	Radius   fixed.Fixed
	OnGround bool
}

type World struct {
	Tick uint64
	Car  Body
}

func NewWorld() World {
	return World{
		Car: Body{
			Pos:      geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
			Vel:      geom.Zero(),
			Radius:   fixed.FromInt(1),
			OnGround: false,
		},
	}
}

func Step(w *World, in Input) {
	b := &w.Car

	ax, az := inputAcceleration(in)

	// acceleration for this frame
	acc := geom.V3(
		ax,
		GravityY,
		az,
	)

	// semi-implicit Euler:
	// 1. update velocity: vel = vel + acc * dt
	b.Vel = b.Vel.Add(acc.Scale(Dt))

	// drag on X/Z only
	applyHorizontalDrag(b, in)

	// clamp X/Z speed
	b.Vel.X = clampSymmetric(b.Vel.X, MaxSpeed)
	b.Vel.Z = clampSymmetric(b.Vel.Z, MaxSpeed)

	// 2. update position: pos = pos + vel * dt
	b.Pos = b.Pos.Add(b.Vel.Scale(Dt))

	// resolve collision against infinite ground plane y=0
	resolveGroundPlane(b, GroundY)

	w.Tick++
}

func inputAcceleration(in Input) (fixed.Fixed, fixed.Fixed) {
	ax := fixed.Zero
	az := fixed.Zero

	if in.Right != 0 {
		ax = ax.Add(MoveAccel)
	}
	if in.Left != 0 {
		ax = ax.Sub(MoveAccel)
	}
	if in.Throttle != 0 {
		az = az.Add(MoveAccel)
	}
	if in.Brake != 0 {
		az = az.Sub(MoveAccel)
	}

	return ax, az
}

func applyHorizontalDrag(b *Body, in Input) {
	dragPerTick := AirDrag.Mul(Dt)
	if b.OnGround {
		dragPerTick = GroundDrag.Mul(Dt)
	}

	if in.Left == 0 && in.Right == 0 {
		b.Vel.X = approachZero(b.Vel.X, dragPerTick)
	}
	if in.Throttle == 0 && in.Brake == 0 {
		b.Vel.Z = approachZero(b.Vel.Z, dragPerTick)
	}
}

func resolveGroundPlane(b *Body, planeY fixed.Fixed) {
	contactY := planeY.Add(b.Radius)

	if b.Pos.Y.Cmp(contactY) < 0 {
		// positional correction (no penetration)
		b.Pos.Y = contactY

		// kill downward velocity only
		if b.Vel.Y.Cmp(fixed.Zero) < 0 {
			b.Vel.Y = fixed.Zero
		}

		b.OnGround = true
		return
	}

	if b.Pos.Y == contactY && b.Vel.Y == fixed.Zero {
		b.OnGround = true
		return
	}

	b.OnGround = false
}

func clampSymmetric(v, limit fixed.Fixed) fixed.Fixed {
	if v.Cmp(limit) > 0 {
		return limit
	}
	n := limit.Neg()
	if v.Cmp(n) < 0 {
		return n
	}
	return v
}

func approachZero(v, delta fixed.Fixed) fixed.Fixed {
	if v.Cmp(fixed.Zero) > 0 {
		nv := v.Sub(delta)
		if nv.Cmp(fixed.Zero) < 0 {
			return fixed.Zero
		}
		return nv
	}

	if v.Cmp(fixed.Zero) < 0 {
		nv := v.Add(delta)
		if nv.Cmp(fixed.Zero) > 0 {
			return fixed.Zero
		}
		return nv
	}

	return fixed.Zero
}

func HashWorld(w World) uint64 {
	h := fnv.New64a()
	buf := make([]byte, 8)

	writeI64 := func(v int64) {
		binary.LittleEndian.PutUint64(buf, uint64(v))
		_, _ = h.Write(buf)
	}
	writeU64 := func(v uint64) {
		binary.LittleEndian.PutUint64(buf, v)
		_, _ = h.Write(buf)
	}

	writeBool := func(v bool) {
		if v {
			writeU64(1)
		} else {
			writeU64(0)
		}
	}

	writeU64(w.Tick)

	writeI64(w.Car.Pos.X.Raw())
	writeI64(w.Car.Pos.Y.Raw())
	writeI64(w.Car.Pos.Z.Raw())

	writeI64(w.Car.Vel.X.Raw())
	writeI64(w.Car.Vel.Y.Raw())
	writeI64(w.Car.Vel.Z.Raw())

	writeI64(w.Car.Radius.Raw())
	writeBool(w.Car.OnGround)

	return h.Sum64()
}
