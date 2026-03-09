package sim

import (
	"encoding/binary"
	"hash/fnv"

	"server2/internal/fixed"
	"server2/internal/geom"
)

const TickHz = 60

var (
	Dt       = fixed.FromFraction(1, TickHz)
	Accel    = fixed.FromInt(12) // 12 m/s^2
	Brake    = fixed.FromInt(20) // 20 m/s^2
	Drag     = fixed.FromInt(4)  // 4 m/s^2
	MaxSpeed = fixed.FromInt(40) // 40 m/s
)

type Input struct {
	Throttle int8
	Brake    int8
	Left     int8
	Right    int8
}

type Body struct {
	Pos geom.Vec3
	Vel geom.Vec3
}

type World struct {
	Tick uint64
	Car  Body
}

func Step(w *World, in Input) {
	var ax fixed.Fixed
	var az fixed.Fixed

	if in.Right != 0 {
		ax = ax.Add(Accel)
	}
	if in.Left != 0 {
		ax = ax.Sub(Accel)
	}
	if in.Throttle != 0 {
		az = az.Add(Accel)
	}
	if in.Brake != 0 {
		az = az.Sub(Brake)
	}

	w.Car.Vel.X = w.Car.Vel.X.Add(ax.Mul(Dt))
	w.Car.Vel.Z = w.Car.Vel.Z.Add(az.Mul(Dt))

	dragStep := Drag.Mul(Dt)

	if in.Left == 0 && in.Right == 0 {
		w.Car.Vel.X = approachZero(w.Car.Vel.X, dragStep)
	}
	if in.Throttle == 0 && in.Brake == 0 {
		w.Car.Vel.Z = approachZero(w.Car.Vel.Z, dragStep)
	}

	w.Car.Vel.X = clampSymmetric(w.Car.Vel.X, MaxSpeed)
	w.Car.Vel.Z = clampSymmetric(w.Car.Vel.Z, MaxSpeed)

	w.Car.Pos = w.Car.Pos.Add(w.Car.Vel.Scale(Dt))
	w.Tick++
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

	writeU64(w.Tick)

	writeI64(w.Car.Pos.X.Raw())
	writeI64(w.Car.Pos.Y.Raw())
	writeI64(w.Car.Pos.Z.Raw())

	writeI64(w.Car.Vel.X.Raw())
	writeI64(w.Car.Vel.Y.Raw())
	writeI64(w.Car.Vel.Z.Raw())

	return h.Sum64()
}
