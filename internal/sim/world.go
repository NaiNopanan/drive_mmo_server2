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

	// walkable slope threshold
	GroundNormalMinY = fixed.FromFraction(1, 2)
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

type ContactDebug struct {
	HasContact bool
	Point      geom.Vec3
	Normal     geom.Vec3
}

type World struct {
	Tick uint64
	Car  Body

	GroundTriangles []geom.Triangle
	LastContact     ContactDebug
}

func NewWorld() World {
	return World{
		Car: Body{
			Pos:      geom.V3(fixed.Zero, fixed.FromInt(10), fixed.Zero),
			Vel:      geom.Zero(),
			Radius:   fixed.FromInt(1),
			OnGround: false,
		},
		GroundTriangles: makeFlatGroundTriangles(),
	}
}

func makeFlatGroundTriangles() []geom.Triangle {
	// ±50 keeps dot-product values well within Q32 safe range.
	// Max dot product: 100 * 100 = 10,000, which is << Q32 max ~2B.
	min := fixed.FromInt(-50)
	max := fixed.FromInt(50)
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

func Step(w *World, in Input) {
	if w == nil {
		return
	}
	b := &w.Car
	wasOnGround := b.OnGround

	b.OnGround = false
	w.LastContact = ContactDebug{}

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
	applyHorizontalDrag(b, in, wasOnGround)

	// clamp X/Z speed
	b.Vel.X = clampSymmetric(b.Vel.X, MaxSpeed)
	b.Vel.Z = clampSymmetric(b.Vel.Z, MaxSpeed)

	// 2. update position: pos = pos + vel * dt
	b.Pos = b.Pos.Add(b.Vel.Scale(Dt))

	// resolve collision against ground triangles
	resolveGroundTriangles(w, b)

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

func applyHorizontalDrag(b *Body, in Input, wasOnGround bool) {
	dragPerTick := AirDrag.Mul(Dt)
	if wasOnGround {
		dragPerTick = GroundDrag.Mul(Dt)
	}

	if in.Left == 0 && in.Right == 0 {
		b.Vel.X = approachZero(b.Vel.X, dragPerTick)
	}
	if in.Throttle == 0 && in.Brake == 0 {
		b.Vel.Z = approachZero(b.Vel.Z, dragPerTick)
	}
}

func resolveGroundTriangles(w *World, b *Body) {
	if w == nil || b == nil {
		return
	}
	tris := w.GroundTriangles
	if len(tris) == 0 {
		return
	}

	const maxResolveIters = 4
	const groundingSlop = 65536 // ~1.5e-5 raw bits

	wasOnGround := b.OnGround
	b.OnGround = false

	for iter := 0; iter < maxResolveIters; iter++ {
		found := false
		var best TriangleContact

		for i := 0; i < len(tris); i++ {
			tri := tris[i]
			c := sphereTriangleContact(b.Pos, b.Radius.Add(fixed.FromRaw(groundingSlop)), tri, i)
			if !c.Hit {
				continue
			}

			if !found ||
				c.Penetration.Cmp(best.Penetration) > 0 ||
				(c.Penetration == best.Penetration && c.TriangleIndex < best.TriangleIndex) {
				best = c
				found = true
			}
		}

		if !found {
			break
		}

		// Apply positional correction
		strict := sphereTriangleContact(b.Pos, b.Radius, tris[best.TriangleIndex], best.TriangleIndex)
		if strict.Hit && strict.Penetration.Cmp(fixed.Zero) > 0 {
			b.Pos = b.Pos.Add(strict.Normal.Scale(strict.Penetration))
		}

		// Velocity response
		vn := b.Vel.Dot(best.Normal)
		if vn.Cmp(fixed.Zero) < 0 {
			b.Vel = b.Vel.Sub(best.Normal.Scale(vn))
		}

		if best.Normal.Y.Cmp(GroundNormalMinY) >= 0 {
			b.OnGround = true
		}

		w.LastContact = ContactDebug{
			HasContact: true,
			Point:      best.Point,
			Normal:     best.Normal,
		}
	}

	if !b.OnGround && wasOnGround {
		for i := 0; i < len(tris); i++ {
			tri := tris[i]
			c := sphereTriangleContact(b.Pos, b.Radius.Add(fixed.FromRaw(groundingSlop)), tri, i)
			if c.Hit && c.Normal.Y.Cmp(GroundNormalMinY) >= 0 {
				b.OnGround = true
				break
			}
		}
	}
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
