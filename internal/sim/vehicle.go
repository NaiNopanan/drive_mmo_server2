package sim

import (
	"encoding/binary"
	"hash/fnv"

	"server2/internal/fixed"
	"server2/internal/geom"
)

var (
	VehicleDt = Dt

	VehicleGravityY = fixed.FromFraction(-98, 10)

	VehicleMaxSpeed   = fixed.FromInt(35)
	VehicleSteerRate  = fixed.FromInt(3)
	VehicleLinearDrag = fixed.FromFraction(3, 2) // 1.5
)

type VehicleInput struct {
	Throttle fixed.Fixed // 0..1
	Brake    fixed.Fixed // 0..1
	Steer    fixed.Fixed // -1..1
}

type VehicleBody struct {
	Pos      geom.Vec3
	Vel      geom.Vec3
	Forward  geom.Vec3
	HalfSize geom.Vec3

	Mass    fixed.Fixed
	InvMass fixed.Fixed

	OnGround bool
}

type Wheel struct {
	LocalMount     geom.Vec3
	Radius         fixed.Fixed
	SuspensionRest fixed.Fixed

	SpringK    fixed.Fixed
	DamperC    fixed.Fixed
	DriveForce fixed.Fixed
	BrakeForce fixed.Fixed
	LateralGrip fixed.Fixed

	Driven    bool
	Steerable bool

	// runtime
	Contact        bool
	ContactPoint   geom.Vec3
	ContactNormal  geom.Vec3
	SuspensionLen  fixed.Fixed
	Compression    fixed.Fixed
	PrevCompression fixed.Fixed
	Load           fixed.Fixed
}

type Vehicle struct {
	ID     uint32
	Body   VehicleBody
	Wheels [4]Wheel
	Input  VehicleInput
}

type VehicleWorld struct {
	Tick            uint64
	GroundTriangles []geom.Triangle
	Vehicles        []Vehicle
}

func NewVehicleWorld(ground []geom.Triangle, vehicles []Vehicle) VehicleWorld {
	if ground == nil {
		ground = GroundFlatSmall()
	}

	return VehicleWorld{
		Tick:            0,
		GroundTriangles: ground,
		Vehicles:        vehicles,
	}
}

func NewDefaultVehicle(id uint32, pos geom.Vec3) Vehicle {
	mass := fixed.FromInt(1200)

	v := Vehicle{
		ID: id,
		Body: VehicleBody{
			Pos:      pos,
			Vel:      geom.Zero(),
			Forward:  geom.V3(fixed.Zero, fixed.Zero, fixed.One),
			HalfSize: geom.V3(fixed.FromFraction(9, 10), fixed.FromFraction(7, 20), fixed.FromFraction(9, 5)), // 0.9, 0.35, 1.8
			Mass:     mass,
			InvMass:  fixed.One.Div(mass),
			OnGround: false,
		},
	}

	frontZ := fixed.FromFraction(13, 10) // 1.3
	rearZ := fixed.FromFraction(-6, 5)   // -1.2
	sideX := fixed.FromFraction(9, 10)   // 0.9
	mountY := fixed.FromFraction(-1, 4)  // -0.25

	makeWheel := func(localX, localZ fixed.Fixed, driven, steerable bool) Wheel {
		return Wheel{
			LocalMount:     geom.V3(localX, mountY, localZ),
			Radius:         fixed.FromFraction(7, 20),  // 0.35
			SuspensionRest: fixed.FromFraction(9, 20),  // 0.45
			SpringK:        fixed.FromInt(18000),
			DamperC:        fixed.FromInt(4000),
			DriveForce:     fixed.FromInt(2800),
			BrakeForce:     fixed.FromInt(3500),
			LateralGrip:    fixed.FromInt(1400),
			Driven:         driven,
			Steerable:      steerable,
		}
	}

	// FL FR RL RR
	v.Wheels[0] = makeWheel(sideX.Neg(), frontZ, true, true)
	v.Wheels[1] = makeWheel(sideX, frontZ, true, true)
	v.Wheels[2] = makeWheel(sideX.Neg(), rearZ, false, false)
	v.Wheels[3] = makeWheel(sideX, rearZ, false, false)

	return v
}

func SpawnVehicleGrid(rows int, cols int, spacing int64, startY int64) []Vehicle {
	out := make([]Vehicle, 0, rows*cols)

	step := fixed.FromInt(spacing)
	baseY := fixed.FromInt(startY)

	id := uint32(1)
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			x := fixed.FromInt(int64(c)).Mul(step)
			z := fixed.FromInt(int64(r)).Mul(step)

			// center around origin a bit
			x = x.Sub(fixed.FromInt(int64(cols-1) * spacing / 2))
			z = z.Sub(fixed.FromInt(int64(rows-1) * spacing / 2))

			out = append(out, NewDefaultVehicle(id, geom.V3(x, baseY, z)))
			id++
		}
	}

	return out
}

func VehicleBasis(v Vehicle) (forward geom.Vec3, right geom.Vec3, up geom.Vec3) {
	up = geom.V3(fixed.Zero, fixed.One, fixed.Zero)

	forward = normalizeHorizontal(v.Body.Forward)
	right = geom.V3(forward.Z, fixed.Zero, forward.X.Neg())

	return
}

func VehicleWheelMountWorld(v Vehicle, wheelIndex int) geom.Vec3 {
	forward, right, up := VehicleBasis(v)
	w := v.Wheels[wheelIndex]

	return v.Body.Pos.
		Add(right.Scale(w.LocalMount.X)).
		Add(up.Scale(w.LocalMount.Y)).
		Add(forward.Scale(w.LocalMount.Z))
}

func VehicleWheelCenterWorld(v Vehicle, wheelIndex int) geom.Vec3 {
	w := v.Wheels[wheelIndex]
	mount := VehicleWheelMountWorld(v, wheelIndex)

	if w.Contact {
		return geom.V3(
			mount.X,
			w.ContactPoint.Y.Add(w.Radius),
			mount.Z,
		)
	}

	return geom.V3(
		mount.X,
		mount.Y.Sub(w.SuspensionRest),
		mount.Z,
	)
}

func HashVehicleWorld(w VehicleWorld) uint64 {
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
	writeU64(uint64(len(w.Vehicles)))

	for i := range w.Vehicles {
		v := w.Vehicles[i]

		writeU64(uint64(v.ID))

		writeI64(v.Body.Pos.X.Raw())
		writeI64(v.Body.Pos.Y.Raw())
		writeI64(v.Body.Pos.Z.Raw())

		writeI64(v.Body.Vel.X.Raw())
		writeI64(v.Body.Vel.Y.Raw())
		writeI64(v.Body.Vel.Z.Raw())

		writeI64(v.Body.Forward.X.Raw())
		writeI64(v.Body.Forward.Y.Raw())
		writeI64(v.Body.Forward.Z.Raw())

		writeBool(v.Body.OnGround)

		for wi := range v.Wheels {
			wh := v.Wheels[wi]

			writeBool(wh.Contact)
			writeI64(wh.ContactPoint.X.Raw())
			writeI64(wh.ContactPoint.Y.Raw())
			writeI64(wh.ContactPoint.Z.Raw())

			writeI64(wh.ContactNormal.X.Raw())
			writeI64(wh.ContactNormal.Y.Raw())
			writeI64(wh.ContactNormal.Z.Raw())

			writeI64(wh.SuspensionLen.Raw())
			writeI64(wh.Compression.Raw())
			writeI64(wh.Load.Raw())
		}
	}

	return h.Sum64()
}

func normalizeHorizontal(v geom.Vec3) geom.Vec3 {
	h := geom.V3(v.X, fixed.Zero, v.Z)
	if h.LengthSq() == fixed.Zero {
		return geom.V3(fixed.Zero, fixed.Zero, fixed.One)
	}
	return h.Normalize()
}
