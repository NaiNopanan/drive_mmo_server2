package sim

import (
	"encoding/binary"
	"hash/fnv"

	"server2/internal/fixed"
	"server2/internal/geom"
)

type VehicleInput struct {
	Throttle fixed.Fixed // -1..+1
	Brake    fixed.Fixed // 0..1
	Steer    fixed.Fixed // -1..1
}

type VehicleTuning struct {
	Mass          fixed.Fixed
	InvMass       fixed.Fixed
	YawInertia    fixed.Fixed
	InvYawInertia fixed.Fixed

	WheelBase  fixed.Fixed
	TrackWidth fixed.Fixed

	MaxSteerAngleRad   fixed.Fixed
	SteerRateRadPerSec fixed.Fixed

	SuspensionRestLength fixed.Fixed
	SuspensionMaxDrop    fixed.Fixed
	SuspensionMaxRaise   fixed.Fixed
	SuspensionStiffness  fixed.Fixed
	SuspensionDamping    fixed.Fixed
	MaxSuspensionForce   fixed.Fixed

	WheelRadius       fixed.Fixed
	DriveForce        fixed.Fixed
	BrakeForce        fixed.Fixed
	RollingResistance fixed.Fixed
	LateralGrip       fixed.Fixed

	MaxSpeed fixed.Fixed
}

type WheelDef struct {
	ID          int
	LocalAnchor geom.Vec3
	IsFront     bool
	IsDriven    bool
}

type WheelState struct {
	SteerAngleRad fixed.Fixed

	InContact       bool
	ContactPoint    geom.Vec3
	ContactNormal   geom.Vec3
	ContactDistance fixed.Fixed

	Compression     fixed.Fixed
	PrevCompression fixed.Fixed

	SpringForce     fixed.Fixed
	DamperForce     fixed.Fixed
	SuspensionForce fixed.Fixed

	DriveForce   fixed.Fixed
	BrakeForce   fixed.Fixed
	RollingForce fixed.Fixed
	LateralForce fixed.Fixed

	WheelForwardWS geom.Vec3
	WheelRightWS   geom.Vec3

	LongSpeed fixed.Fixed
	LatSpeed  fixed.Fixed
}

type Vehicle struct {
	ID     uint32
	Tuning VehicleTuning
	Input  VehicleInput

	Position    geom.Vec3
	Velocity    geom.Vec3
	Yaw         fixed.Fixed
	YawVelocity fixed.Fixed

	ForwardWS geom.Vec3
	RightWS   geom.Vec3
	UpWS      geom.Vec3

	WheelDefs [4]WheelDef
	Wheels    [4]WheelState

	OnGround       bool
	GroundedWheels int

	TotalForce   geom.Vec3
	TotalTorqueY fixed.Fixed

	Hash uint64
}

func DefaultTuning() VehicleTuning {
	mass := fixed.FromInt(1200)
	yawInertia := fixed.FromInt(1600) // ลดจาก 2200 → เลี้ยวได้ง่ายขึ้น

	return VehicleTuning{
		Mass:          mass,
		InvMass:       fixed.One.Div(mass),
		YawInertia:    yawInertia,
		InvYawInertia: fixed.One.Div(yawInertia),

		WheelBase:  fixed.FromFraction(26, 10), // 2.6
		TrackWidth: fixed.FromFraction(16, 10), // 1.6

		MaxSteerAngleRad:   fixed.FromFraction(48, 100), // ~28deg (↑ คืนมา ให้เลี้ยวคมขึ้น)
		SteerRateRadPerSec: fixed.FromFraction(19, 10),  // ~109deg/s (↑ พวงมาลัวตอบสนองเร็วขึ้น)

		SuspensionRestLength: fixed.FromFraction(35, 100),
		SuspensionMaxDrop:    fixed.FromFraction(20, 100),
		SuspensionMaxRaise:   fixed.FromFraction(10, 100),
		SuspensionStiffness:  fixed.FromInt(35000),
		SuspensionDamping:    fixed.FromInt(5000),
		MaxSuspensionForce:   fixed.FromInt(30000),

		WheelRadius:       fixed.FromFraction(34, 100),
		DriveForce:        fixed.FromInt(7000),
		BrakeForce:        fixed.FromInt(9000),
		RollingResistance: fixed.FromInt(300),
		LateralGrip:       fixed.FromInt(2800), // กลาง: ไม่แข็งเกิน ไม่ไถลเกิน

		MaxSpeed: fixed.FromInt(40),
	}
}

// PrototypeTuning returns an AWD-tuned vehicle with forgiving handling,
// ideal for prototype testing where driveability matters more than realism.
func PrototypeTuning() VehicleTuning {
	t := DefaultTuning()
	return t
}

func NewVehicle(id uint32, pos geom.Vec3) Vehicle {
	t := DefaultTuning()
	v := Vehicle{
		ID:        id,
		Tuning:    t,
		Position:  pos,
		ForwardWS: geom.V3(fixed.Zero, fixed.Zero, fixed.One),
		RightWS:   geom.V3(fixed.One, fixed.Zero, fixed.Zero),
		UpWS:      geom.V3(fixed.Zero, fixed.One, fixed.Zero),
	}

	hx := t.TrackWidth.Div(fixed.FromInt(2))
	hz := t.WheelBase.Div(fixed.FromInt(2))

	// RWD: ล้อหลังขับ, ล้อหน้าเลี้ยว — ขับสนุกกว่า FWD สำหรับ prototype
	v.WheelDefs = [4]WheelDef{
		{ID: 0, IsFront: true, IsDriven: false, LocalAnchor: geom.V3(hx.Neg(), fixed.Zero, hz)},
		{ID: 1, IsFront: true, IsDriven: false, LocalAnchor: geom.V3(hx, fixed.Zero, hz)},
		{ID: 2, IsFront: false, IsDriven: true, LocalAnchor: geom.V3(hx.Neg(), fixed.Zero, hz.Neg())},
		{ID: 3, IsFront: false, IsDriven: true, LocalAnchor: geom.V3(hx, fixed.Zero, hz.Neg())},
	}

	return v
}

// NewAWDVehicle creates a vehicle with all-wheel drive — easiest to prototype with.
func NewAWDVehicle(id uint32, pos geom.Vec3) Vehicle {
	v := NewVehicle(id, pos)
	for i := range v.WheelDefs {
		v.WheelDefs[i].IsDriven = true
	}
	return v
}

func HashVehicle(v Vehicle) uint64 {
	h := fnv.New64a()
	buf := make([]byte, 8)

	writeI64 := func(v int64) {
		binary.LittleEndian.PutUint64(buf, uint64(v))
		_, _ = h.Write(buf)
	}

	writeI64(int64(v.ID))
	writeI64(v.Position.X.Raw())
	writeI64(v.Position.Y.Raw())
	writeI64(v.Position.Z.Raw())
	writeI64(v.Velocity.X.Raw())
	writeI64(v.Velocity.Y.Raw())
	writeI64(v.Velocity.Z.Raw())
	writeI64(v.Yaw.Raw())
	writeI64(v.YawVelocity.Raw())

	for i := range v.Wheels {
		w := v.Wheels[i]
		writeI64(w.Compression.Raw())
		if w.InContact {
			writeI64(1)
		} else {
			writeI64(0)
		}
	}

	return h.Sum64()
}
