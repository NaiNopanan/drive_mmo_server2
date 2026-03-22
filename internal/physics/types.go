package physics

import (
	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const wheelCount = 4

type SupportState string

const (
	SupportStateStable  SupportState = "stable"
	SupportStateEdge    SupportState = "edge"
	SupportStateFalling SupportState = "falling"
)

// DriveInput คือ input ดิบที่ส่งเข้าแกนรถโดยตรง
type DriveInput struct {
	Throttle  float32
	Brake     float32
	Steering  float32
	Handbrake bool
	Nitro     bool
}

// SuspensionParams คือค่าพื้นฐานของระบบล้อแบบ raycast suspension
type SuspensionParams struct {
	RestLength      float32
	MaxTravel       float32
	WheelRadius     float32
	MountHeight     float32
	FrontAxleOffset float32
	RearAxleOffset  float32
	HalfTrackWidth  float32
}

// VehicleParams รวมค่าปรับแต่งหลักของรถ
type VehicleParams struct {
	MaxForwardSpeed    float32
	MaxReverseSpeed    float32
	AccelRate          float32
	BrakeRate          float32
	Drag               float32
	RollingResistance  float32
	MaxSteerRate       float32
	SteerResponse      float32
	HighSpeedSteerDamp float32
	Grip               float32
	DriftGrip          float32
	BodyLength         float32
	BodyHeight         float32
	BodyWidth          float32
	Suspension         SuspensionParams
}

// WheelState คือข้อมูล raycast ของล้อแต่ละมุมใน tick ปัจจุบัน
type WheelState struct {
	Label            string
	MountPoint       geom.Vec3
	Hit              bool
	HitPoint         geom.Vec3
	WheelCenter      geom.Vec3
	HitDistance      float32
	SuspensionLength float32
	Compression      float32
}

// VehicleBody คือสถานะฟิสิกส์ของรถหนึ่งคันใน sandbox
type VehicleBody struct {
	ID           string
	Position     geom.PlanarVec
	Velocity     geom.PlanarVec
	Heading      float32
	Pitch        float32
	Roll         float32
	YawRate      float32
	ReverseSteer bool
	Speed        float32
	Height       float32
	GroundHeight float32
	VerticalVel  float32
	BodyHitMap   bool
	OBBCCD       OBBCCDDebug
	Kinematic    KinematicDebug
	SupportState SupportState
	SupportHits  int
	Wheels       [wheelCount]WheelState
	LastSafePos  geom.PlanarVec
	Params       VehicleParams
}

// WheelSnapshot คือข้อมูลอ่านอย่างเดียวของล้อสำหรับใช้วาด debug
type WheelSnapshot struct {
	Label            string
	MountPoint       geom.Vec3
	Hit              bool
	HitPoint         geom.Vec3
	WheelCenter      geom.Vec3
	HitDistance      float32
	SuspensionLength float32
	Compression      float32
}

type OBBCCDDebug struct {
	Hit      bool
	Time     float32
	Position geom.PlanarVec
	Height   float32
	Heading  float32
	Pitch    float32
	Roll     float32
	Normal   geom.Vec3
}

type KinematicDebug struct {
	Grounded          bool
	ContactCount      int
	Substeps          int
	Throttle          float32
	Brake             float32
	Steering          float32
	ForwardVector     geom.Vec3
	RightVector       geom.Vec3
	PlanarVelocity    geom.Vec3
	ProjectedVelocity geom.Vec3
	MoveDelta         geom.Vec3
	ContactNormal     geom.Vec3
	GroundNormal      geom.Vec3
	GroundProbeHit    bool
	GroundProbeOrigin geom.Vec3
	GroundProbePoint  geom.Vec3
	GroundProbeNormal geom.Vec3
	LookAheadHit      bool
	LookAheadOrigin   geom.Vec3
	LookAheadPoint    geom.Vec3
	LookAheadNormal   geom.Vec3
	SnapApplied       bool
}

// VehicleSnapshot คือข้อมูลอ่านอย่างเดียวสำหรับ viewer
type VehicleSnapshot struct {
	ID                 string
	Position           geom.PlanarVec
	Velocity           geom.PlanarVec
	Heading            float32
	Pitch              float32
	Roll               float32
	Speed              float32
	Height             float32
	GroundHeight       float32
	VerticalVel        float32
	BodyHitMap         bool
	OBBCCD             OBBCCDDebug
	Kinematic          KinematicDebug
	SupportState       SupportState
	SupportHits        int
	Wheels             [wheelCount]WheelSnapshot
	Length             float32
	Width              float32
	BodyHeight         float32
	ColliderRadius     float32
	ColliderHalfLength float32
	IsPlayer           bool
}

// WorldSnapshot คือภาพรวมของโลกใน tick ปัจจุบัน
type WorldSnapshot struct {
	Tick       uint64
	Bounds     geom.AABB
	StaticMesh worldmesh.StaticMesh
	Player     VehicleSnapshot
}
