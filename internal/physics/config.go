package physics

import (
	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

type WorldConfig struct {
	FixedDT     float32
	WorldBounds geom.AABB
	PlayerSpawn geom.PlanarVec
	StaticMesh  worldmesh.StaticMesh
}

// DefaultVehicleParams เป็นค่าพื้นฐานที่อ่านง่ายและยังขับได้แบบ arcade
func DefaultVehicleParams() VehicleParams {
	return VehicleParams{
		// ใช้มาตรฐาน 1 world unit = 1 meter
		MaxForwardSpeed:    58,
		MaxReverseSpeed:    14,
		AccelRate:          16,
		BrakeRate:          26,
		Drag:               0.08,
		RollingResistance:  0.9,
		MaxSteerRate:       2.6,
		SteerResponse:      8,
		HighSpeedSteerDamp: 0.55,
		Grip:               9.0,
		DriftGrip:          3.2,
		BodyLength:         4.4,
		BodyHeight:         0.9,
		BodyWidth:          1.9,
		Suspension: SuspensionParams{
			RestLength:      0.50,
			MaxTravel:       0.22,
			WheelRadius:     0.32,
			MountHeight:     0.58,
			FrontAxleOffset: 1.25,
			RearAxleOffset:  1.15,
			HalfTrackWidth:  0.78,
		},
	}
}

// DefaultWorldConfig สร้างโลกเรียบง่ายสำหรับลองระบบรถใหม่ทีละชั้น
func DefaultWorldConfig() WorldConfig {
	return WorldConfig{
		FixedDT:     1.0 / 20.0,
		WorldBounds: geom.NewAABB(-760, -520, 760, 520),
		PlayerSpawn: geom.Planar(0, 0),
	}
}
