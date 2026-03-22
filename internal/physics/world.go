package physics

import (
	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

// PhysicsWorld เป็น sandbox เรียบง่ายที่โฟกัสเฉพาะรถผู้เล่น 1 คัน
type PhysicsWorld struct {
	tick       uint64
	config     WorldConfig
	staticMesh worldmesh.StaticMesh
	player     VehicleBody
}

func NewWorld(config WorldConfig) *PhysicsWorld {
	spawnGroundHeight := querySpawnGroundHeight(config.StaticMesh, config.PlayerSpawn)

	return &PhysicsWorld{
		config:     config,
		staticMesh: config.StaticMesh,
		player: VehicleBody{
			ID:           "player-1",
			Position:     config.PlayerSpawn,
			Height:       spawnGroundHeight + 3,
			GroundHeight: spawnGroundHeight,
			VerticalVel:  0,
			LastSafePos:  config.PlayerSpawn,
			Params:       DefaultVehicleParams(),
		},
	}
}

func (w *PhysicsWorld) FixedDT() float32 {
	return w.config.FixedDT
}

func (w *PhysicsWorld) Reset() {
	resetWorld := NewWorld(w.config)
	*w = *resetWorld
}

func (w *PhysicsWorld) Snapshot() WorldSnapshot {
	return WorldSnapshot{
		Tick:       w.tick,
		Bounds:     w.config.WorldBounds,
		StaticMesh: w.staticMesh,
		Player:     snapshotFromVehicle(w.player, true),
	}
}

func snapshotFromVehicle(vehicle VehicleBody, isPlayer bool) VehicleSnapshot {
	colliderRadius, colliderHalfLength := bodyCapsuleDimensions(vehicle.Params)
	snapshot := VehicleSnapshot{
		ID:                 vehicle.ID,
		Position:           vehicle.Position,
		Velocity:           vehicle.Velocity,
		Heading:            vehicle.Heading,
		Pitch:              vehicle.Pitch,
		Roll:               vehicle.Roll,
		Speed:              vehicle.Speed,
		Height:             vehicle.Height,
		GroundHeight:       vehicle.GroundHeight,
		BodyHitMap:         vehicle.BodyHitMap,
		OBBCCD:             vehicle.OBBCCD,
		SupportState:       vehicle.SupportState,
		SupportHits:        vehicle.SupportHits,
		Length:             vehicle.Params.BodyLength,
		Width:              vehicle.Params.BodyWidth,
		BodyHeight:         vehicle.Params.BodyHeight,
		ColliderRadius:     colliderRadius,
		ColliderHalfLength: colliderHalfLength,
		IsPlayer:           isPlayer,
	}

	for index, wheel := range vehicle.Wheels {
		snapshot.Wheels[index] = WheelSnapshot{
			Label:            wheel.Label,
			MountPoint:       wheel.MountPoint,
			Hit:              wheel.Hit,
			HitPoint:         wheel.HitPoint,
			WheelCenter:      wheel.WheelCenter,
			HitDistance:      wheel.HitDistance,
			SuspensionLength: wheel.SuspensionLength,
			Compression:      wheel.Compression,
		}
	}

	return snapshot
}

func querySpawnGroundHeight(staticMesh worldmesh.StaticMesh, spawn geom.PlanarVec) float32 {
	if len(staticMesh.Triangles) == 0 {
		return 0
	}

	originY := staticMesh.Max.Y + 8
	maxDistance := (staticMesh.Max.Y - staticMesh.Min.Y) + 16
	hit := staticMesh.RaycastDown(geom.V3(spawn.X, originY, spawn.Z), maxDistance)
	if !hit.Hit {
		return 0
	}

	return hit.Point.Y
}
