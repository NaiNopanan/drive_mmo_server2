package world

import (
	"server2/math/fixed"
	"server2/physics"
)

// World คือ owner ของ entity และ component stores ทั้งหมด
// v1 ใช้ map-based storage เพื่อให้โครงชัดและแก้ต่อได้ง่าย
type World struct {
	Config WorldConfig

	nextEntityID EntityID
	alive        map[EntityID]struct{}

	Systems          []System
	Names            map[EntityID]NameComponent
	Transforms       map[EntityID]TransformComponent
	Bodies           map[EntityID]physics.BodyComponent
	PrimitiveRenders map[EntityID]PrimitiveRenderComponent
}

// NewWorld สร้าง ECS world เปล่า
func NewWorld(config WorldConfig) *World {
	return &World{
		Config:           config,
		alive:            make(map[EntityID]struct{}),
		Names:            make(map[EntityID]NameComponent),
		Transforms:       make(map[EntityID]TransformComponent),
		Bodies:           make(map[EntityID]physics.BodyComponent),
		PrimitiveRenders: make(map[EntityID]PrimitiveRenderComponent),
	}
}

// NewEntity สร้าง entity ใหม่และคืน id กลับไป
func (w *World) NewEntity() EntityID {
	w.nextEntityID++
	id := w.nextEntityID
	w.alive[id] = struct{}{}
	return id
}

// DestroyEntity ลบ entity และ component ทั้งหมดของมันออกจาก world
func (w *World) DestroyEntity(id EntityID) {
	if w == nil {
		return
	}
	delete(w.alive, id)
	delete(w.Names, id)
	delete(w.Transforms, id)
	delete(w.Bodies, id)
	delete(w.PrimitiveRenders, id)
}

// HasEntity ใช้เช็กว่า entity นี้ยังมีอยู่ใน world หรือไม่
func (w *World) HasEntity(id EntityID) bool {
	if w == nil {
		return false
	}
	_, ok := w.alive[id]
	return ok
}

// EntityIDs คืนรายการ entity ที่ยังมีชีวิตอยู่ใน world
func (w *World) EntityIDs() []EntityID {
	if w == nil {
		return nil
	}
	ids := make([]EntityID, 0, len(w.alive))
	for id := range w.alive {
		ids = append(ids, id)
	}
	return ids
}

// AddSystem เพิ่ม system เข้า world ตามลำดับที่เรียก
func (w *World) AddSystem(system System) {
	if w == nil || system == nil {
		return
	}
	w.Systems = append(w.Systems, system)
}

// Step เรียกทุก system ใน world 1 รอบตามลำดับ
func (w *World) Step(dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}
	for _, system := range w.Systems {
		system.Step(w, dt)
	}
}
