package ecs

import (
	"server2/internal/engine/physics"
	"server2/internal/engine/scene"
)

// SetName ผูก NameComponent ให้ entity เป้าหมาย
func (w *World) SetName(id EntityID, component scene.NameComponent) bool {
	if !w.HasEntity(id) {
		return false
	}
	w.Names[id] = component
	return true
}

// RemoveName ลบ NameComponent ออกจาก entity
func (w *World) RemoveName(id EntityID) {
	if w == nil {
		return
	}
	delete(w.Names, id)
}

// SetTransform ผูก TransformComponent ให้ entity เป้าหมาย
func (w *World) SetTransform(id EntityID, component scene.TransformComponent) bool {
	if !w.HasEntity(id) {
		return false
	}
	w.Transforms[id] = component
	return true
}

// RemoveTransform ลบ TransformComponent ออกจาก entity
func (w *World) RemoveTransform(id EntityID) {
	if w == nil {
		return
	}
	delete(w.Transforms, id)
}

// SetBody ผูก BodyComponent ให้ entity เป้าหมาย
func (w *World) SetBody(id EntityID, component physics.BodyComponent) bool {
	if !w.HasEntity(id) {
		return false
	}
	w.Bodies[id] = component
	return true
}

// RemoveBody ลบ BodyComponent ออกจาก entity
func (w *World) RemoveBody(id EntityID) {
	if w == nil {
		return
	}
	delete(w.Bodies, id)
}

// SetPrimitiveRender ผูก PrimitiveRenderComponent ให้ entity เป้าหมาย
func (w *World) SetPrimitiveRender(id EntityID, component scene.PrimitiveRenderComponent) bool {
	if !w.HasEntity(id) {
		return false
	}
	w.PrimitiveRenders[id] = component
	return true
}

// RemovePrimitiveRender ลบ PrimitiveRenderComponent ออกจาก entity
func (w *World) RemovePrimitiveRender(id EntityID) {
	if w == nil {
		return
	}
	delete(w.PrimitiveRenders, id)
}
