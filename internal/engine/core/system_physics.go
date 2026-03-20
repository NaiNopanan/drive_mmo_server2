package core

import (
	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	"server2/internal/engine/ecs"
	enginephysics "server2/internal/engine/physics"
)

// PhysicsIntegrateSystem เดินเฉพาะ body dynamic ไปข้างหน้า
type PhysicsIntegrateSystem struct {
	Gravity geometry.Vector3
}

func (s PhysicsIntegrateSystem) Step(w *ecs.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, body := range w.Bodies {
		if !w.HasEntity(id) {
			continue
		}
		enginephysics.IntegrateBody(&body, dt, s.Gravity)
		w.Bodies[id] = body
	}
}
