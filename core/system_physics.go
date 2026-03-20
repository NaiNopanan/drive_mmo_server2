package core

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/physics"
	"server2/world"
)

// PhysicsIntegrateSystem เดินเฉพาะ body dynamic ไปข้างหน้า
type PhysicsIntegrateSystem struct {
	Gravity geometry.Vector3
}

func (s PhysicsIntegrateSystem) Step(w *world.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, body := range w.Bodies {
		if !w.HasEntity(id) {
			continue
		}
		physics.IntegrateBody(&body, dt, s.Gravity)
		w.Bodies[id] = body
	}
}
