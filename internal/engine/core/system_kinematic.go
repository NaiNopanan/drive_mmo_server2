package core

import (
	"server2/internal/base/fixed"
	"server2/internal/engine/ecs"
	enginephysics "server2/internal/engine/physics"
)

// KinematicSystem อัปเดต body แบบ kinematic จาก target ที่ gameplay code ตั้งไว้
type KinematicSystem struct{}

func (KinematicSystem) Step(w *ecs.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, body := range w.Bodies {
		if !w.HasEntity(id) || body.BodyType != enginephysics.BodyTypeKinematic {
			continue
		}
		body.Position = body.KinematicTargetPosition
		body.Velocity = body.KinematicTargetVelocity
		w.Bodies[id] = body
	}
}
