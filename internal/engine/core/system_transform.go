package core

import (
	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	"server2/internal/engine/ecs"
	"server2/internal/engine/scene"
)

// TransformSyncSystem sync physics body state กลับไปที่ transform component
type TransformSyncSystem struct{}

func (TransformSyncSystem) Step(w *ecs.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, body := range w.Bodies {
		if !w.HasEntity(id) {
			continue
		}
		transform, ok := w.Transforms[id]
		if !ok {
			transform = scene.IdentityTransform()
		}
		transform.Position = body.Position
		transform.Rotation = body.Orientation
		if transform.Scale.LengthSquared() == fixed.Zero {
			transform.Scale = geometry.NewVector3(fixed.One, fixed.One, fixed.One)
		}
		w.Transforms[id] = transform
	}
}
