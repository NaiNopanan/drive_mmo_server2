package core

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/world"
)

// TransformSyncSystem sync physics body state กลับไปที่ transform component
type TransformSyncSystem struct{}

func (TransformSyncSystem) Step(w *world.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, body := range w.Bodies {
		if !w.HasEntity(id) {
			continue
		}
		transform, ok := w.Transforms[id]
		if !ok {
			transform = world.IdentityTransform()
		}
		transform.Position = body.Position
		transform.Rotation = body.Orientation
		if transform.Scale.LengthSquared() == fixed.Zero {
			transform.Scale = geometry.NewVector3(fixed.One, fixed.One, fixed.One)
		}
		w.Transforms[id] = transform
	}
}
