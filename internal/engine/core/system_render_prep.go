package core

import (
	"server2/internal/base/fixed"
	"server2/internal/engine/ecs"
	enginephysics "server2/internal/engine/physics"
	"server2/internal/engine/scene"
)

// DebugRenderPrepSystem เติมค่า render component ที่จำเป็นสำหรับ debug renderer
type DebugRenderPrepSystem struct{}

func (DebugRenderPrepSystem) Step(w *ecs.World, dt fixed.Fixed) {
	if w == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	for id, renderComponent := range w.PrimitiveRenders {
		if !w.HasEntity(id) {
			continue
		}

		if renderComponent.Color.A == 0 {
			if body, ok := w.Bodies[id]; ok {
				renderComponent.Color = defaultBodyColor(body.BodyType)
			}
		}
		if !renderComponent.Visible && renderComponent.Color.A != 0 {
			renderComponent.Visible = true
		}

		w.PrimitiveRenders[id] = renderComponent
	}
}

func defaultBodyColor(bodyType enginephysics.BodyType) scene.Color {
	switch bodyType {
	case enginephysics.BodyTypeDynamic:
		return scene.NewColorRGBA(80, 170, 255, 255)
	case enginephysics.BodyTypeKinematic:
		return scene.NewColorRGBA(245, 160, 60, 255)
	default:
		return scene.NewColorRGBA(185, 185, 185, 255)
	}
}
