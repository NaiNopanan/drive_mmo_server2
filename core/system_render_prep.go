package core

import (
	"server2/math/fixed"
	"server2/physics"
	"server2/world"
)

// DebugRenderPrepSystem เติมค่า render component ที่จำเป็นสำหรับ debug renderer
type DebugRenderPrepSystem struct{}

func (DebugRenderPrepSystem) Step(w *world.World, dt fixed.Fixed) {
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

func defaultBodyColor(bodyType physics.BodyType) world.Color {
	switch bodyType {
	case physics.BodyTypeDynamic:
		return world.NewColorRGBA(80, 170, 255, 255)
	case physics.BodyTypeKinematic:
		return world.NewColorRGBA(245, 160, 60, 255)
	default:
		return world.NewColorRGBA(185, 185, 185, 255)
	}
}
