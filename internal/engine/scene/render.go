package scene

// PrimitiveRenderComponent บอกว่าควรวาด object นี้แบบ primitive debug อย่างไร
type PrimitiveRenderComponent struct {
	Visible  bool
	Color    Color
	DrawWire bool
}

// NewPrimitiveRender คืนค่า render component ที่เปิดให้มองเห็นทันที
func NewPrimitiveRender(color Color) PrimitiveRenderComponent {
	return PrimitiveRenderComponent{
		Visible: true,
		Color:   color,
	}
}
