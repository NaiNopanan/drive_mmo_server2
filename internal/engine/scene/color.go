package scene

// Color เป็นสีกลางของ engine เพื่อไม่ให้ component ต้องผูกกับ raylib ตรง ๆ
type Color struct {
	R uint8
	G uint8
	B uint8
	A uint8
}

// NewColorRGBA ช่วยสร้างสีแบบสั้น ๆ
func NewColorRGBA(r, g, b, a uint8) Color {
	return Color{R: r, G: g, B: b, A: a}
}
