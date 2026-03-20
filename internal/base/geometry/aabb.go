package geometry

// AABB คือ axis-aligned bounding box แบบย่อให้อ่านง่ายขึ้น
type AABB struct {
	Min Vector3
	Max Vector3
}

// NewAABB สร้าง AABB ใหม่และตรวจว่าค่า min/max ถูกต้อง
func NewAABB(min, max Vector3) AABB {
	if min.X > max.X || min.Y > max.Y || min.Z > max.Z {
		panic("invalid aabb: min > max")
	}
	return AABB{Min: min, Max: max}
}

// Intersects เช็กว่า AABB สองตัวทับกันหรือไม่
func (a AABB) Intersects(b AABB) bool {
	if a.Max.X < b.Min.X || a.Min.X > b.Max.X {
		return false
	}
	if a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y {
		return false
	}
	if a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z {
		return false
	}
	return true
}

// ContainsPoint เช็กว่า point อยู่ภายใน AABB นี้หรือไม่
func (a AABB) ContainsPoint(p Vector3) bool {
	if p.X < a.Min.X || p.X > a.Max.X {
		return false
	}
	if p.Y < a.Min.Y || p.Y > a.Max.Y {
		return false
	}
	if p.Z < a.Min.Z || p.Z > a.Max.Z {
		return false
	}
	return true
}
