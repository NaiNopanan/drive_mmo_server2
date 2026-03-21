package geom

// AABB ใช้แทนกรอบสี่เหลี่ยมบนระนาบ X/Z สำหรับ world bounds, ถนน และ trigger
type AABB struct {
	Min PlanarVec
	Max PlanarVec
}

func NewAABB(minX, minZ, maxX, maxZ float32) AABB {
	return AABB{
		Min: PlanarVec{X: minX, Z: minZ},
		Max: PlanarVec{X: maxX, Z: maxZ},
	}
}

func (b AABB) Width() float32 {
	return b.Max.X - b.Min.X
}

func (b AABB) Height() float32 {
	return b.Max.Z - b.Min.Z
}

func (b AABB) ContainsPoint(point PlanarVec) bool {
	return point.X >= b.Min.X && point.X <= b.Max.X &&
		point.Z >= b.Min.Z && point.Z <= b.Max.Z
}

func (b AABB) ClampPoint(point PlanarVec) PlanarVec {
	clamped := point

	if clamped.X < b.Min.X {
		clamped.X = b.Min.X
	}
	if clamped.X > b.Max.X {
		clamped.X = b.Max.X
	}
	if clamped.Z < b.Min.Z {
		clamped.Z = b.Min.Z
	}
	if clamped.Z > b.Max.Z {
		clamped.Z = b.Max.Z
	}

	return clamped
}
