package primitive

import (
	"server2/math/fixed"
	"server2/math/geometry"
)

// PlaneShape คือ primitive แบบ plane ไม่สิ้นสุด ใช้เป็นพื้น/debug ได้ดี
type PlaneShape struct {
	Normal geometry.Vector3
}

// BoxShape คือ primitive กล่องแบบครึ่งขนาด
type BoxShape struct {
	HalfExtents geometry.Vector3
}

// SphereShape คือ primitive ทรงกลม
type SphereShape struct {
	Radius fixed.Fixed
}
