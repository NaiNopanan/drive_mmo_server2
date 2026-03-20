package physics

import (
	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
)

// Shape3D คือ interface ขั้นต่ำของ shape ใน engine ใหม่
type Shape3D interface {
	ShapeType() ShapeType
}

// PlaneShape คือ primitive แบบ plane ไม่สิ้นสุด ใช้เป็นพื้น/debug ได้ดี
type PlaneShape struct {
	Normal geometry.Vector3
}

func (PlaneShape) ShapeType() ShapeType { return ShapeTypePlane }

// BoxShape คือ primitive กล่องแบบครึ่งขนาด
type BoxShape struct {
	HalfExtents geometry.Vector3
}

func (BoxShape) ShapeType() ShapeType { return ShapeTypeBox }

// SphereShape คือ primitive ทรงกลม
type SphereShape struct {
	Radius fixed.Fixed
}

func (SphereShape) ShapeType() ShapeType { return ShapeTypeSphere }
