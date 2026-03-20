package physics

import (
	"server2/math/fixed"
	"server2/math/geometry"
)

// BodyType บอกว่า body ถูก simulate แบบไหน
type BodyType int

const (
	BodyTypeStatic BodyType = iota
	BodyTypeDynamic
	BodyTypeKinematic
)

// ShapeType บอกว่ารูปร่างหลักของ body คืออะไร
type ShapeType int

const (
	ShapeTypePlane ShapeType = iota
	ShapeTypeBox
	ShapeTypeSphere
)

// StandardGravity เป็น gravity เริ่มต้นของ engine ใหม่
var StandardGravity = geometry.NewVector3(fixed.Zero, fixed.FromFraction(-981, 100), fixed.Zero)
