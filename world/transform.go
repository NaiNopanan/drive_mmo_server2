package world

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/physics"
)

// TransformComponent คือ transform ของ object ใน scene/world
type TransformComponent struct {
	Position geometry.Vector3
	Rotation physics.Quaternion
	Scale    geometry.Vector3
}

// IdentityTransform คืน transform เริ่มต้นที่พร้อมใช้งาน
func IdentityTransform() TransformComponent {
	return TransformComponent{
		Rotation: physics.IdentityQuaternion(),
		Scale:    geometry.NewVector3(fixed.One, fixed.One, fixed.One),
	}
}
