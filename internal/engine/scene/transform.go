package scene

import (
	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	enginephysics "server2/internal/engine/physics"
)

// TransformComponent คือ transform ของ object ใน scene/world
type TransformComponent struct {
	Position geometry.Vector3
	Rotation enginephysics.Quaternion
	Scale    geometry.Vector3
}

// IdentityTransform คืน transform เริ่มต้นที่พร้อมใช้งาน
func IdentityTransform() TransformComponent {
	return TransformComponent{
		Rotation: enginephysics.IdentityQuaternion(),
		Scale:    geometry.NewVector3(fixed.One, fixed.One, fixed.One),
	}
}
