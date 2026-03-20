package physics

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/primitive"
)

// BodyComponent คือ state ของ physics body ใน ECS world
// v1 ตั้งใจให้เป็นข้อมูลกลางสำหรับ body ทุกชนิดก่อน ยังไม่ทำ collision system เต็ม
type BodyComponent struct {
	BodyType    BodyType
	ShapeType   ShapeType
	Position    geometry.Vector3
	Orientation Quaternion

	Velocity         geometry.Vector3
	AngularVelocity  geometry.Vector3
	AccumulatedForce geometry.Vector3

	Mass        fixed.Fixed
	InverseMass fixed.Fixed
	Restitution fixed.Fixed
	Friction    fixed.Fixed

	Plane  primitive.PlaneShape
	Box    primitive.BoxShape
	Sphere primitive.SphereShape

	// KinematicTargetPosition และ KinematicTargetVelocity
	// ใช้โดย KinematicSystem เพื่อขยับ body แบบควบคุมจาก gameplay code
	KinematicTargetPosition geometry.Vector3
	KinematicTargetVelocity geometry.Vector3
}
