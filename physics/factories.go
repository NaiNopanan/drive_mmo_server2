package physics

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/primitive"
)

// NewStaticPlane สร้าง plane ที่ไม่ขยับ ใช้เป็นพื้นหรือฉากคงที่
func NewStaticPlane(position, normal geometry.Vector3) BodyComponent {
	return BodyComponent{
		BodyType:    BodyTypeStatic,
		ShapeType:   ShapeTypePlane,
		Position:    position,
		Orientation: IdentityQuaternion(),
		Plane: primitive.PlaneShape{
			Normal: sanitizePlaneNormal(normal),
		},
	}
}

// NewStaticBox สร้างกล่องคงที่
func NewStaticBox(position, halfExtents geometry.Vector3) BodyComponent {
	validateHalfExtents(halfExtents)
	return BodyComponent{
		BodyType:    BodyTypeStatic,
		ShapeType:   ShapeTypeBox,
		Position:    position,
		Orientation: IdentityQuaternion(),
		Box: primitive.BoxShape{
			HalfExtents: halfExtents,
		},
	}
}

// NewDynamicBox สร้างกล่อง dynamic ที่ตอบสนองต่อแรงและการ integrate
func NewDynamicBox(mass fixed.Fixed, position, halfExtents geometry.Vector3) BodyComponent {
	validateDynamicMass(mass)
	validateHalfExtents(halfExtents)
	return BodyComponent{
		BodyType:    BodyTypeDynamic,
		ShapeType:   ShapeTypeBox,
		Position:    position,
		Orientation: IdentityQuaternion(),
		Mass:        mass,
		InverseMass: fixed.One.Div(mass),
		Box: primitive.BoxShape{
			HalfExtents: halfExtents,
		},
	}
}

// NewDynamicSphere สร้าง sphere dynamic
func NewDynamicSphere(mass fixed.Fixed, position geometry.Vector3, radius fixed.Fixed) BodyComponent {
	validateDynamicMass(mass)
	validateRadius(radius)
	return BodyComponent{
		BodyType:    BodyTypeDynamic,
		ShapeType:   ShapeTypeSphere,
		Position:    position,
		Orientation: IdentityQuaternion(),
		Mass:        mass,
		InverseMass: fixed.One.Div(mass),
		Sphere: primitive.SphereShape{
			Radius: radius,
		},
	}
}

// NewKinematicBox สร้างกล่อง kinematic ที่ขยับด้วย target state จากระบบเกม
func NewKinematicBox(position, halfExtents geometry.Vector3) BodyComponent {
	validateHalfExtents(halfExtents)
	return BodyComponent{
		BodyType:    BodyTypeKinematic,
		ShapeType:   ShapeTypeBox,
		Position:    position,
		Orientation: IdentityQuaternion(),
		Box: primitive.BoxShape{
			HalfExtents: halfExtents,
		},
		KinematicTargetPosition: position,
	}
}
