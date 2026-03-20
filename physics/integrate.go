package physics

import (
	"server2/math/fixed"
	"server2/math/geometry"
)

// ApplyForce ใส่แรงสะสมให้ body dynamic สำหรับใช้ใน tick ปัจจุบัน
func ApplyForce(body *BodyComponent, force geometry.Vector3) {
	if body == nil || body.BodyType != BodyTypeDynamic || body.InverseMass == fixed.Zero {
		return
	}
	body.AccumulatedForce = body.AccumulatedForce.Add(force)
}

// IntegrateBody เดิน body ไปข้างหน้า 1 tick
// v1 ทำเฉพาะ linear integration สำหรับ body dynamic
func IntegrateBody(body *BodyComponent, dt fixed.Fixed, gravity geometry.Vector3) {
	if body == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}
	if body.BodyType != BodyTypeDynamic || body.InverseMass == fixed.Zero {
		body.AccumulatedForce = geometry.ZeroVector3()
		return
	}

	totalForce := body.AccumulatedForce.Add(gravity.Scale(body.Mass))
	acceleration := totalForce.Scale(body.InverseMass)
	body.Velocity = body.Velocity.Add(acceleration.Scale(dt))
	body.Position = body.Position.Add(body.Velocity.Scale(dt))
	body.AccumulatedForce = geometry.ZeroVector3()
}

func sanitizePlaneNormal(normal geometry.Vector3) geometry.Vector3 {
	normalized := normal.Normalize()
	if normalized.LengthSquared() == fixed.Zero {
		return geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}
	return normalized
}

func validateHalfExtents(halfExtents geometry.Vector3) {
	if halfExtents.X.Cmp(fixed.Zero) <= 0 || halfExtents.Y.Cmp(fixed.Zero) <= 0 || halfExtents.Z.Cmp(fixed.Zero) <= 0 {
		panic("physics: box half extents must be > 0")
	}
}

func validateRadius(radius fixed.Fixed) {
	if radius.Cmp(fixed.Zero) <= 0 {
		panic("physics: sphere radius must be > 0")
	}
}

func validateDynamicMass(mass fixed.Fixed) {
	if mass.Cmp(fixed.Zero) <= 0 {
		panic("physics: dynamic mass must be > 0")
	}
}
