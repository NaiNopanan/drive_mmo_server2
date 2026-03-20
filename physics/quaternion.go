package physics

import (
	"server2/math/fixed"
	"server2/math/geometry"
)

// Quaternion ใช้เก็บการหมุนของ object ใน engine ใหม่
// v1 ยังใช้ไม่หนักมาก แต่ใส่ไว้ตั้งแต่ต้นเพื่อให้ body/transform มีหน้าตาแบบ engine จริง
type Quaternion struct {
	W fixed.Fixed
	X fixed.Fixed
	Y fixed.Fixed
	Z fixed.Fixed
}

// IdentityQuaternion คืนค่า quaternion ที่ไม่หมุนอะไรเลย
func IdentityQuaternion() Quaternion {
	return Quaternion{W: fixed.One}
}

// Normalize ทำให้ quaternion มีขนาดเป็น 1 เพื่อใช้เป็น rotation ได้อย่างถูกต้อง
func (q Quaternion) Normalize() Quaternion {
	lengthSq := q.W.Mul(q.W).Add(q.X.Mul(q.X)).Add(q.Y.Mul(q.Y)).Add(q.Z.Mul(q.Z))
	if lengthSq == fixed.Zero {
		return IdentityQuaternion()
	}
	invLength := fixed.One.Div(lengthSq.Sqrt())
	return Quaternion{
		W: q.W.Mul(invLength),
		X: q.X.Mul(invLength),
		Y: q.Y.Mul(invLength),
		Z: q.Z.Mul(invLength),
	}
}

// Conjugate คืนค่า quaternion กลับทิศ ใช้กับการหมุน vector
func (q Quaternion) Conjugate() Quaternion {
	return Quaternion{
		W: q.W,
		X: q.X.Neg(),
		Y: q.Y.Neg(),
		Z: q.Z.Neg(),
	}
}

// Mul คูณ quaternion สองตัวเข้าด้วยกัน
func (q Quaternion) Mul(o Quaternion) Quaternion {
	return Quaternion{
		W: q.W.Mul(o.W).Sub(q.X.Mul(o.X)).Sub(q.Y.Mul(o.Y)).Sub(q.Z.Mul(o.Z)),
		X: q.W.Mul(o.X).Add(q.X.Mul(o.W)).Add(q.Y.Mul(o.Z)).Sub(q.Z.Mul(o.Y)),
		Y: q.W.Mul(o.Y).Sub(q.X.Mul(o.Z)).Add(q.Y.Mul(o.W)).Add(q.Z.Mul(o.X)),
		Z: q.W.Mul(o.Z).Add(q.X.Mul(o.Y)).Sub(q.Y.Mul(o.X)).Add(q.Z.Mul(o.W)),
	}
}

// RotateVector หมุน vector ด้วย quaternion นี้
func (q Quaternion) RotateVector(v geometry.Vector3) geometry.Vector3 {
	p := Quaternion{X: v.X, Y: v.Y, Z: v.Z}
	rotated := q.Mul(p).Mul(q.Conjugate())
	return geometry.NewVector3(rotated.X, rotated.Y, rotated.Z)
}
