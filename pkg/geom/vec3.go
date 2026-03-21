package geom

import "math"

// Vec3 ใช้แทนตำแหน่งหรือเวกเตอร์ 3 มิติสำหรับ static mesh ของโลก
type Vec3 struct {
	X float32
	Y float32
	Z float32
}

func V3(x, y, z float32) Vec3 {
	return Vec3{
		X: x,
		Y: y,
		Z: z,
	}
}

func LerpVec3(from, to Vec3, t float32) Vec3 {
	return Vec3{
		X: from.X + (to.X-from.X)*t,
		Y: from.Y + (to.Y-from.Y)*t,
		Z: from.Z + (to.Z-from.Z)*t,
	}
}

func (v Vec3) Add(other Vec3) Vec3 {
	return Vec3{
		X: v.X + other.X,
		Y: v.Y + other.Y,
		Z: v.Z + other.Z,
	}
}

func (v Vec3) Sub(other Vec3) Vec3 {
	return Vec3{
		X: v.X - other.X,
		Y: v.Y - other.Y,
		Z: v.Z - other.Z,
	}
}

func (v Vec3) MulScalar(scale float32) Vec3 {
	return Vec3{
		X: v.X * scale,
		Y: v.Y * scale,
		Z: v.Z * scale,
	}
}

func (v Vec3) Dot(other Vec3) float32 {
	return v.X*other.X + v.Y*other.Y + v.Z*other.Z
}

func (v Vec3) Cross(other Vec3) Vec3 {
	return Vec3{
		X: v.Y*other.Z - v.Z*other.Y,
		Y: v.Z*other.X - v.X*other.Z,
		Z: v.X*other.Y - v.Y*other.X,
	}
}

func (v Vec3) LengthSquared() float32 {
	return v.Dot(v)
}

func (v Vec3) Normalize() Vec3 {
	lengthSquared := v.LengthSquared()
	if lengthSquared == 0 {
		return Vec3{}
	}

	return v.MulScalar(1 / float32(math.Sqrt(float64(lengthSquared))))
}
