package physics

import "math"

func float32Sqrt(value float32) float32 {
	return float32(math.Sqrt(float64(value)))
}

func lerpFloat32(from, to, t float32) float32 {
	return from + (to-from)*t
}

func lerpAngle(from, to, t float32) float32 {
	return normalizeAngle(from + normalizeAngle(to-from)*t)
}
