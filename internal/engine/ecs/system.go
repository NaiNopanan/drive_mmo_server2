package ecs

import "server2/internal/base/fixed"

// WorldConfig เผื่อพื้นที่สำหรับ config ของ ECS world ในอนาคต
type WorldConfig struct{}

// System คือ interface ขั้นต่ำของระบบที่ถูกเรียกทุก tick
type System interface {
	Step(w *World, dt fixed.Fixed)
}
