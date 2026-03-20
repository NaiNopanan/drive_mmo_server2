package core

import (
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/physics"
	"server2/world"
)

// EngineConfig คือค่าตั้งต้นของ engine runtime
type EngineConfig struct {
	FixedStep fixed.Fixed
	Gravity   geometry.Vector3
}

// DefaultConfig คืนค่า config เริ่มต้นของ engine ใหม่
func DefaultConfig() EngineConfig {
	return EngineConfig{
		FixedStep: fixed.FromFraction(1, 60),
		Gravity:   physics.StandardGravity,
	}
}

// Engine คือ runtime owner หลักของ ECS world และระบบต่าง ๆ
type Engine struct {
	Config      EngineConfig
	world       *world.World
	accumulator fixed.Fixed
}

// New สร้าง engine ใหม่พร้อม register systems ตามลำดับมาตรฐานของ v1
func New(config EngineConfig) *Engine {
	defaults := DefaultConfig()
	if config.FixedStep.Cmp(fixed.Zero) <= 0 {
		config.FixedStep = defaults.FixedStep
	}

	simWorld := world.NewWorld(world.WorldConfig{})
	simWorld.AddSystem(KinematicSystem{})
	simWorld.AddSystem(PhysicsIntegrateSystem{Gravity: config.Gravity})
	simWorld.AddSystem(TransformSyncSystem{})
	simWorld.AddSystem(DebugRenderPrepSystem{})

	return &Engine{
		Config: config,
		world:  simWorld,
	}
}

// Update เดิน engine ไปข้างหน้าตาม dt ที่รับเข้ามา
// ใช้ accumulator เพื่อให้ world ทำงานที่ fixed step คงที่
func (e *Engine) Update(dt fixed.Fixed) {
	if e == nil || dt.Cmp(fixed.Zero) <= 0 {
		return
	}

	e.accumulator = e.accumulator.Add(dt)
	for e.accumulator.Cmp(e.Config.FixedStep) >= 0 {
		e.world.Step(e.Config.FixedStep)
		e.accumulator = e.accumulator.Sub(e.Config.FixedStep)
	}
}

// World คืน ECS world ของ engine นี้
func (e *Engine) World() *world.World {
	if e == nil {
		return nil
	}
	return e.world
}
