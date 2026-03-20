package render

import (
	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/base/fixed"
	"server2/internal/base/geometry"
	"server2/internal/engine/ecs"
	enginephysics "server2/internal/engine/physics"
	"server2/internal/engine/scene"
)

// RaylibDebugRendererConfig คือค่าตั้งต้นของ debug renderer
type RaylibDebugRendererConfig struct {
	PlaneExtent fixed.Fixed
}

// RaylibDebugRenderer วาด primitive จาก ECS world ด้วย raylib
type RaylibDebugRenderer struct {
	Config RaylibDebugRendererConfig
}

// NewRaylibDebugRenderer สร้าง renderer ใหม่พร้อมค่าเริ่มต้น
func NewRaylibDebugRenderer(config RaylibDebugRendererConfig) *RaylibDebugRenderer {
	if config.PlaneExtent.Cmp(fixed.Zero) <= 0 {
		config.PlaneExtent = fixed.FromInt(40)
	}
	return &RaylibDebugRenderer{Config: config}
}

// DrawWorld วาด object ทั้งหมดที่มี body และ primitive render component
func (r *RaylibDebugRenderer) DrawWorld(world *ecs.World) {
	if r == nil || world == nil {
		return
	}

	for _, id := range world.EntityIDs() {
		body, hasBody := world.Bodies[id]
		renderComponent, hasRender := world.PrimitiveRenders[id]
		if !hasBody || !hasRender || !renderComponent.Visible {
			continue
		}

		switch body.ShapeType {
		case enginephysics.ShapeTypePlane:
			r.drawPlane(body, renderComponent)
		case enginephysics.ShapeTypeBox:
			r.drawBox(body, renderComponent)
		case enginephysics.ShapeTypeSphere:
			r.drawSphere(body, renderComponent)
		}
	}
}

func (r *RaylibDebugRenderer) drawPlane(body enginephysics.BodyComponent, renderComponent scene.PrimitiveRenderComponent) {
	normal := body.Plane.Normal.Normalize()
	if normal.LengthSquared() == fixed.Zero {
		normal = geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	}

	helper := geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero)
	if normal.Dot(helper).Abs().Cmp(fixed.FromFraction(19, 20)) > 0 {
		helper = geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero)
	}

	tangentA := normal.Cross(helper).Normalize()
	tangentB := normal.Cross(tangentA).Normalize()
	extent := r.Config.PlaneExtent
	cornerA := body.Position.Add(tangentA.Scale(extent)).Add(tangentB.Scale(extent))
	cornerB := body.Position.Add(tangentA.Scale(extent)).Sub(tangentB.Scale(extent))
	cornerC := body.Position.Sub(tangentA.Scale(extent)).Sub(tangentB.Scale(extent))
	cornerD := body.Position.Sub(tangentA.Scale(extent)).Add(tangentB.Scale(extent))

	fill := toRaylibColor(renderComponent.Color)
	wire := darkenColor(fill, 35)
	rl.DrawTriangle3D(toRaylibVector(cornerA), toRaylibVector(cornerB), toRaylibVector(cornerC), rl.Fade(fill, 0.75))
	rl.DrawTriangle3D(toRaylibVector(cornerA), toRaylibVector(cornerC), toRaylibVector(cornerD), rl.Fade(fill, 0.75))
	rl.DrawLine3D(toRaylibVector(cornerA), toRaylibVector(cornerB), wire)
	rl.DrawLine3D(toRaylibVector(cornerB), toRaylibVector(cornerC), wire)
	rl.DrawLine3D(toRaylibVector(cornerC), toRaylibVector(cornerD), wire)
	rl.DrawLine3D(toRaylibVector(cornerD), toRaylibVector(cornerA), wire)
}

func (r *RaylibDebugRenderer) drawBox(body enginephysics.BodyComponent, renderComponent scene.PrimitiveRenderComponent) {
	halfExtents := body.Box.HalfExtents
	corners := []geometry.Vector3{
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y.Neg(), halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X, halfExtents.Y.Neg(), halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X, halfExtents.Y, halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y, halfExtents.Z.Neg()),
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y.Neg(), halfExtents.Z),
		geometry.NewVector3(halfExtents.X, halfExtents.Y.Neg(), halfExtents.Z),
		geometry.NewVector3(halfExtents.X, halfExtents.Y, halfExtents.Z),
		geometry.NewVector3(halfExtents.X.Neg(), halfExtents.Y, halfExtents.Z),
	}

	worldCorners := make([]rl.Vector3, 0, len(corners))
	for _, corner := range corners {
		worldCorner := body.Position.Add(body.Orientation.RotateVector(corner))
		worldCorners = append(worldCorners, toRaylibVector(worldCorner))
	}

	fill := toRaylibColor(renderComponent.Color)
	wire := darkenColor(fill, 45)
	faces := [][4]int{
		{0, 1, 2, 3},
		{4, 5, 6, 7},
		{0, 1, 5, 4},
		{1, 2, 6, 5},
		{2, 3, 7, 6},
		{3, 0, 4, 7},
	}
	for _, face := range faces {
		rl.DrawTriangle3D(worldCorners[face[0]], worldCorners[face[1]], worldCorners[face[2]], rl.Fade(fill, 0.85))
		rl.DrawTriangle3D(worldCorners[face[0]], worldCorners[face[2]], worldCorners[face[3]], rl.Fade(fill, 0.85))
	}

	if renderComponent.DrawWire || true {
		edges := [][2]int{
			{0, 1}, {1, 2}, {2, 3}, {3, 0},
			{4, 5}, {5, 6}, {6, 7}, {7, 4},
			{0, 4}, {1, 5}, {2, 6}, {3, 7},
		}
		for _, edge := range edges {
			rl.DrawLine3D(worldCorners[edge[0]], worldCorners[edge[1]], wire)
		}
	}
}

func (r *RaylibDebugRenderer) drawSphere(body enginephysics.BodyComponent, renderComponent scene.PrimitiveRenderComponent) {
	fill := toRaylibColor(renderComponent.Color)
	position := toRaylibVector(body.Position)
	radius := fixedToFloat32(body.Sphere.Radius)
	rl.DrawSphere(position, radius, rl.Fade(fill, 0.9))
	if renderComponent.DrawWire || true {
		rl.DrawSphereWires(position, radius, 10, 10, darkenColor(fill, 45))
	}
}

func fixedToFloat32(value fixed.Fixed) float32 {
	return float32(float64(value.Raw()) / float64(uint64(1)<<fixed.FracBits))
}

func toRaylibVector(value geometry.Vector3) rl.Vector3 {
	return rl.NewVector3(
		fixedToFloat32(value.X),
		fixedToFloat32(value.Y),
		fixedToFloat32(value.Z),
	)
}

func toRaylibColor(color scene.Color) rl.Color {
	return rl.NewColor(color.R, color.G, color.B, color.A)
}

func darkenColor(color rl.Color, amount uint8) rl.Color {
	darken := func(v uint8) uint8 {
		if v <= amount {
			return 0
		}
		return v - amount
	}
	return rl.NewColor(darken(color.R), darken(color.G), darken(color.B), color.A)
}
