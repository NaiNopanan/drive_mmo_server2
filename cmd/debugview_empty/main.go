package main

import (
	"fmt"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/core"
	"server2/debugview"
	"server2/math/fixed"
	"server2/math/geometry"
	"server2/physics"
	"server2/world"
)

const (
	cameraModePerspectiveFree = 1
	cameraModeSideView        = 2
)

func drawWorldAxes() {
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(3, 0, 0), rl.Red)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 3, 0), rl.Green)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 0, 3), rl.Blue)
}

func setSideViewCamera(camera *rl.Camera3D) {
	if camera == nil {
		return
	}

	camera.Position = rl.NewVector3(18, 8, 0)
	camera.Target = rl.NewVector3(0, 2, 0)
	camera.Up = rl.NewVector3(0, 1, 0)
	camera.Fovy = 45
	camera.Projection = rl.CameraPerspective
}

func buildSandboxScene(simWorld *world.World) {
	if simWorld == nil {
		return
	}

	ground := simWorld.NewEntity()
	simWorld.SetName(ground, world.NameComponent{Value: "ground_plane"})
	simWorld.SetTransform(ground, world.IdentityTransform())
	simWorld.SetBody(ground, physics.NewStaticPlane(
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
	))
	simWorld.SetPrimitiveRender(ground, world.NewPrimitiveRender(world.NewColorRGBA(215, 215, 215, 255)))

	staticBox := simWorld.NewEntity()
	simWorld.SetName(staticBox, world.NameComponent{Value: "static_box"})
	simWorld.SetTransform(staticBox, world.IdentityTransform())
	simWorld.SetBody(staticBox, physics.NewStaticBox(
		geometry.NewVector3(fixed.Zero, fixed.FromInt(1), fixed.Zero),
		geometry.NewVector3(fixed.FromInt(2), fixed.One, fixed.FromInt(2)),
	))
	simWorld.SetPrimitiveRender(staticBox, world.NewPrimitiveRender(world.NewColorRGBA(150, 150, 150, 255)))

	dynamicSphere := simWorld.NewEntity()
	simWorld.SetName(dynamicSphere, world.NameComponent{Value: "dynamic_sphere"})
	simWorld.SetTransform(dynamicSphere, world.IdentityTransform())
	body := physics.NewDynamicSphere(
		fixed.FromInt(1),
		geometry.NewVector3(fixed.FromFraction(-7, 2), fixed.FromInt(6), fixed.FromFraction(3, 2)),
		fixed.FromFraction(4, 5),
	)
	body.Velocity = geometry.NewVector3(fixed.FromFraction(3, 2), fixed.Zero, fixed.FromFraction(1, 2))
	simWorld.SetBody(dynamicSphere, body)
	simWorld.SetPrimitiveRender(dynamicSphere, world.NewPrimitiveRender(world.NewColorRGBA(80, 170, 255, 255)))

	kinematicBox := simWorld.NewEntity()
	simWorld.SetName(kinematicBox, world.NameComponent{Value: "kinematic_box"})
	simWorld.SetTransform(kinematicBox, world.IdentityTransform())
	kinematicBody := physics.NewKinematicBox(
		geometry.NewVector3(fixed.FromInt(6), fixed.FromInt(2), fixed.Zero),
		geometry.NewVector3(fixed.One, fixed.One, fixed.One),
	)
	kinematicBody.KinematicTargetPosition = geometry.NewVector3(fixed.FromInt(6), fixed.FromInt(2), fixed.Zero)
	simWorld.SetBody(kinematicBox, kinematicBody)
	simWorld.SetPrimitiveRender(kinematicBox, world.NewPrimitiveRender(world.NewColorRGBA(245, 160, 60, 255)))
}

func updateKinematicDemo(simWorld *world.World, frame int) {
	if simWorld == nil {
		return
	}

	for id, body := range simWorld.Bodies {
		if body.BodyType != physics.BodyTypeKinematic {
			continue
		}

		phaseIndex := int64((frame / 45) % 4)
		x := fixed.FromInt(6)
		z := fixed.Zero
		switch phaseIndex {
		case 1:
			z = fixed.FromInt(4)
		case 2:
			x = fixed.FromInt(2)
			z = fixed.FromInt(4)
		case 3:
			x = fixed.FromInt(2)
		}

		target := geometry.NewVector3(x, fixed.FromInt(2), z)
		body.KinematicTargetVelocity = target.Sub(body.Position)
		body.KinematicTargetPosition = target
		simWorld.Bodies[id] = body
	}
}

func main() {
	const screenWidth = 1400
	const screenHeight = 900

	rl.InitWindow(screenWidth, screenHeight, "server2 Empty Debug View")
	defer rl.CloseWindow()

	camera := rl.Camera3D{
		Position:   rl.NewVector3(12, 10, 12),
		Target:     rl.NewVector3(0, 1, 0),
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       45,
		Projection: rl.CameraPerspective,
	}

	engine := core.New(core.DefaultConfig())
	renderer := debugview.NewRaylibDebugRenderer(debugview.RaylibDebugRendererConfig{
		PlaneExtent: fixed.FromInt(40),
	})
	buildSandboxScene(engine.World())

	rl.SetTargetFPS(60)
	mouseLocked := true
	cameraMode := cameraModePerspectiveFree
	rl.DisableCursor()
	frameCounter := 0

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeyTab) {
			mouseLocked = !mouseLocked
			if mouseLocked {
				rl.DisableCursor()
			} else {
				rl.EnableCursor()
			}
		}

		if rl.IsKeyPressed(rl.KeyOne) {
			cameraMode = cameraModePerspectiveFree
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			cameraMode = cameraModeSideView
		}

		if cameraMode == cameraModePerspectiveFree {
			rl.UpdateCamera(&camera, rl.CameraFree)
		} else {
			setSideViewCamera(&camera)
		}

		updateKinematicDemo(engine.World(), frameCounter)
		engine.Update(engine.Config.FixedStep)
		frameCounter++

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode3D(camera)
		renderer.DrawWorld(engine.World())
		rl.DrawGrid(80, 1.0)
		drawWorldAxes()
		rl.EndMode3D()

		rl.DrawText("Engine Sandbox Debug View", 30, 30, 28, rl.DarkBlue)
		rl.DrawText("1 perspective free camera / 2 side view", 30, 64, 18, rl.Black)
		rl.DrawText("TAB toggle mouse lock", 30, 88, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Entities: %d", len(engine.World().EntityIDs())), 30, 112, 18, rl.Black)
		rl.DrawText("Primitives: static plane, static box, dynamic sphere, kinematic box", 30, 136, 18, rl.Black)
		if cameraMode == cameraModePerspectiveFree {
			rl.DrawText("Free camera: WASDQE move / mouse look / wheel zoom", 30, 160, 18, rl.Black)
		} else {
			rl.DrawText("Side view: fixed camera from scene right side", 30, 160, 18, rl.Black)
		}
		if !mouseLocked {
			rl.DrawText("Mouse unlocked", 30, 184, 18, rl.Gray)
		} else {
			rl.DrawText("Mouse locked", 30, 184, 18, rl.Gray)
		}
		rl.DrawText("ESC to close", 30, 208, 18, rl.Gray)

		rl.EndDrawing()
	}
}
