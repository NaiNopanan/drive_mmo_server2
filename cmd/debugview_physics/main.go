package main

import (
	"fmt"
	"math"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
	"server2/internal/scenario"
)

type cameraViewMode int

const (
	cameraViewPerspective cameraViewMode = iota
	cameraViewTop
	cameraViewBack
	cameraViewFront
	cameraViewBottom
)

func fixedToFloat32(value fixed.Fixed) float32 {
	return float32(float64(value.Raw()) / float64(uint64(1)<<fixed.FracBits))
}

func vector3ToRaylibVector(value geometry.Vector3) rl.Vector3 {
	return rl.NewVector3(
		fixedToFloat32(value.X),
		fixedToFloat32(value.Y),
		fixedToFloat32(value.Z),
	)
}

func drawWorldAxes() {
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(3, 0, 0), rl.Red)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 3, 0), rl.Green)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 0, 3), rl.Blue)
}

func drawTriangleWireframe(triangle geometry.Triangle, color rl.Color) {
	a := vector3ToRaylibVector(triangle.A)
	b := vector3ToRaylibVector(triangle.B)
	c := vector3ToRaylibVector(triangle.C)

	rl.DrawLine3D(a, b, color)
	rl.DrawLine3D(b, c, color)
	rl.DrawLine3D(c, a, color)
}

func drawGroundTriangles(triangles []geometry.Triangle) {
	for _, triangle := range triangles {
		drawTriangleWireframe(triangle, rl.Purple)
	}
}

func drawSphereBody(body physics.SphereBody) {
	position := vector3ToRaylibVector(body.Motion.Position)
	radius := fixedToFloat32(body.Radius)
	color := rl.Orange
	wireColor := rl.Brown
	if body.Grounded {
		color = rl.Green
		wireColor = rl.DarkGreen
	}

	rl.DrawSphere(position, radius, color)
	rl.DrawSphereWires(position, radius, 16, 16, wireColor)

	velocityEnd := rl.NewVector3(
		position.X+fixedToFloat32(body.Motion.Velocity.X)*0.25,
		position.Y+fixedToFloat32(body.Motion.Velocity.Y)*0.25,
		position.Z+fixedToFloat32(body.Motion.Velocity.Z)*0.25,
	)
	rl.DrawLine3D(position, velocityEnd, rl.Red)
}

func drawSphereBodies(bodies []physics.SphereBody) {
	for _, body := range bodies {
		drawSphereBody(body)
	}
}

func drawContactDebug(contact physics.SphereTriangleContact) {
	if !contact.Hit {
		return
	}

	point := vector3ToRaylibVector(contact.Point)
	normalEnd := rl.NewVector3(
		point.X+fixedToFloat32(contact.Normal.X)*1.5,
		point.Y+fixedToFloat32(contact.Normal.Y)*1.5,
		point.Z+fixedToFloat32(contact.Normal.Z)*1.5,
	)

	rl.DrawSphere(point, 0.12, rl.Magenta)
	rl.DrawLine3D(point, normalEnd, rl.Magenta)
}

func drawContactDebugList(contacts []physics.SphereTriangleContact) {
	for _, contact := range contacts {
		drawContactDebug(contact)
	}
}

func sceneSpheres(state scenario.SceneState) []physics.SphereBody {
	if len(state.Spheres) > 0 {
		return state.Spheres
	}
	return []physics.SphereBody{state.Sphere}
}

func sceneContacts(state scenario.SceneState) []physics.SphereTriangleContact {
	if len(state.LastContacts) > 0 {
		return state.LastContacts
	}
	return []physics.SphereTriangleContact{state.LastContact}
}

func moveCameraPlanar(camera *rl.Camera3D, dx, dz float32) {
	camera.Position.X += dx
	camera.Position.Z += dz
	camera.Target.X += dx
	camera.Target.Z += dz
}

func zoomCamera(camera *rl.Camera3D, amount float32) {
	dx := camera.Target.X - camera.Position.X
	dy := camera.Target.Y - camera.Position.Y
	dz := camera.Target.Z - camera.Position.Z

	length := float32(dx*dx + dy*dy + dz*dz)
	if length == 0 {
		return
	}

	length = float32(math.Sqrt(float64(length)))
	if length == 0 {
		return
	}

	scale := amount / length
	camera.Position.X += dx * scale
	camera.Position.Y += dy * scale
	camera.Position.Z += dz * scale
}

func cameraViewLabel(mode cameraViewMode) string {
	switch mode {
	case cameraViewTop:
		return "Top"
	case cameraViewBack:
		return "Back"
	case cameraViewFront:
		return "Front"
	case cameraViewBottom:
		return "Bottom"
	default:
		return "Perspective"
	}
}

func updateViewModeFromKeyboard(current cameraViewMode) cameraViewMode {
	switch {
	case rl.IsKeyPressed(rl.KeyOne):
		return cameraViewPerspective
	case rl.IsKeyPressed(rl.KeyTwo):
		return cameraViewTop
	case rl.IsKeyPressed(rl.KeyThree):
		return cameraViewBack
	case rl.IsKeyPressed(rl.KeyFour):
		return cameraViewFront
	case rl.IsKeyPressed(rl.KeyFive):
		return cameraViewBottom
	default:
		return current
	}
}

func setCameraFromViewMode(camera *rl.Camera3D, mode cameraViewMode, distance float32, focus rl.Vector3) {
	if distance < 2 {
		distance = 2
	}

	camera.Target = focus
	camera.Projection = rl.CameraPerspective
	camera.Fovy = 45

	switch mode {
	case cameraViewTop:
		camera.Position = rl.NewVector3(focus.X, focus.Y+distance, focus.Z)
		camera.Up = rl.NewVector3(0, 0, -1)
	case cameraViewBack:
		camera.Position = rl.NewVector3(focus.X, focus.Y+distance*0.35, focus.Z+distance)
		camera.Up = rl.NewVector3(0, 1, 0)
	case cameraViewFront:
		camera.Position = rl.NewVector3(focus.X, focus.Y+distance*0.35, focus.Z-distance)
		camera.Up = rl.NewVector3(0, 1, 0)
	case cameraViewBottom:
		camera.Position = rl.NewVector3(focus.X, focus.Y-distance, focus.Z)
		camera.Up = rl.NewVector3(0, 0, 1)
	default:
		camera.Position = rl.NewVector3(focus.X+distance, focus.Y+distance*0.7, focus.Z+distance)
		camera.Up = rl.NewVector3(0, 1, 0)
	}
}

func updateCamera(camera *rl.Camera3D, mode cameraViewMode, distance *float32, focus rl.Vector3) {
	if distance == nil {
		return
	}

	scroll := rl.GetMouseWheelMove()
	if scroll != 0 {
		*distance -= scroll * 1.5
	}
	if *distance < 2 {
		*distance = 2
	}

	if mode == cameraViewPerspective {
		const moveSpeed float32 = 0.25
		if rl.IsKeyDown(rl.KeyW) {
			moveCameraPlanar(camera, 0, -moveSpeed)
		}
		if rl.IsKeyDown(rl.KeyS) {
			moveCameraPlanar(camera, 0, moveSpeed)
		}
		if rl.IsKeyDown(rl.KeyA) {
			moveCameraPlanar(camera, -moveSpeed, 0)
		}
		if rl.IsKeyDown(rl.KeyD) {
			moveCameraPlanar(camera, moveSpeed, 0)
		}
		if scroll != 0 {
			zoomCamera(camera, scroll)
		}
		return
	}

	setCameraFromViewMode(camera, mode, *distance, focus)
}

func statusLabel(status scenario.ScenarioResultStatus) string {
	switch status {
	case scenario.Passed:
		return "PASS"
	case scenario.Failed:
		return "FAIL"
	default:
		return "RUNNING"
	}
}

func statusColor(status scenario.ScenarioResultStatus) rl.Color {
	switch status {
	case scenario.Passed:
		return rl.Lime
	case scenario.Failed:
		return rl.Red
	default:
		return rl.Gold
	}
}

func drawOverlayWithCameraMode(definition scenario.ScenarioDefinition, runner *scenario.ScenarioRunner, viewMode cameraViewMode) {
	state := runner.State
	spheres := sceneSpheres(state)
	sphere := spheres[0]
	contactNormal := state.LastContact.Normal
	sceneHash := scenario.HashSceneState(state)

	rl.DrawRectangle(18, 18, 560, 320, rl.Fade(rl.RayWhite, 0.92))
	rl.DrawRectangleLinesEx(rl.NewRectangle(18, 18, 560, 320), 2, rl.DarkGray)

	rl.DrawText(definition.Name, 30, 30, 28, rl.DarkBlue)
	rl.DrawText(definition.Description, 30, 66, 18, rl.Black)

	status := statusLabel(runner.LastResult.Status)
	color := statusColor(runner.LastResult.Status)
	rl.DrawRectangle(30, 96, 110, 28, color)
	rl.DrawText(status, 54, 101, 18, rl.White)
	rl.DrawText(fmt.Sprintf("Tick: %d / %d", runner.Tick, definition.MaxTicks), 156, 101, 20, rl.DarkGray)

	rl.DrawText(fmt.Sprintf("Result: %s", runner.LastResult.Message), 30, 138, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Position: (%.3f, %.3f, %.3f)",
		fixedToFloat32(sphere.Motion.Position.X),
		fixedToFloat32(sphere.Motion.Position.Y),
		fixedToFloat32(sphere.Motion.Position.Z),
	), 30, 174, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Velocity: (%.3f, %.3f, %.3f)",
		fixedToFloat32(sphere.Motion.Velocity.X),
		fixedToFloat32(sphere.Motion.Velocity.Y),
		fixedToFloat32(sphere.Motion.Velocity.Z),
	), 30, 202, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Grounded: %v | Ever touched ground: %v", sphere.Grounded, state.EverTouchedGround), 30, 230, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Sphere count: %d", len(spheres)), 30, 258, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Contact normal: (%.3f, %.3f, %.3f)",
		fixedToFloat32(contactNormal.X),
		fixedToFloat32(contactNormal.Y),
		fixedToFloat32(contactNormal.Z),
	), 30, 286, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("View: %s", cameraViewLabel(viewMode)), 420, 101, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Hash: %016x", sceneHash), 30, 314, 18, rl.Black)
	rl.DrawText("Controls: Space pause | N step | R reset | Left/Right scene | 1-5 view | Wheel zoom | WASD move in perspective", 30, 342, 16, rl.Gray)
}

func main() {
	const screenWidth = 1400
	const screenHeight = 900

	rl.InitWindow(screenWidth, screenHeight, "server2 Physics Debug View")
	defer rl.CloseWindow()

	rl.SetTargetFPS(int32(physics.DefaultStepRateHz))

	definitions := scenario.DefaultScenarioDefinitions()
	currentIndex := 0
	runner := scenario.NewScenarioRunner(definitions[currentIndex])
	paused := false
	stepOnce := false
	viewMode := cameraViewPerspective
	viewDistance := float32(16)

	camera := rl.Camera3D{
		Position:   rl.NewVector3(16, 14, 16),
		Target:     rl.NewVector3(0, 2, 0),
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       45,
		Projection: rl.CameraPerspective,
	}

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyR) {
			runner.Reset()
		}
		if rl.IsKeyPressed(rl.KeyRight) {
			currentIndex = (currentIndex + 1) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
		}
		if rl.IsKeyPressed(rl.KeyLeft) {
			currentIndex = (currentIndex - 1 + len(definitions)) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
		}

		previousViewMode := viewMode
		viewMode = updateViewModeFromKeyboard(viewMode)
		spheres := sceneSpheres(runner.State)
		focus := rl.NewVector3(
			fixedToFloat32(spheres[0].Motion.Position.X),
			fixedToFloat32(spheres[0].Motion.Position.Y),
			fixedToFloat32(spheres[0].Motion.Position.Z),
		)

		if focus.Y < 1 {
			focus.Y = 1
		}
		focus.X = 0
		focus.Z = 0

		if viewMode != previousViewMode {
			setCameraFromViewMode(&camera, viewMode, viewDistance, focus)
		} else if viewMode != cameraViewPerspective {
			setCameraFromViewMode(&camera, viewMode, viewDistance, focus)
		}

		updateCamera(&camera, viewMode, &viewDistance, focus)

		if !paused || stepOnce {
			runner.Step()
			stepOnce = false
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode3D(camera)
		rl.DrawGrid(40, 1.0)
		drawWorldAxes()
		drawGroundTriangles(runner.State.GroundTriangles)
		drawSphereBodies(sceneSpheres(runner.State))
		drawContactDebugList(sceneContacts(runner.State))
		rl.EndMode3D()

		drawOverlayWithCameraMode(definitions[currentIndex], runner, viewMode)

		rl.EndDrawing()
	}
}
