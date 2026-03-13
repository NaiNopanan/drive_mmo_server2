package main

import (
	"fmt"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/fixed"
	"server2/internal/geom"
	"server2/internal/sim"
)

func fixedToFloat32(v fixed.Fixed) float32 {
	return float32(float64(v.Raw()) / float64(uint64(1)<<fixed.FracBits))
}

func vecToRL(v geom.Vec3) rl.Vector3 {
	return rl.NewVector3(
		fixedToFloat32(v.X),
		fixedToFloat32(v.Y),
		fixedToFloat32(v.Z),
	)
}

func drawTriangle3D(t geom.Triangle, color rl.Color) {
	a := vecToRL(t.A)
	b := vecToRL(t.B)
	c := vecToRL(t.C)

	rl.DrawLine3D(a, b, color)
	rl.DrawLine3D(b, c, color)
	rl.DrawLine3D(c, a, color)
}

func drawWorld3D(world sim.World) {
	// grid / ground
	rl.DrawGrid(40, 1.0)

	// axis helper
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(3, 0, 0), rl.Red)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 3, 0), rl.Green)
	rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 0, 3), rl.Blue)

	// triangles
	for _, tri := range world.GroundTriangles {
		drawTriangle3D(tri, rl.Purple)
	}

	// body
	px := fixedToFloat32(world.Car.Pos.X)
	py := fixedToFloat32(world.Car.Pos.Y)
	pz := fixedToFloat32(world.Car.Pos.Z)
	radius := fixedToFloat32(world.Car.Radius)

	bodyPos := rl.NewVector3(px, py, pz)

	if world.Car.OnGround {
		rl.DrawSphere(bodyPos, radius, rl.Green)
		rl.DrawSphereWires(bodyPos, radius, 16, 16, rl.DarkGreen)
	} else {
		rl.DrawSphere(bodyPos, radius, rl.Orange)
		rl.DrawSphereWires(bodyPos, radius, 16, 16, rl.Brown)
	}

	// velocity line
	vx := fixedToFloat32(world.Car.Vel.X)
	vy := fixedToFloat32(world.Car.Vel.Y)
	vz := fixedToFloat32(world.Car.Vel.Z)

	velEnd := rl.NewVector3(px+vx*0.25, py+vy*0.25, pz+vz*0.25)
	rl.DrawLine3D(bodyPos, velEnd, rl.Red)

	// contact debug
	if world.LastContact.HasContact {
		cp := vecToRL(world.LastContact.Point)
		cn := vecToRL(world.LastContact.Normal)

		rl.DrawSphere(cp, 0.12, rl.Magenta)

		nEnd := rl.NewVector3(
			cp.X+cn.X*1.5,
			cp.Y+cn.Y*1.5,
			cp.Z+cn.Z*1.5,
		)
		rl.DrawLine3D(cp, nEnd, rl.Magenta)
	}
}

func renderView(target rl.RenderTexture2D, camera rl.Camera3D, world sim.World) {
	rl.BeginTextureMode(target)
	rl.ClearBackground(rl.RayWhite)

	rl.BeginMode3D(camera)
	drawWorld3D(world)
	rl.EndMode3D()

	rl.EndTextureMode()
}

func drawRenderTextureFit(rt rl.RenderTexture2D, dest rl.Rectangle) {
	src := rl.Rectangle{
		X:      0,
		Y:      0,
		Width:  float32(rt.Texture.Width),
		Height: -float32(rt.Texture.Height), // flip Y
	}
	rl.DrawTexturePro(
		rt.Texture,
		src,
		dest,
		rl.Vector2{},
		0,
		rl.White,
	)
}

func drawPanelBorder(rect rl.Rectangle, title string) {
	rl.DrawRectangleLinesEx(rect, 2, rl.DarkGray)
	rl.DrawRectangle(int32(rect.X), int32(rect.Y), int32(rect.Width), 28, rl.Fade(rl.LightGray, 0.8))
	rl.DrawText(title, int32(rect.X)+8, int32(rect.Y)+5, 18, rl.Black)
}

func main() {
	const screenWidth = 1400
	const screenHeight = 900

	rl.InitWindow(screenWidth, screenHeight, "server2-perfect Visual Test Runner")
	defer rl.CloseWindow()

	// Scenario system setup
	scenarios := sim.DefaultScenarios()
	currentScenarioIdx := 0
	runner := sim.NewScenarioRunner(scenarios[currentScenarioIdx])

	// cameras
	perspectiveCam := rl.Camera3D{
		Position:   rl.NewVector3(16, 14, 16),
		Target:     rl.NewVector3(0, 2, 0),
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       45,
		Projection: rl.CameraPerspective,
	}

	sideCam := rl.Camera3D{
		Position:   rl.NewVector3(20, 6, 0),
		Target:     rl.NewVector3(0, 2, 0),
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       20,
		Projection: rl.CameraOrthographic,
	}

	topCam := rl.Camera3D{
		Position:   rl.NewVector3(0, 30, 0),
		Target:     rl.NewVector3(0, 0, 0),
		Up:         rl.NewVector3(0, 0, -1),
		Fovy:       20,
		Projection: rl.CameraOrthographic,
	}

	// render textures for each panel
	rtW := int32(screenWidth / 2)
	rtH := int32(screenHeight / 2)

	perspectiveRT := rl.LoadRenderTexture(rtW, rtH)
	sideRT := rl.LoadRenderTexture(rtW, rtH)
	topRT := rl.LoadRenderTexture(rtW, rtH)
	defer rl.UnloadRenderTexture(perspectiveRT)
	defer rl.UnloadRenderTexture(sideRT)
	defer rl.UnloadRenderTexture(topRT)

	rl.SetTargetFPS(int32(sim.TickHz))

	paused := false
	stepOnce := false

	for !rl.WindowShouldClose() {
		// controls
		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyEnter) || rl.IsKeyPressed(rl.KeyR) {
			runner.Reset()
		}

		// Scenario switching
		if rl.IsKeyPressed(rl.KeyDown) {
			currentScenarioIdx = (currentScenarioIdx + 1) % len(scenarios)
			runner = sim.NewScenarioRunner(scenarios[currentScenarioIdx])
		}
		if rl.IsKeyPressed(rl.KeyUp) {
			currentScenarioIdx = (currentScenarioIdx - 1 + len(scenarios)) % len(scenarios)
			runner = sim.NewScenarioRunner(scenarios[currentScenarioIdx])
		}

		// perspective camera controls
		if rl.IsKeyDown(rl.KeyA) {
			perspectiveCam.Position.X -= 0.20
			perspectiveCam.Target.X -= 0.20
		}
		if rl.IsKeyDown(rl.KeyD) {
			perspectiveCam.Position.X += 0.20
			perspectiveCam.Target.X += 0.20
		}
		if rl.IsKeyDown(rl.KeyW) {
			perspectiveCam.Position.Z -= 0.20
			perspectiveCam.Target.Z -= 0.20
		}
		if rl.IsKeyDown(rl.KeyS) {
			perspectiveCam.Position.Z += 0.20
			perspectiveCam.Target.Z += 0.20
		}

		if !paused || stepOnce {
			runner.Step()
			stepOnce = false
		}

		world := runner.World
		px := fixedToFloat32(world.Car.Pos.X)
		py := fixedToFloat32(world.Car.Pos.Y)
		pz := fixedToFloat32(world.Car.Pos.Z)

		// follow body for side/top cameras
		sideCam.Target = rl.NewVector3(px, py, pz)
		sideCam.Position = rl.NewVector3(px+20, py+6, pz)

		topCam.Target = rl.NewVector3(px, 0, pz)
		topCam.Position = rl.NewVector3(px, 30, pz)

		// render each panel
		renderView(perspectiveRT, perspectiveCam, world)
		renderView(sideRT, sideCam, world)
		renderView(topRT, topCam, world)

		// screen layout
		perspectiveRect := rl.Rectangle{X: 0, Y: 0, Width: float32(screenWidth / 2), Height: float32(screenHeight / 2)}
		topRect := rl.Rectangle{X: float32(screenWidth / 2), Y: 0, Width: float32(screenWidth / 2), Height: float32(screenHeight / 2)}
		sideRect := rl.Rectangle{X: 0, Y: float32(screenHeight / 2), Width: float32(screenWidth / 2), Height: float32(screenHeight / 2)}
		infoRect := rl.Rectangle{X: float32(screenWidth / 2), Y: float32(screenHeight / 2), Width: float32(screenWidth / 2), Height: float32(screenHeight / 2)}

		rl.BeginDrawing()
		rl.ClearBackground(rl.White)

		drawRenderTextureFit(perspectiveRT, perspectiveRect)
		drawRenderTextureFit(topRT, topRect)
		drawRenderTextureFit(sideRT, sideRect)

		drawPanelBorder(perspectiveRect, "Perspective")
		drawPanelBorder(topRect, "Top View")
		drawPanelBorder(sideRect, "Side View")
		drawPanelBorder(infoRect, "Test Dashboard")

		// info panel contents
		rl.DrawRectangleRec(infoRect, rl.Fade(rl.LightGray, 0.05))

		baseX := int32(infoRect.X) + 20
		baseY := int32(infoRect.Y) + 45

		// Scenario Status
		statusText := "RUNNING"
		statusColor := rl.Gold
		switch runner.LastResult.Status {
		case sim.ScenarioPassed:
			statusText = "PASS"
			statusColor = rl.Lime
		case sim.ScenarioFailed:
			statusText = "FAIL"
			statusColor = rl.Red
		}

		rl.DrawText(runner.Scenario.Name, baseX, baseY, 28, rl.DarkBlue)
		rl.DrawRectangle(baseX, baseY+35, 120, 30, statusColor)
		rl.DrawText(statusText, baseX+15, baseY+40, 22, rl.White)
		
		rl.DrawText(fmt.Sprintf("Tick: %d / %d", runner.Tick, runner.Scenario.MaxTicks), baseX+140, baseY+40, 20, rl.DarkGray)
		
		rl.DrawText("Description:", baseX, baseY+80, 18, rl.Gray)
		rl.DrawText(runner.Scenario.Description, baseX, baseY+100, 18, rl.Black)
		
		rl.DrawText("Result:", baseX, baseY+130, 18, rl.Gray)
		rl.DrawText(runner.LastResult.Message, baseX, baseY+150, 18, rl.DarkGreen)

		// Divider
		rl.DrawLine(baseX, baseY+180, baseX+600, baseY+180, rl.LightGray)

		// Stats
		vx := fixedToFloat32(world.Car.Vel.X)
		vy := fixedToFloat32(world.Car.Vel.Y)
		vz := fixedToFloat32(world.Car.Vel.Z)
		
		rl.DrawText(fmt.Sprintf("pos: (%.3f, %.3f, %.3f)", px, py, pz), baseX, baseY+195, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("vel: (%.3f, %.3f, %.3f)", vx, vy, vz), baseX, baseY+225, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("onGround: %v", world.Car.OnGround), baseX, baseY+255, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("hash: %x", sim.HashWorld(world)), baseX, baseY+285, 18, rl.DarkGray)

		// Controls Hint
		rl.DrawText("Toggle: Space (Pause), N (Step)", baseX, baseY+325, 16, rl.Gray)
		rl.DrawText("Scenario: Up/Down arrow keys", baseX, baseY+345, 16, rl.Gray)
		rl.DrawText("Reset: Enter or R", baseX, baseY+365, 16, rl.Gray)
		rl.DrawText("Camera: WASD move, Mouse scroll zoom", baseX, baseY+385, 16, rl.Gray)

		rl.EndDrawing()
	}
}
