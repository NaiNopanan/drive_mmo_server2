package main

import (
	"fmt"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/fixed"
	"server2/internal/sim"
)

func fixedToFloat32(v fixed.Fixed) float32 {
	// Q32.32 -> float32
	return float32(float64(v.Raw()) / float64(uint64(1)<<fixed.FracBits))
}

func main() {
	const screenWidth = 1280
	const screenHeight = 720

	rl.InitWindow(screenWidth, screenHeight, "server2-perfect debug viewer")
	defer rl.CloseWindow()

	camera := rl.Camera3D{
		Position:   rl.NewVector3(18, 18, 18),
		Target:     rl.NewVector3(0, 3, 0),
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       45,
		Projection: rl.CameraPerspective,
	}

	rl.SetTargetFPS(int32(sim.TickHz))

	world := sim.NewWorld()

	paused := false
	stepOnce := false

	for !rl.WindowShouldClose() {
		// camera control
		if rl.IsKeyDown(rl.KeyA) {
			camera.Position.X -= 0.15
			camera.Target.X -= 0.15
		}
		if rl.IsKeyDown(rl.KeyD) {
			camera.Position.X += 0.15
			camera.Target.X += 0.15
		}
		if rl.IsKeyDown(rl.KeyW) {
			camera.Position.Z -= 0.15
			camera.Target.Z -= 0.15
		}
		if rl.IsKeyDown(rl.KeyS) {
			camera.Position.Z += 0.15
			camera.Target.Z += 0.15
		}

		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyR) {
			world = sim.NewWorld()
		}

		// simple input -> sim.Input
		var in sim.Input
		if rl.IsKeyDown(rl.KeyUp) {
			in.Throttle = 1
		}
		if rl.IsKeyDown(rl.KeyDown) {
			in.Brake = 1
		}
		if rl.IsKeyDown(rl.KeyLeft) {
			in.Left = 1
		}
		if rl.IsKeyDown(rl.KeyRight) {
			in.Right = 1
		}

		// run sim
		if !paused || stepOnce {
			sim.Step(&world, in)
			stepOnce = false
		}

		// convert sim state -> render state
		px := fixedToFloat32(world.Car.Pos.X)
		py := fixedToFloat32(world.Car.Pos.Y)
		pz := fixedToFloat32(world.Car.Pos.Z)
		radius := fixedToFloat32(world.Car.Radius)

		vx := fixedToFloat32(world.Car.Vel.X)
		vy := fixedToFloat32(world.Car.Vel.Y)
		vz := fixedToFloat32(world.Car.Vel.Z)

		bodyPos := rl.NewVector3(px, py, pz)
		velEnd := rl.NewVector3(px+vx*0.25, py+vy*0.25, pz+vz*0.25)

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode3D(camera)

		// ground
		rl.DrawGrid(40, 1.0)

		// body
		if world.Car.OnGround {
			rl.DrawSphere(bodyPos, radius, rl.Green)
			rl.DrawSphereWires(bodyPos, radius, 16, 16, rl.DarkGreen)
		} else {
			rl.DrawSphere(bodyPos, radius, rl.Orange)
			rl.DrawSphereWires(bodyPos, radius, 16, 16, rl.Brown)
		}

		// velocity line
		rl.DrawLine3D(bodyPos, velEnd, rl.Red)

		// axis helper
		rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(3, 0, 0), rl.Red)
		rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 3, 0), rl.Green)
		rl.DrawLine3D(rl.NewVector3(0, 0, 0), rl.NewVector3(0, 0, 3), rl.Blue)

		rl.EndMode3D()

		status := "RUNNING"
		if paused {
			status = "PAUSED"
		}

		rl.DrawRectangle(10, 10, 430, 170, rl.Fade(rl.SkyBlue, 0.20))
		rl.DrawText(fmt.Sprintf("state: %s", status), 20, 20, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("tick: %d", world.Tick), 20, 45, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("pos: (%.3f, %.3f, %.3f)", px, py, pz), 20, 70, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("vel: (%.3f, %.3f, %.3f)", vx, vy, vz), 20, 95, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("onGround: %v", world.Car.OnGround), 20, 120, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("hash: %d", sim.HashWorld(world)), 20, 145, 20, rl.DarkGray)

		rl.DrawText("Arrow = move | Space = pause | N = single step | R = reset | WASD = move camera", 20, screenHeight-30, 20, rl.Gray)

		rl.EndDrawing()
	}
}
