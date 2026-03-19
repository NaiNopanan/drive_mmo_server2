package main

import (
	"fmt"
	"time"

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
	cameraViewSide
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

func fixedToText(value fixed.Fixed) string {
	return value.String()
}

func durationText(value time.Duration) string {
	return fmt.Sprintf("%.3f ms", float64(value)/float64(time.Millisecond))
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

func drawTriangleSolid(triangle geometry.Triangle, color rl.Color) {
	rl.DrawTriangle3D(
		vector3ToRaylibVector(triangle.A),
		vector3ToRaylibVector(triangle.B),
		vector3ToRaylibVector(triangle.C),
		color,
	)
}

func drawGroundTriangles(triangles []geometry.Triangle) {
	for _, triangle := range triangles {
		drawTriangleSolid(triangle, rl.Fade(rl.SkyBlue, 0.45))
		drawTriangleWireframe(triangle, rl.Purple)
	}
}

func drawRigidBoxBody(body physics.RigidBoxBody3D) {
	color := rl.Orange
	wireColor := rl.Brown
	if body.Grounded {
		color = rl.Green
		wireColor = rl.DarkGreen
	}

	corners := make([]rl.Vector3, 0, 8)
	for _, corner := range body.WorldCorners() {
		corners = append(corners, vector3ToRaylibVector(corner))
	}

	faces := [][4]int{
		{0, 1, 2, 3},
		{4, 5, 6, 7},
		{0, 1, 5, 4},
		{1, 2, 6, 5},
		{2, 3, 7, 6},
		{3, 0, 4, 7},
	}
	for _, face := range faces {
		rl.DrawTriangle3D(corners[face[0]], corners[face[1]], corners[face[2]], color)
		rl.DrawTriangle3D(corners[face[0]], corners[face[2]], corners[face[3]], color)
	}

	edges := [][2]int{
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	}
	for _, edge := range edges {
		rl.DrawLine3D(corners[edge[0]], corners[edge[1]], wireColor)
	}

	position := vector3ToRaylibVector(body.Motion.Position)
	velocityEnd := rl.NewVector3(
		position.X+fixedToFloat32(body.Motion.Velocity.X)*0.25,
		position.Y+fixedToFloat32(body.Motion.Velocity.Y)*0.25,
		position.Z+fixedToFloat32(body.Motion.Velocity.Z)*0.25,
	)
	rl.DrawLine3D(position, velocityEnd, rl.Red)
}

func drawVehicleWheelProbes(probes []scenario.VehicleWheelProbeState, wheelRadius fixed.Fixed) {
	radius := fixedToFloat32(wheelRadius)
	for _, probe := range probes {
		start := vector3ToRaylibVector(probe.WorldPosition)
		if probe.Grounded {
			end := vector3ToRaylibVector(probe.ContactPoint)
			rl.DrawLine3D(start, end, rl.Gold)
			rl.DrawSphere(end, 0.08, rl.Magenta)
			if radius > 0 {
				wheelCenter := probe.ContactPoint.Add(probe.ContactNormal.Scale(wheelRadius))
				center := vector3ToRaylibVector(wheelCenter)
				rl.DrawSphere(center, radius, rl.Fade(rl.DarkGray, 0.45))
				rl.DrawCircle3D(center, radius, rl.NewVector3(1, 0, 0), 90, rl.Black)
			}
		} else {
			end := vector3ToRaylibVector(probe.WorldPosition.Add(geometry.NewVector3(fixed.Zero, fixed.FromFraction(-7, 10), fixed.Zero)))
			rl.DrawLine3D(start, end, rl.Gray)
			if radius > 0 {
				center := vector3ToRaylibVector(probe.WorldPosition.Add(geometry.NewVector3(fixed.Zero, wheelRadius.Neg(), fixed.Zero)))
				rl.DrawSphere(center, radius, rl.Fade(rl.Gray, 0.2))
			}
		}
		rl.DrawSphere(start, 0.05, rl.DarkPurple)
	}
}

func cameraForView(state scenario.SceneState, viewMode cameraViewMode, zoom float32) rl.Camera3D {
	target := vector3ToRaylibVector(state.VehicleChassis.Motion.Position)
	if zoom < 0.35 {
		zoom = 0.35
	}
	switch viewMode {
	case cameraViewTop:
		return rl.Camera3D{
			Position: rl.NewVector3(target.X, target.Y+14*zoom, target.Z+0.01),
			Target:   target,
			Up:       rl.NewVector3(0, 0, -1),
			Fovy:     45,
		}
	case cameraViewSide:
		return rl.Camera3D{
			Position: rl.NewVector3(target.X+14*zoom, target.Y+1.25*zoom, target.Z),
			Target:   target,
			Up:       rl.NewVector3(0, 1, 0),
			Fovy:     45,
		}
	default:
		return rl.Camera3D{
			Position: rl.NewVector3(target.X+8*zoom, target.Y+6*zoom, target.Z+10*zoom),
			Target:   target,
			Up:       rl.NewVector3(0, 1, 0),
			Fovy:     45,
		}
	}
}

func viewLabel(mode cameraViewMode) string {
	switch mode {
	case cameraViewTop:
		return "Top"
	case cameraViewSide:
		return "Side"
	default:
		return "Perspective"
	}
}

func drawOverlay(definition scenario.ScenarioDefinition, runner *scenario.ScenarioRunner, viewMode cameraViewMode, sceneIndex int, sceneCount int) {
	state := runner.State
	chassis := state.VehicleChassis
	sceneHash := scenario.HashSceneState(state)
	rl.DrawRectangle(18, 18, 540, 342, rl.Fade(rl.RayWhite, 0.92))
	rl.DrawRectangleLinesEx(rl.NewRectangle(18, 18, 540, 342), 2, rl.DarkGray)

	rl.DrawText(definition.Name, 30, 30, 22, rl.Black)
	rl.DrawText(fmt.Sprintf("Scene: %d / %d", sceneIndex+1, sceneCount), 30, 62, 18, rl.DarkGray)
	rl.DrawText(fmt.Sprintf("Tick: %d / %d", runner.Tick, definition.MaxTicks), 180, 62, 18, rl.DarkGray)
	rl.DrawText(fmt.Sprintf("Result: %s", runner.LastResult.Message), 30, 88, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("View: %s", viewLabel(viewMode)), 430, 30, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Step time: %s", durationText(runner.LastStepDuration)), 30, 114, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Position: (%s, %s, %s)", fixedToText(chassis.Motion.Position.X), fixedToText(chassis.Motion.Position.Y), fixedToText(chassis.Motion.Position.Z)), 30, 140, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Velocity: (%s, %s, %s)", fixedToText(chassis.Motion.Velocity.X), fixedToText(chassis.Motion.Velocity.Y), fixedToText(chassis.Motion.Velocity.Z)), 30, 166, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Angular: (%s, %s, %s)", fixedToText(chassis.AngularVelocity.X), fixedToText(chassis.AngularVelocity.Y), fixedToText(chassis.AngularVelocity.Z)), 30, 192, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Grounded probes: %d / %d", state.VehicleGroundedProbeCount, len(state.VehicleWheelProbes)), 30, 218, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Avg compression: %s", fixedToText(state.VehicleAverageCompression)), 30, 244, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Wheel radius: %s", fixedToText(state.VehicleWheelRadius)), 30, 270, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Upright dot: %s | Settled: %v", fixedToText(state.VehicleUprightDot), state.VehicleSettled), 30, 296, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Hash: %016x", sceneHash), 30, 322, 18, rl.Black)
}

func main() {
	definitions := scenario.VehicleScenarioDefinitions()
	if len(definitions) == 0 {
		panic("debugview_vehicle: no vehicle scenarios registered")
	}

	rl.InitWindow(1280, 720, "debugview_vehicle")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	currentIndex := 0
	runner := scenario.NewScenarioRunner(definitions[currentIndex])
	viewMode := cameraViewSide
	cameraZoom := float32(1)
	paused := false

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			runner.Step()
		}
		if rl.IsKeyPressed(rl.KeyR) {
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
		}
		if rl.IsKeyPressed(rl.KeyRight) {
			currentIndex = (currentIndex + 1) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
		}
		if rl.IsKeyPressed(rl.KeyLeft) {
			currentIndex--
			if currentIndex < 0 {
				currentIndex = len(definitions) - 1
			}
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
		}
		if rl.IsKeyPressed(rl.KeyOne) {
			viewMode = cameraViewPerspective
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			viewMode = cameraViewTop
		}
		if rl.IsKeyPressed(rl.KeyThree) {
			viewMode = cameraViewSide
		}
		wheel := rl.GetMouseWheelMove()
		if wheel != 0 {
			cameraZoom -= wheel * 0.1
			if cameraZoom < 0.35 {
				cameraZoom = 0.35
			}
			if cameraZoom > 2.5 {
				cameraZoom = 2.5
			}
		}
		if !paused && !runner.Finished {
			runner.Step()
		}

		camera := cameraForView(runner.State, viewMode, cameraZoom)
		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)
		rl.BeginMode3D(camera)
		drawWorldAxes()
		drawGroundTriangles(runner.State.GroundTriangles)
		drawRigidBoxBody(runner.State.VehicleChassis)
		drawVehicleWheelProbes(runner.State.VehicleWheelProbes, runner.State.VehicleWheelRadius)
		rl.EndMode3D()
		drawOverlay(definitions[currentIndex], runner, viewMode, currentIndex, len(definitions))
		rl.DrawText("Controls: Space pause | N step | R reset | Left/Right scene | 1 perspective | 2 top | 3 side | Wheel zoom", 20, int32(rl.GetScreenHeight()-28), 18, rl.Gray)
		rl.EndDrawing()
	}
}
