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
type tuneParam int
type overlayMode int
type cameraRig struct {
	position rl.Vector3
	target   rl.Vector3
	ready    bool
}

const (
	cameraViewPerspective cameraViewMode = iota
	cameraViewTop
	cameraViewSide
)

const (
	tuneDriveForce tuneParam = iota
	tuneFrontGrip
	tuneRearGrip
	tuneAntiRoll
	tuneMaxSteerAngle
	tuneFrontDriveShare
	tuneParamCount
)

const (
	overlayFull overlayMode = iota
	overlayCompact
	overlayHidden
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

func drawVehicleWheelProbes(probes []scenario.VehicleWheelProbeState, wheelRadius fixed.Fixed, wheelWidth fixed.Fixed) {
	radius := fixedToFloat32(wheelRadius)
	halfWidth := fixedToFloat32(wheelWidth) * 0.5
	for _, probe := range probes {
		start := vector3ToRaylibVector(probe.WorldPosition)
		axis := vector3ToRaylibVector(probe.WheelRight)
		if axis.X == 0 && axis.Y == 0 && axis.Z == 0 {
			axis = rl.NewVector3(1, 0, 0)
		}
		if probe.Grounded {
			end := vector3ToRaylibVector(probe.ContactPoint)
			rl.DrawLine3D(start, end, rl.Gold)
			if radius > 0 {
				wheelCenter := probe.ContactPoint.Add(probe.ContactNormal.Scale(wheelRadius))
				center := vector3ToRaylibVector(wheelCenter)
				up := vector3ToRaylibVector(probe.ContactNormal)
				if up.X == 0 && up.Y == 0 && up.Z == 0 {
					up = rl.NewVector3(0, 1, 0)
				}
				drawWheelPlate(center, axis, radius, halfWidth, rl.Black, rl.Fade(rl.DarkGray, 0.7))
			}
		} else {
			end := vector3ToRaylibVector(probe.WorldPosition.Add(geometry.NewVector3(fixed.Zero, fixed.FromFraction(-3, 5), fixed.Zero)))
			rl.DrawLine3D(start, end, rl.Gray)
			if radius > 0 {
				center := vector3ToRaylibVector(probe.WorldPosition.Add(geometry.NewVector3(fixed.Zero, wheelRadius.Neg(), fixed.Zero)))
				drawWheelPlate(center, axis, radius, halfWidth, rl.DarkGray, rl.Fade(rl.Gray, 0.35))
			}
		}
	}
}

func drawWheelPlate(center rl.Vector3, axis rl.Vector3, radius float32, halfWidth float32, wireColor rl.Color, fillColor rl.Color) {
	axisLen := float32(rl.Vector3Length(axis))
	if axisLen == 0 {
		axis = rl.NewVector3(1, 0, 0)
		axisLen = 1
	}
	axis = rl.Vector3Scale(axis, 1/axisLen)
	leftCenter := rl.NewVector3(center.X-axis.X*halfWidth, center.Y-axis.Y*halfWidth, center.Z-axis.Z*halfWidth)
	rightCenter := rl.NewVector3(center.X+axis.X*halfWidth, center.Y+axis.Y*halfWidth, center.Z+axis.Z*halfWidth)
	rl.DrawCylinderEx(leftCenter, rightCenter, radius, radius, 24, fillColor)
	rl.DrawCircle3D(leftCenter, radius, axis, 90, wireColor)
	rl.DrawCircle3D(rightCenter, radius, axis, 90, wireColor)
	rl.DrawCircle3D(leftCenter, radius*0.55, axis, 90, rl.Fade(wireColor, 0.65))
	rl.DrawCircle3D(rightCenter, radius*0.55, axis, 90, rl.Fade(wireColor, 0.65))
}

func lerpVector3(from, to rl.Vector3, alpha float32) rl.Vector3 {
	return rl.NewVector3(
		from.X+(to.X-from.X)*alpha,
		from.Y+(to.Y-from.Y)*alpha,
		from.Z+(to.Z-from.Z)*alpha,
	)
}

func vehicleForwardVector(state scenario.SceneState) rl.Vector3 {
	forward := state.VehicleChassis.Orientation.RotateVector(geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One))
	forward.Y = fixed.Zero
	if forward.LengthSquared() == fixed.Zero {
		return rl.NewVector3(0, 0, 1)
	}
	result := vector3ToRaylibVector(forward.Normalize())
	if result.X == 0 && result.Y == 0 && result.Z == 0 {
		return rl.NewVector3(0, 0, 1)
	}
	return result
}

func chaseCameraTarget(state scenario.SceneState, zoom float32) rl.Vector3 {
	if zoom < 0.35 {
		zoom = 0.35
	}
	baseTarget := vector3ToRaylibVector(state.VehicleChassis.Motion.Position)
	return rl.NewVector3(baseTarget.X, baseTarget.Y+1.2*zoom, baseTarget.Z)
}

func chaseCameraPosition(state scenario.SceneState, zoom float32) rl.Vector3 {
	if zoom < 0.35 {
		zoom = 0.35
	}
	target := chaseCameraTarget(state, zoom)
	forward := vehicleForwardVector(state)
	return rl.NewVector3(
		target.X-forward.X*(8.5*zoom),
		target.Y+2.8*zoom,
		target.Z-forward.Z*(8.5*zoom),
	)
}

func cameraForView(state scenario.SceneState, viewMode cameraViewMode, zoom float32, rig *cameraRig) rl.Camera3D {
	target := vector3ToRaylibVector(state.VehicleChassis.Motion.Position)
	if zoom < 0.35 {
		zoom = 0.35
	}
	switch viewMode {
	case cameraViewTop:
		if rig != nil {
			rig.ready = false
		}
		return rl.Camera3D{
			Position: rl.NewVector3(target.X, target.Y+14*zoom, target.Z+0.01),
			Target:   target,
			Up:       rl.NewVector3(0, 0, -1),
			Fovy:     45,
		}
	case cameraViewSide:
		if rig != nil {
			rig.ready = false
		}
		return rl.Camera3D{
			Position: rl.NewVector3(target.X+14*zoom, target.Y+1.25*zoom, target.Z),
			Target:   target,
			Up:       rl.NewVector3(0, 1, 0),
			Fovy:     45,
		}
	default:
		desiredTarget := chaseCameraTarget(state, zoom)
		desiredPosition := chaseCameraPosition(state, zoom)
		if rig != nil {
			if !rig.ready {
				rig.target = desiredTarget
				rig.position = desiredPosition
				rig.ready = true
			} else {
				rig.target = lerpVector3(rig.target, desiredTarget, 0.18)
				rig.position = lerpVector3(rig.position, desiredPosition, 0.14)
			}
			desiredTarget = rig.target
			desiredPosition = rig.position
		}
		return rl.Camera3D{
			Position: desiredPosition,
			Target:   desiredTarget,
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
		return "Chase"
	}
}

func isManualVehicle3Scene(definition scenario.ScenarioDefinition) bool {
	return definition.Name == "Vehicle3 RayCity Manual Tune"
}

func tuneParamLabel(param tuneParam) string {
	switch param {
	case tuneFrontGrip:
		return "Front Grip"
	case tuneRearGrip:
		return "Rear Grip"
	case tuneAntiRoll:
		return "Anti-Roll"
	case tuneMaxSteerAngle:
		return "Max Steer"
	case tuneFrontDriveShare:
		return "Front Drive"
	default:
		return "Drive Force"
	}
}

func overlayModeLabel(mode overlayMode) string {
	switch mode {
	case overlayCompact:
		return "Compact"
	case overlayHidden:
		return "Hidden"
	default:
		return "Full"
	}
}

func clampFixed(value, min, max fixed.Fixed) fixed.Fixed {
	if value.Cmp(min) < 0 {
		return min
	}
	if value.Cmp(max) > 0 {
		return max
	}
	return value
}

func applyManualVehicle3Input(runner *scenario.ScenarioRunner) {
	if runner == nil {
		return
	}
	throttle := fixed.Zero
	if rl.IsKeyDown(rl.KeyUp) || rl.IsKeyDown(rl.KeyW) {
		throttle = throttle.Add(fixed.One)
	}
	if rl.IsKeyDown(rl.KeyDown) || rl.IsKeyDown(rl.KeyS) {
		throttle = throttle.Sub(fixed.FromFraction(3, 5))
	}
	steer := fixed.Zero
	if rl.IsKeyDown(rl.KeyLeft) || rl.IsKeyDown(rl.KeyA) {
		steer = steer.Add(fixed.One)
	}
	if rl.IsKeyDown(rl.KeyRight) || rl.IsKeyDown(rl.KeyD) {
		steer = steer.Sub(fixed.One)
	}
	runner.State.VehicleThrottleInput = throttle
	runner.State.VehicleSteerInput = steer
}

func adjustVehicle3TuneValue(state *scenario.SceneState, param tuneParam, increase bool) {
	if state == nil {
		return
	}
	signedUnit := fixed.One
	if !increase {
		signedUnit = signedUnit.Neg()
	}
	switch param {
	case tuneDriveForce:
		state.VehicleDriveForce = clampFixed(state.VehicleDriveForce.Add(signedUnit), fixed.FromInt(8), fixed.FromInt(80))
	case tuneFrontGrip:
		state.VehicleFrontGrip = clampFixed(state.VehicleFrontGrip.Add(signedUnit), fixed.FromInt(4), fixed.FromInt(40))
	case tuneRearGrip:
		state.VehicleRearGrip = clampFixed(state.VehicleRearGrip.Add(signedUnit), fixed.FromInt(4), fixed.FromInt(40))
	case tuneAntiRoll:
		state.VehicleAntiRoll = clampFixed(state.VehicleAntiRoll.Add(signedUnit), fixed.Zero, fixed.FromInt(40))
	case tuneMaxSteerAngle:
		delta := fixed.FromFraction(1, 50)
		if !increase {
			delta = delta.Neg()
		}
		state.VehicleMaxSteerAngle = clampFixed(state.VehicleMaxSteerAngle.Add(delta), fixed.FromFraction(1, 20), fixed.FromFraction(3, 5))
	case tuneFrontDriveShare:
		delta := fixed.FromFraction(1, 20)
		if !increase {
			delta = delta.Neg()
		}
		state.VehicleFrontDriveShare = clampFixed(state.VehicleFrontDriveShare.Add(delta), fixed.Zero, fixed.One)
	}
}

func drawOverlay(definition scenario.ScenarioDefinition, runner *scenario.ScenarioRunner, viewMode cameraViewMode, sceneIndex int, sceneCount int, selectedTune tuneParam, overlay overlayMode) {
	if overlay == overlayHidden {
		return
	}
	state := runner.State
	chassis := state.VehicleChassis
	sceneHash := scenario.HashSceneState(state)
	if overlay == overlayCompact {
		rl.DrawRectangle(18, 18, 410, 138, rl.Fade(rl.RayWhite, 0.82))
		rl.DrawRectangleLinesEx(rl.NewRectangle(18, 18, 410, 138), 1, rl.DarkGray)
		rl.DrawText(definition.Name, 28, 26, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Scene %d/%d | Tick %d/%d | %s", sceneIndex+1, sceneCount, runner.Tick, definition.MaxTicks, viewLabel(viewMode)), 28, 52, 16, rl.DarkGray)
		rl.DrawText(fmt.Sprintf("Pos (%s, %s, %s)", fixedToText(chassis.Motion.Position.X), fixedToText(chassis.Motion.Position.Y), fixedToText(chassis.Motion.Position.Z)), 28, 76, 16, rl.Black)
		rl.DrawText(fmt.Sprintf("Gnd %d/%d | Thr %s | Str %s", state.VehicleGroundedProbeCount, len(state.VehicleWheelProbes), fixedToText(state.VehicleThrottleInput), fixedToText(state.VehicleSteerInput)), 28, 100, 16, rl.Black)
		rl.DrawText(fmt.Sprintf("Tune %s | Overlay %s", tuneParamLabel(selectedTune), overlayModeLabel(overlay)), 28, 124, 16, rl.Black)
		return
	}

	rl.DrawRectangle(18, 18, 520, 332, rl.Fade(rl.RayWhite, 0.84))
	rl.DrawRectangleLinesEx(rl.NewRectangle(18, 18, 520, 332), 1, rl.DarkGray)
	rl.DrawText(definition.Name, 28, 26, 20, rl.Black)
	rl.DrawText(fmt.Sprintf("Scene: %d / %d", sceneIndex+1, sceneCount), 28, 54, 16, rl.DarkGray)
	rl.DrawText(fmt.Sprintf("Tick: %d / %d", runner.Tick, definition.MaxTicks), 170, 54, 16, rl.DarkGray)
	rl.DrawText(fmt.Sprintf("View: %s | Overlay: %s", viewLabel(viewMode), overlayModeLabel(overlay)), 330, 54, 16, rl.DarkGray)
	rl.DrawText(fmt.Sprintf("Pos: (%s, %s, %s)", fixedToText(chassis.Motion.Position.X), fixedToText(chassis.Motion.Position.Y), fixedToText(chassis.Motion.Position.Z)), 28, 82, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Vel: (%s, %s, %s)", fixedToText(chassis.Motion.Velocity.X), fixedToText(chassis.Motion.Velocity.Y), fixedToText(chassis.Motion.Velocity.Z)), 28, 106, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Grounded: %d / %d | Front %d | Rear %d", state.VehicleGroundedProbeCount, len(state.VehicleWheelProbes), state.VehicleFrontGroundedProbeCount, state.VehicleRearGroundedProbeCount), 28, 130, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Throttle: %s | Steer: %s | Upright: %s", fixedToText(state.VehicleThrottleInput), fixedToText(state.VehicleSteerInput), fixedToText(state.VehicleUprightDot)), 28, 154, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Drive: %s | Front grip: %s | Rear grip: %s", fixedToText(state.VehicleDriveForce), fixedToText(state.VehicleFrontGrip), fixedToText(state.VehicleRearGrip)), 28, 178, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Anti-roll: %s | Max steer: %s | Front drive: %s", fixedToText(state.VehicleAntiRoll), fixedToText(state.VehicleMaxSteerAngle), fixedToText(state.VehicleFrontDriveShare)), 28, 202, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Wheel radius: %s | Clamp: %s | Avg comp: %s", fixedToText(state.VehicleWheelRadius), fixedToText(state.VehicleWheelCorrectionClamp), fixedToText(state.VehicleAverageCompression)), 28, 226, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Tune param: %s | Step: %s", tuneParamLabel(selectedTune), durationText(runner.LastStepDuration)), 28, 250, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Result: %s", runner.LastResult.Message), 28, 274, 16, rl.Black)
	rl.DrawText(fmt.Sprintf("Hash: %016x", sceneHash), 28, 298, 16, rl.Black)
}

func main() {
	definitions := scenario.Vehicle3ScenarioDefinitions()
	if len(definitions) == 0 {
		panic("debugview_vehicle3: no vehicle scenarios registered")
	}

	rl.InitWindow(1280, 720, "debugview_vehicle3")
	defer rl.CloseWindow()
	rl.SetTargetFPS(60)

	currentIndex := 0
	runner := scenario.NewScenarioRunner(definitions[currentIndex])
	viewMode := cameraViewSide
	cameraZoom := float32(1)
	paused := false
	selectedTune := tuneDriveForce
	overlay := overlayCompact
	rig := cameraRig{}
	autoDrive := false

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyH) {
			overlay = (overlay + 1) % 3
		}
		if rl.IsKeyPressed(rl.KeyF) {
			autoDrive = !autoDrive
		}
		if rl.IsKeyPressed(rl.KeyTab) {
			selectedTune = (selectedTune + 1) % tuneParamCount
		}
		if rl.IsKeyPressed(rl.KeyN) {
			runner.Step()
		}
		if rl.IsKeyPressed(rl.KeyR) {
			previousState := runner.State
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
			rig.ready = false
			if isManualVehicle3Scene(definitions[currentIndex]) {
				runner.State.VehicleDriveForce = previousState.VehicleDriveForce
				runner.State.VehicleFrontGrip = previousState.VehicleFrontGrip
				runner.State.VehicleRearGrip = previousState.VehicleRearGrip
				runner.State.VehicleAntiRoll = previousState.VehicleAntiRoll
				runner.State.VehicleMaxSteerAngle = previousState.VehicleMaxSteerAngle
				runner.State.VehicleFrontDriveShare = previousState.VehicleFrontDriveShare
			}
		}
		if rl.IsKeyPressed(rl.KeyPageDown) {
			currentIndex = (currentIndex + 1) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
			rig.ready = false
		}
		if rl.IsKeyPressed(rl.KeyPageUp) {
			currentIndex--
			if currentIndex < 0 {
				currentIndex = len(definitions) - 1
			}
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
			rig.ready = false
		}
		if rl.IsKeyPressed(rl.KeyOne) {
			viewMode = cameraViewPerspective
			rig.ready = false
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			viewMode = cameraViewTop
			rig.ready = false
		}
		if rl.IsKeyPressed(rl.KeyThree) {
			viewMode = cameraViewSide
			rig.ready = false
		}
		if isManualVehicle3Scene(definitions[currentIndex]) {
			if autoDrive {
				scenario.ApplyVehicle3AutoDriveInput(&runner.State)
			} else {
				applyManualVehicle3Input(runner)
			}
			if rl.IsKeyPressed(rl.KeyEqual) || rl.IsKeyPressed(rl.KeyKpAdd) {
				adjustVehicle3TuneValue(&runner.State, selectedTune, true)
			}
			if rl.IsKeyPressed(rl.KeyMinus) || rl.IsKeyPressed(rl.KeyKpSubtract) {
				adjustVehicle3TuneValue(&runner.State, selectedTune, false)
			}
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

		camera := cameraForView(runner.State, viewMode, cameraZoom, &rig)
		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)
		rl.BeginMode3D(camera)
		drawWorldAxes()
		drawGroundTriangles(runner.State.GroundTriangles)
		drawRigidBoxBody(runner.State.VehicleChassis)
		drawVehicleWheelProbes(runner.State.VehicleWheelProbes, runner.State.VehicleWheelRadius, runner.State.VehicleWheelWidth)
		rl.EndMode3D()
		drawOverlay(definitions[currentIndex], runner, viewMode, currentIndex, len(definitions), selectedTune, overlay)
		rl.DrawText("Controls: Space pause | N step | R reset | PgUp/PgDn scene | 1 chase | 2 top | 3 side | Wheel zoom", 20, int32(rl.GetScreenHeight()-52), 18, rl.Gray)
		autoLabel := "OFF"
		if autoDrive {
			autoLabel = "ON"
		}
		rl.DrawText(fmt.Sprintf("Manual scene: WASD/Arrows drive | F auto drive %s | Tab tune param | +/- adjust value | H overlay", autoLabel), 20, int32(rl.GetScreenHeight()-28), 18, rl.Gray)
		rl.EndDrawing()
	}
}
