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

func fixedToText(value fixed.Fixed) string {
	return value.String()
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
	a := vector3ToRaylibVector(triangle.A)
	b := vector3ToRaylibVector(triangle.B)
	c := vector3ToRaylibVector(triangle.C)

	rl.DrawTriangle3D(a, b, c, color)
}

func drawGroundTriangles(triangles []geometry.Triangle) {
	for _, triangle := range triangles {
		fillColor := rl.Fade(rl.SkyBlue, 0.45)
		drawTriangleSolid(triangle, fillColor)
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

func drawRigidSphereBody(body physics.RigidSphereBody3D) {
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

	axisLength := body.Radius.Mul(fixed.FromFraction(6, 5))
	localAxes := []struct {
		Axis  geometry.Vector3
		Color rl.Color
	}{
		{Axis: geometry.NewVector3(axisLength, fixed.Zero, fixed.Zero), Color: rl.Red},
		{Axis: geometry.NewVector3(fixed.Zero, axisLength, fixed.Zero), Color: rl.Blue},
		{Axis: geometry.NewVector3(fixed.Zero, fixed.Zero, axisLength), Color: rl.Gold},
	}
	for _, axis := range localAxes {
		worldAxis := body.Orientation.RotateVector(axis.Axis)
		rl.DrawLine3D(position, vector3ToRaylibVector(body.Motion.Position.Add(worldAxis)), axis.Color)
	}

	velocityEnd := rl.NewVector3(
		position.X+fixedToFloat32(body.Motion.Velocity.X)*0.25,
		position.Y+fixedToFloat32(body.Motion.Velocity.Y)*0.25,
		position.Z+fixedToFloat32(body.Motion.Velocity.Z)*0.25,
	)
	rl.DrawLine3D(position, velocityEnd, rl.Maroon)
}

func drawRigidSphereBodies(bodies []physics.RigidSphereBody3D) {
	for _, body := range bodies {
		drawRigidSphereBody(body)
	}
}

func drawBoxBody(body physics.BoxBody) {
	color := rl.Orange
	wireColor := rl.Brown
	if body.Grounded {
		color = rl.Green
		wireColor = rl.DarkGreen
	}

	drawBoxSolid(body, color)
	drawBoxWireframe(body, wireColor)

	position := vector3ToRaylibVector(body.Motion.Position)

	velocityEnd := rl.NewVector3(
		position.X+fixedToFloat32(body.Motion.Velocity.X)*0.25,
		position.Y+fixedToFloat32(body.Motion.Velocity.Y)*0.25,
		position.Z+fixedToFloat32(body.Motion.Velocity.Z)*0.25,
	)
	rl.DrawLine3D(position, velocityEnd, rl.Red)
}

func boxCorners(body physics.BoxBody) []rl.Vector3 {
	corners := make([]rl.Vector3, 0, 8)
	localCorners := []geometry.Vector3{
		geometry.NewVector3(body.HalfExtents.X.Neg(), body.HalfExtents.Y.Neg(), body.HalfExtents.Z.Neg()),
		geometry.NewVector3(body.HalfExtents.X, body.HalfExtents.Y.Neg(), body.HalfExtents.Z.Neg()),
		geometry.NewVector3(body.HalfExtents.X, body.HalfExtents.Y, body.HalfExtents.Z.Neg()),
		geometry.NewVector3(body.HalfExtents.X.Neg(), body.HalfExtents.Y, body.HalfExtents.Z.Neg()),
		geometry.NewVector3(body.HalfExtents.X.Neg(), body.HalfExtents.Y.Neg(), body.HalfExtents.Z),
		geometry.NewVector3(body.HalfExtents.X, body.HalfExtents.Y.Neg(), body.HalfExtents.Z),
		geometry.NewVector3(body.HalfExtents.X, body.HalfExtents.Y, body.HalfExtents.Z),
		geometry.NewVector3(body.HalfExtents.X.Neg(), body.HalfExtents.Y, body.HalfExtents.Z),
	}

	sinAngle := fixed.Sin(body.RotationZ)
	cosAngle := fixed.Cos(body.RotationZ)
	for _, corner := range localCorners {
		rotated := geometry.NewVector3(
			corner.X.Mul(cosAngle).Sub(corner.Y.Mul(sinAngle)),
			corner.X.Mul(sinAngle).Add(corner.Y.Mul(cosAngle)),
			corner.Z,
		)
		corners = append(corners, vector3ToRaylibVector(body.Motion.Position.Add(rotated)))
	}

	return corners
}

func drawBoxSolid(body physics.BoxBody, color rl.Color) {
	corners := boxCorners(body)
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
}

func drawBoxWireframe(body physics.BoxBody, color rl.Color) {
	corners := boxCorners(body)
	edges := [][2]int{
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	}
	for _, edge := range edges {
		rl.DrawLine3D(corners[edge[0]], corners[edge[1]], color)
	}
}

func drawBoxBodies(bodies []physics.BoxBody) {
	for _, body := range bodies {
		drawBoxBody(body)
	}
}

func rigidBoxCorners(body physics.RigidBoxBody3D) []rl.Vector3 {
	corners := make([]rl.Vector3, 0, 8)
	for _, corner := range body.WorldCorners() {
		corners = append(corners, vector3ToRaylibVector(corner))
	}
	return corners
}

func drawRigidBoxBody(body physics.RigidBoxBody3D) {
	color := rl.Orange
	wireColor := rl.Brown
	if body.Grounded {
		color = rl.Green
		wireColor = rl.DarkGreen
	}

	corners := rigidBoxCorners(body)
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

func drawRigidBoxBodies(bodies []physics.RigidBoxBody3D) {
	for _, body := range bodies {
		drawRigidBoxBody(body)
	}
}

func sceneRigidBoxes(state scenario.SceneState) []physics.RigidBoxBody3D {
	if len(state.RigidBoxes) > 0 {
		return state.RigidBoxes
	}
	if state.RigidBox.HalfExtents.X.Cmp(fixed.Zero) > 0 {
		return []physics.RigidBoxBody3D{state.RigidBox}
	}
	return nil
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

func drawObjectLabel(camera rl.Camera3D, worldPosition geometry.Vector3, lines []string, color rl.Color) {
	if len(lines) == 0 {
		return
	}

	labelWorldPosition := worldPosition.Add(geometry.NewVector3(fixed.Zero, fixed.FromInt(2), fixed.Zero))
	screenPosition := rl.GetWorldToScreen(vector3ToRaylibVector(labelWorldPosition), camera)
	if screenPosition.X < -200 || screenPosition.X > float32(rl.GetScreenWidth()+200) ||
		screenPosition.Y < -200 || screenPosition.Y > float32(rl.GetScreenHeight()+200) {
		return
	}

	const lineHeight int32 = 16
	padding := int32(6)
	maxWidth := int32(0)
	for _, line := range lines {
		width := rl.MeasureText(line, 14)
		if width > maxWidth {
			maxWidth = width
		}
	}

	x := int32(screenPosition.X) - maxWidth/2 - padding
	y := int32(screenPosition.Y) - int32(len(lines))*lineHeight - padding
	height := int32(len(lines))*lineHeight + padding*2
	width := maxWidth + padding*2

	rl.DrawRectangle(x, y, width, height, rl.Fade(rl.RayWhite, 0.9))
	rl.DrawRectangleLines(x, y, width, height, color)
	for index, line := range lines {
		rl.DrawText(line, x+padding, y+padding+int32(index)*lineHeight, 14, color)
	}
}

func drawSceneObjectLabels(camera rl.Camera3D, state scenario.SceneState) {
	for index, sphere := range sceneSpheres(state) {
		lines := []string{
			fmt.Sprintf("Sphere %d", index+1),
			fmt.Sprintf("m=%s r=%s", fixedToText(sphere.Motion.Mass), fixedToText(sphere.Radius)),
			fmt.Sprintf("vx=%s vy=%s", fixedToText(sphere.Motion.Velocity.X), fixedToText(sphere.Motion.Velocity.Y)),
		}
		if state.SphereSphereCollisionDetected {
			lines = append(lines, "collision=true")
		} else {
			lines = append(lines, fmt.Sprintf("grounded=%v", sphere.Grounded))
		}
		drawObjectLabel(camera, sphere.Motion.Position, lines, rl.DarkBlue)
	}

	for index, sphere := range sceneRigidSpheres(state) {
		lines := []string{
			fmt.Sprintf("Rigid Sphere %d", index+1),
			fmt.Sprintf("m=%s r=%s", fixedToText(sphere.Motion.Mass), fixedToText(sphere.Radius)),
			fmt.Sprintf("vx=%s vy=%s", fixedToText(sphere.Motion.Velocity.X), fixedToText(sphere.Motion.Velocity.Y)),
			fmt.Sprintf("grounded=%v", sphere.Grounded),
		}
		drawObjectLabel(camera, sphere.Motion.Position, lines, rl.Purple)
	}

	for index, box := range sceneBoxes(state) {
		lines := []string{
			fmt.Sprintf("Box %d", index+1),
			fmt.Sprintf("m=%s", fixedToText(box.Motion.Mass)),
			fmt.Sprintf("vx=%s vy=%s", fixedToText(box.Motion.Velocity.X), fixedToText(box.Motion.Velocity.Y)),
			fmt.Sprintf("grounded=%v", box.Grounded),
		}
		drawObjectLabel(camera, box.Motion.Position, lines, rl.DarkGreen)
	}

	for index, box := range sceneRigidBoxes(state) {
		lines := []string{
			fmt.Sprintf("Rigid Box %d", index+1),
			fmt.Sprintf("m=%s", fixedToText(box.Motion.Mass)),
			fmt.Sprintf("vx=%s vy=%s", fixedToText(box.Motion.Velocity.X), fixedToText(box.Motion.Velocity.Y)),
			fmt.Sprintf("grounded=%v", box.Grounded),
		}
		drawObjectLabel(camera, box.Motion.Position, lines, rl.Brown)
	}
}

func sceneSpheres(state scenario.SceneState) []physics.SphereBody {
	if len(state.Spheres) > 0 {
		return state.Spheres
	}
	if state.Sphere.Radius.Cmp(fixed.Zero) > 0 {
		return []physics.SphereBody{state.Sphere}
	}
	return nil
}

func sceneRigidSpheres(state scenario.SceneState) []physics.RigidSphereBody3D {
	if len(state.RigidSpheres) > 0 {
		return state.RigidSpheres
	}
	if state.RigidSphere.Radius.Cmp(fixed.Zero) > 0 {
		return []physics.RigidSphereBody3D{state.RigidSphere}
	}
	return nil
}

func sceneBoxes(state scenario.SceneState) []physics.BoxBody {
	if len(state.Boxes) > 0 {
		return state.Boxes
	}
	if state.Box.HalfExtents.X.Cmp(fixed.Zero) > 0 {
		return []physics.BoxBody{state.Box}
	}
	return nil
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

func defaultSideViewMode() cameraViewMode {
	return cameraViewFront
}

func shouldFollowSphere(definition scenario.ScenarioDefinition) bool {
	return definition.Name == "Sphere Drop On 30 Degree Slope" ||
		definition.Name == "Sphere Bounce On 30 Degree Slope"
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
	rigidSpheres := sceneRigidSpheres(state)
	boxes := sceneBoxes(state)
	rigidBoxes := sceneRigidBoxes(state)
	contactNormal := state.LastContact.Normal
	sceneHash := scenario.HashSceneState(state)

	position := geometry.ZeroVector3()
	velocity := geometry.ZeroVector3()
	grounded := false
	objectCount := len(spheres)

	if len(rigidBoxes) > 0 {
		position = rigidBoxes[0].Motion.Position
		velocity = rigidBoxes[0].Motion.Velocity
		grounded = rigidBoxes[0].Grounded
		objectCount = len(rigidBoxes)
	} else if len(rigidSpheres) > 0 {
		position = rigidSpheres[0].Motion.Position
		velocity = rigidSpheres[0].Motion.Velocity
		grounded = rigidSpheres[0].Grounded
		objectCount = len(rigidSpheres)
	} else if len(boxes) > 0 {
		position = boxes[0].Motion.Position
		velocity = boxes[0].Motion.Velocity
		grounded = boxes[0].Grounded
		objectCount = len(boxes)
	} else if len(spheres) > 0 {
		position = spheres[0].Motion.Position
		velocity = spheres[0].Motion.Velocity
		grounded = spheres[0].Grounded
	}

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
		fixedToFloat32(position.X),
		fixedToFloat32(position.Y),
		fixedToFloat32(position.Z),
	), 30, 174, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Velocity: (%.3f, %.3f, %.3f)",
		fixedToFloat32(velocity.X),
		fixedToFloat32(velocity.Y),
		fixedToFloat32(velocity.Z),
	), 30, 202, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Grounded: %v | Ever touched ground: %v", grounded, state.EverTouchedGround), 30, 230, 18, rl.Black)
	rl.DrawText(fmt.Sprintf("Object count: %d", objectCount), 30, 258, 18, rl.Black)
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
	viewMode := defaultSideViewMode()
	viewDistance := float32(16)

	camera := rl.Camera3D{
		Position:   rl.NewVector3(0, 6, -16),
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
			viewMode = defaultSideViewMode()
		}
		if rl.IsKeyPressed(rl.KeyRight) {
			currentIndex = (currentIndex + 1) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
			viewMode = defaultSideViewMode()
		}
		if rl.IsKeyPressed(rl.KeyLeft) {
			currentIndex = (currentIndex - 1 + len(definitions)) % len(definitions)
			runner = scenario.NewScenarioRunner(definitions[currentIndex])
			viewMode = defaultSideViewMode()
		}

		previousViewMode := viewMode
		viewMode = updateViewModeFromKeyboard(viewMode)
		spheres := sceneSpheres(runner.State)
		rigidSpheres := sceneRigidSpheres(runner.State)
		boxes := sceneBoxes(runner.State)
		rigidBoxes := sceneRigidBoxes(runner.State)
		focusSource := geometry.ZeroVector3()
		if len(rigidBoxes) > 0 {
			focusSource = rigidBoxes[0].Motion.Position
		} else if len(rigidSpheres) > 0 {
			focusSource = rigidSpheres[0].Motion.Position
		} else if len(boxes) > 0 {
			focusSource = boxes[0].Motion.Position
		} else if len(spheres) > 0 {
			focusSource = spheres[0].Motion.Position
		}
		focus := rl.NewVector3(
			fixedToFloat32(focusSource.X),
			fixedToFloat32(focusSource.Y),
			fixedToFloat32(focusSource.Z),
		)

		if shouldFollowSphere(definitions[currentIndex]) {
			if focus.Y < -8 {
				focus.Y = -8
			}
		} else {
			if focus.Y < 1 {
				focus.Y = 1
			}
			focus.X = 0
			focus.Z = 0
		}

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
		drawRigidSphereBodies(sceneRigidSpheres(runner.State))
		drawBoxBodies(sceneBoxes(runner.State))
		drawRigidBoxBodies(sceneRigidBoxes(runner.State))
		drawContactDebugList(sceneContacts(runner.State))
		rl.EndMode3D()

		drawSceneObjectLabels(camera, runner.State)
		drawOverlayWithCameraMode(definitions[currentIndex], runner, viewMode)

		rl.EndDrawing()
	}
}
