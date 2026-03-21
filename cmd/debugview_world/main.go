package main

import (
	"fmt"
	"math"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/physics"
	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const (
	screenWidth  = 1280
	screenHeight = 720

	groundY      = 0.0
	chassisH     = 0.9
	cameraHeight = 6.5

	worldMeshPath  = "world.glb"
	worldMeshScale = 1.0
)

type followCamera struct {
	position rl.Vector3
	target   rl.Vector3
	distance float32
	preset   cameraPreset
}

type renderMode int

const (
	renderModeRaw renderMode = iota
	renderModeInterpolated
)

type cameraPreset int

const (
	cameraPresetChase cameraPreset = iota
	cameraPresetClose
	cameraPresetTop
	cameraPresetSide
)

type debugRenderer struct {
	cube rl.Model
}

func main() {
	rl.InitWindow(screenWidth, screenHeight, "Drive MMO Vehicle Sandbox")
	defer rl.CloseWindow()

	rl.SetTargetFPS(60)

	config := physics.DefaultWorldConfig()
	staticMesh, err := worldmesh.LoadGLB(worldMeshPath)
	if err != nil {
		panic(err)
	}
	config.StaticMesh = staticMesh
	config.WorldBounds = meshBoundsToWorldBounds(staticMesh, 20)

	world := physics.NewWorld(config)
	renderer := newDebugRenderer()
	defer renderer.Close()

	currentSnapshot := world.Snapshot()
	previousSnapshot := currentSnapshot
	cameraRig := newFollowCamera(currentSnapshot.Player)

	var accumulator float32
	paused := false
	mode := renderModeInterpolated

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeyP) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyOne) {
			cameraRig.SetPreset(cameraPresetChase)
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			cameraRig.SetPreset(cameraPresetClose)
		}
		if rl.IsKeyPressed(rl.KeyThree) {
			cameraRig.SetPreset(cameraPresetTop)
		}
		if rl.IsKeyPressed(rl.KeyFour) {
			cameraRig.SetPreset(cameraPresetSide)
		}
		if rl.IsKeyPressed(rl.KeyF1) {
			mode = renderModeInterpolated
		}
		if rl.IsKeyPressed(rl.KeyF2) {
			mode = renderModeRaw
		}

		if rl.IsKeyPressed(rl.KeyR) {
			world.Reset()
			accumulator = 0
			currentSnapshot = world.Snapshot()
			previousSnapshot = currentSnapshot
		}

		input := readPlayerInput()
		frameDT := min(rl.GetFrameTime(), 0.25)

		if !paused {
			accumulator += frameDT
			for accumulator >= world.FixedDT() {
				previousSnapshot = currentSnapshot
				world.Step(input)
				currentSnapshot = world.Snapshot()
				accumulator -= world.FixedDT()
			}
		} else if rl.IsKeyPressed(rl.KeyN) {
			previousSnapshot = currentSnapshot
			world.Step(input)
			currentSnapshot = world.Snapshot()
		}

		renderSnapshot := currentSnapshot
		if mode == renderModeInterpolated && !paused {
			alpha := accumulator / world.FixedDT()
			renderSnapshot = interpolateSnapshot(previousSnapshot, currentSnapshot, alpha)
		}

		cameraRig.Update(renderSnapshot.Player)

		rl.BeginDrawing()
		rl.ClearBackground(rl.NewColor(176, 220, 255, 255))

		rl.BeginMode3D(cameraRig.Camera())
		drawScene(renderSnapshot, renderer)
		rl.EndMode3D()

		drawHUD(renderSnapshot, paused, mode, cameraRig.preset)
		rl.EndDrawing()
	}
}

func newDebugRenderer() debugRenderer {
	return debugRenderer{
		cube: rl.LoadModelFromMesh(rl.GenMeshCube(1, 1, 1)),
	}
}

func (r debugRenderer) Close() {
	rl.UnloadModel(r.cube)
}

func newFollowCamera(player physics.VehicleSnapshot) *followCamera {
	rig := &followCamera{
		preset: cameraPresetChase,
	}
	rig.SetPreset(cameraPresetChase)

	target, position := rig.desiredView(player)
	rig.target = target
	rig.position = position
	return rig
}

func (c *followCamera) SetPreset(preset cameraPreset) {
	c.preset = preset
	switch preset {
	case cameraPresetClose:
		c.distance = 7
	case cameraPresetTop:
		c.distance = 18
	case cameraPresetSide:
		c.distance = 11
	default:
		c.distance = 12
	}
}

func (c *followCamera) Update(player physics.VehicleSnapshot) {
	// ซูมเข้าออกด้วยล้อเมาส์เพื่อดูตัวรถหรือดูภาพรวมของ sandbox
	c.distance -= rl.GetMouseWheelMove() * 2
	minDistance, maxDistance := c.zoomRange()
	c.distance = clamp(c.distance, minDistance, maxDistance)

	desiredTarget, desiredPosition := c.desiredView(player)

	c.target = rl.Vector3Lerp(c.target, desiredTarget, 0.12)
	c.position = rl.Vector3Lerp(c.position, desiredPosition, 0.10)
}

func (c *followCamera) Camera() rl.Camera3D {
	return rl.NewCamera3D(
		c.position,
		c.target,
		rl.NewVector3(0, 1, 0),
		45,
		rl.CameraPerspective,
	)
}

func (c *followCamera) desiredView(player physics.VehicleSnapshot) (rl.Vector3, rl.Vector3) {
	baseTarget := rl.Vector3Add(toRenderVector(player.Position, player.Height+chassisH*0.5), rl.NewVector3(0, 1.2, 0))
	forward := headingForward3D(player.Heading)

	switch c.preset {
	case cameraPresetClose:
		target := rl.Vector3Add(baseTarget, rl.Vector3Scale(forward, 2.8))
		position := rl.Vector3Add(target, rl.NewVector3(0, 3.4, 0))
		position = rl.Vector3Add(position, rl.Vector3Scale(forward, -c.distance))
		return target, position
	case cameraPresetTop:
		target := baseTarget
		position := rl.Vector3Add(target, rl.NewVector3(0, c.distance, 0))
		position = rl.Vector3Add(position, rl.Vector3Scale(forward, -1.5))
		return target, position
	case cameraPresetSide:
		right := rl.NewVector3(-forward.Z, 0, forward.X)
		target := rl.Vector3Add(baseTarget, rl.NewVector3(0, -0.9, 0))
		position := rl.Vector3Add(target, rl.NewVector3(0, 0.32, 0))
		position = rl.Vector3Add(position, rl.Vector3Scale(right, c.distance))
		return target, position
	default:
		target := baseTarget
		position := rl.Vector3Add(target, rl.NewVector3(0, cameraHeight, 0))
		position = rl.Vector3Add(position, rl.Vector3Scale(forward, -c.distance))
		return target, position
	}
}

func (c *followCamera) zoomRange() (float32, float32) {
	switch c.preset {
	case cameraPresetClose:
		return 5.5, 10
	case cameraPresetTop:
		return 12, 28
	case cameraPresetSide:
		return 8, 16
	default:
		return 8, 24
	}
}

func readPlayerInput() physics.DriveInput {
	input := physics.DriveInput{}

	if rl.IsKeyDown(rl.KeyW) || rl.IsKeyDown(rl.KeyUp) {
		input.Throttle = 1
	}
	if rl.IsKeyDown(rl.KeyS) || rl.IsKeyDown(rl.KeyDown) {
		input.Brake = 1
	}
	if rl.IsKeyDown(rl.KeyA) || rl.IsKeyDown(rl.KeyLeft) {
		input.Steering = -1
	}
	if rl.IsKeyDown(rl.KeyD) || rl.IsKeyDown(rl.KeyRight) {
		input.Steering = 1
	}

	input.Handbrake = rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl)
	input.Nitro = rl.IsKeyDown(rl.KeyLeftShift) || rl.IsKeyDown(rl.KeyRightShift)

	return input
}

func drawScene(snapshot physics.WorldSnapshot, renderer debugRenderer) {
	drawGround(snapshot)
	drawWorldBounds(snapshot)
	drawStaticMesh(snapshot.StaticMesh)
	renderer.DrawVehicleCollider(snapshot.Player, rl.NewColor(86, 194, 255, 110), rl.NewColor(86, 194, 255, 255))
	drawWheelRays(snapshot.Player)
}

func drawGround(snapshot physics.WorldSnapshot) {
	rl.DrawGrid(40, 40)
}

func drawWorldBounds(snapshot physics.WorldSnapshot) {
	center := toRenderVector(geom.Planar(
		(snapshot.Bounds.Min.X+snapshot.Bounds.Max.X)/2,
		(snapshot.Bounds.Min.Z+snapshot.Bounds.Max.Z)/2,
	), 4)

	size := rl.NewVector3(snapshot.Bounds.Width(), 8, snapshot.Bounds.Height())
	rl.DrawCubeWiresV(center, size, rl.NewColor(24, 38, 44, 255))
}

func drawStaticMesh(staticMesh worldmesh.StaticMesh) {
	// viewer วาดจาก triangle cache ที่ world เก็บไว้ ไม่ได้ถือ model ดิบระหว่างรัน
	fillColor := rl.NewColor(96, 102, 112, 255)
	wireColor := rl.NewColor(180, 188, 198, 255)
	for _, triangle := range staticMesh.Triangles {
		a := toRenderVector3(triangle.A)
		b := toRenderVector3(triangle.B)
		c := toRenderVector3(triangle.C)

		rl.DrawTriangle3D(a, b, c, fillColor)
		rl.DrawLine3D(a, b, wireColor)
		rl.DrawLine3D(b, c, wireColor)
		rl.DrawLine3D(c, a, wireColor)
	}
}

func (r debugRenderer) DrawVehicleCollider(vehicle physics.VehicleSnapshot, fillColor rl.Color, lineColor rl.Color) {
	corners := chassisCorners(vehicle)
	drawFilledBox(corners, fillColor)
	drawLoop3D(corners[:4], lineColor)
	drawLoop3D(corners[4:], lineColor)
	for index := 0; index < 4; index++ {
		rl.DrawLine3D(corners[index], corners[index+4], lineColor)
	}

	headingStart := vehiclePoint(vehicle, 0, 0, 0)
	headingEnd := vehiclePoint(vehicle, vehicle.Length*0.65, 0, 0)
	rl.DrawLine3D(headingStart, headingEnd, rl.NewColor(255, 255, 255, 255))
	drawHeadingArrow(vehicle)

	velocityEnd := rl.Vector3Add(
		headingStart,
		rl.NewVector3(vehicle.Velocity.X*0.35, 0, vehicle.Velocity.Z*0.35),
	)
	rl.DrawLine3D(headingStart, velocityEnd, rl.NewColor(255, 96, 96, 255))
}

func drawFilledBox(corners [8]rl.Vector3, color rl.Color) {
	drawQuad(corners[0], corners[1], corners[2], corners[3], color)
	drawQuad(corners[4], corners[5], corners[6], corners[7], color)
	drawQuad(corners[0], corners[1], corners[5], corners[4], color)
	drawQuad(corners[3], corners[2], corners[6], corners[7], color)
	drawQuad(corners[0], corners[3], corners[7], corners[4], color)
	drawQuad(corners[1], corners[2], corners[6], corners[5], color)
}

func drawQuad(a, b, c, d rl.Vector3, color rl.Color) {
	rl.DrawTriangle3D(a, b, c, color)
	rl.DrawTriangle3D(a, c, d, color)
}

func drawWheelRays(vehicle physics.VehicleSnapshot) {
	for _, wheel := range vehicle.Wheels {
		mountPoint := toRenderVector3(wheel.MountPoint)
		if wheel.Hit {
			wheelCenter := toRenderVector3(wheel.WheelCenter)
			hitPoint := toRenderVector3(wheel.HitPoint)
			rl.DrawLine3D(mountPoint, wheelCenter, rl.NewColor(255, 210, 64, 255))
			rl.DrawLine3D(wheelCenter, hitPoint, rl.NewColor(255, 120, 96, 255))
			rl.DrawSphere(wheelCenter, 0.07, rl.NewColor(255, 245, 180, 255))
			rl.DrawSphere(hitPoint, 0.05, rl.NewColor(255, 120, 96, 255))
			continue
		}

		end := rl.Vector3Add(mountPoint, rl.NewVector3(0, -0.8, 0))
		rl.DrawLine3D(mountPoint, end, rl.NewColor(255, 210, 64, 180))
	}
}

func chassisCorners(vehicle physics.VehicleSnapshot) [8]rl.Vector3 {
	halfLength := vehicle.Length * 0.5
	halfWidth := vehicle.Width * 0.5
	halfHeight := float32(chassisH * 0.5)

	var corners [8]rl.Vector3
	corners[0] = vehiclePoint(vehicle, halfLength, halfHeight, halfWidth)
	corners[1] = vehiclePoint(vehicle, halfLength, halfHeight, -halfWidth)
	corners[2] = vehiclePoint(vehicle, -halfLength, halfHeight, -halfWidth)
	corners[3] = vehiclePoint(vehicle, -halfLength, halfHeight, halfWidth)
	corners[4] = vehiclePoint(vehicle, halfLength, -halfHeight, halfWidth)
	corners[5] = vehiclePoint(vehicle, halfLength, -halfHeight, -halfWidth)
	corners[6] = vehiclePoint(vehicle, -halfLength, -halfHeight, -halfWidth)
	corners[7] = vehiclePoint(vehicle, -halfLength, -halfHeight, halfWidth)

	return corners
}

func drawLoop3D(points []rl.Vector3, color rl.Color) {
	for index := range points {
		next := (index + 1) % len(points)
		rl.DrawLine3D(points[index], points[next], color)
	}
}

func drawHeadingArrow(vehicle physics.VehicleSnapshot) {
	arrowColor := rl.NewColor(255, 255, 255, 255)
	roofY := float32(chassisH*0.5 + 0.05)

	tip := vehiclePoint(vehicle, vehicle.Length*0.45, roofY, 0)
	left := vehiclePoint(vehicle, vehicle.Length*0.08, roofY, vehicle.Width*0.22)
	rightPoint := vehiclePoint(vehicle, vehicle.Length*0.08, roofY, -vehicle.Width*0.22)

	rl.DrawTriangle3D(tip, left, rightPoint, arrowColor)
	rl.DrawLine3D(tip, left, arrowColor)
	rl.DrawLine3D(left, rightPoint, arrowColor)
	rl.DrawLine3D(rightPoint, tip, arrowColor)
}

func drawHUD(snapshot physics.WorldSnapshot, paused bool, mode renderMode, camera cameraPreset) {
	lines := []string{
		"W/S = throttle/brake",
		"A/D = steer",
		"Ctrl = handbrake",
		"Shift = nitro",
		"Mouse wheel = zoom camera",
		"1/2/3/4 = camera, P = pause, N = single step, R = reset",
		"F1 = smooth, F2 = raw",
		fmt.Sprintf("tick: %d", snapshot.Tick),
		fmt.Sprintf("speed: %.1f", snapshot.Player.Speed),
		fmt.Sprintf("car size: %.1fm x %.1fm", snapshot.Player.Length, snapshot.Player.Width),
		fmt.Sprintf("height: %.2fm / ground: %.2fm", snapshot.Player.Height, snapshot.Player.GroundHeight),
		fmt.Sprintf("pitch: %.2f / roll: %.2f", snapshot.Player.Pitch, snapshot.Player.Roll),
		fmt.Sprintf("suspension: rest=%.2fm wheel=%.2fm", 0.50, 0.32),
		fmt.Sprintf("support: %s (%d hits)", snapshot.Player.SupportState, snapshot.Player.SupportHits),
		fmt.Sprintf("heading: %.2f", snapshot.Player.Heading),
		fmt.Sprintf("position: [%.1f, %.1f]", snapshot.Player.Position.X, snapshot.Player.Position.Z),
		fmt.Sprintf("world triangles: %d", len(snapshot.StaticMesh.Triangles)),
		fmt.Sprintf("world size: %.1fm x %.1fm", snapshot.StaticMesh.Max.X-snapshot.StaticMesh.Min.X, snapshot.StaticMesh.Max.Z-snapshot.StaticMesh.Min.Z),
	}
	for _, wheel := range snapshot.Player.Wheels {
		lines = append(lines, fmt.Sprintf("%s: hit=%t len=%.2f comp=%.2f", wheel.Label, wheel.Hit, wheel.SuspensionLength, wheel.Compression))
	}

	state := "running"
	if paused {
		state = "paused"
	}
	lines = append(lines, "state: "+state)
	lines = append(lines, "render mode: "+mode.String())
	lines = append(lines, "camera: "+camera.String())

	for index, line := range lines {
		rl.DrawText(line, 28, 28+int32(index)*22, 20, rl.RayWhite)
	}
}

func toRenderVector(position geom.PlanarVec, height float32) rl.Vector3 {
	return rl.NewVector3(position.X, height, position.Z)
}

func toRenderVector3(position geom.Vec3) rl.Vector3 {
	return rl.NewVector3(position.X, position.Y*worldMeshScale, position.Z)
}

func meshBoundsToWorldBounds(staticMesh worldmesh.StaticMesh, margin float32) geom.AABB {
	if len(staticMesh.Triangles) == 0 {
		return geom.NewAABB(-120, -120, 120, 120)
	}

	return geom.NewAABB(
		staticMesh.Min.X-margin,
		staticMesh.Min.Z-margin,
		staticMesh.Max.X+margin,
		staticMesh.Max.Z+margin,
	)
}

func headingForward3D(heading float32) rl.Vector3 {
	return rl.NewVector3(
		float32(math.Cos(float64(heading))),
		0,
		float32(math.Sin(float64(heading))),
	)
}

func clamp(value, minValue, maxValue float32) float32 {
	if value < minValue {
		return minValue
	}
	if value > maxValue {
		return maxValue
	}
	return value
}

func min(a, b float32) float32 {
	if a < b {
		return a
	}
	return b
}

func (m renderMode) String() string {
	switch m {
	case renderModeInterpolated:
		return "interpolated"
	default:
		return "raw"
	}
}

func (p cameraPreset) String() string {
	switch p {
	case cameraPresetClose:
		return "close"
	case cameraPresetTop:
		return "top"
	case cameraPresetSide:
		return "side"
	default:
		return "chase"
	}
}

func interpolateSnapshot(previous, current physics.WorldSnapshot, alpha float32) physics.WorldSnapshot {
	alpha = clamp(alpha, 0, 1)

	snapshot := current
	snapshot.Player = interpolateVehicleSnapshot(previous.Player, current.Player, alpha)
	return snapshot
}

func interpolateVehicleSnapshot(previous, current physics.VehicleSnapshot, alpha float32) physics.VehicleSnapshot {
	snapshot := physics.VehicleSnapshot{
		ID:           current.ID,
		Position:     geom.LerpPlanar(previous.Position, current.Position, alpha),
		Velocity:     geom.LerpPlanar(previous.Velocity, current.Velocity, alpha),
		Heading:      lerpAngle(previous.Heading, current.Heading, alpha),
		Pitch:        lerp(previous.Pitch, current.Pitch, alpha),
		Roll:         lerp(previous.Roll, current.Roll, alpha),
		Speed:        lerp(previous.Speed, current.Speed, alpha),
		Height:       lerp(previous.Height, current.Height, alpha),
		GroundHeight: lerp(previous.GroundHeight, current.GroundHeight, alpha),
		SupportState: current.SupportState,
		SupportHits:  current.SupportHits,
		Length:       current.Length,
		Width:        current.Width,
		IsPlayer:     current.IsPlayer,
	}

	for index := range current.Wheels {
		snapshot.Wheels[index] = interpolateWheelSnapshot(previous.Wheels[index], current.Wheels[index], alpha)
	}

	return snapshot
}

func interpolateWheelSnapshot(previous, current physics.WheelSnapshot, alpha float32) physics.WheelSnapshot {
	return physics.WheelSnapshot{
		Label:            current.Label,
		MountPoint:       geom.LerpVec3(previous.MountPoint, current.MountPoint, alpha),
		Hit:              current.Hit,
		HitPoint:         geom.LerpVec3(previous.HitPoint, current.HitPoint, alpha),
		WheelCenter:      geom.LerpVec3(previous.WheelCenter, current.WheelCenter, alpha),
		HitDistance:      lerp(previous.HitDistance, current.HitDistance, alpha),
		SuspensionLength: lerp(previous.SuspensionLength, current.SuspensionLength, alpha),
		Compression:      lerp(previous.Compression, current.Compression, alpha),
	}
}

func vehiclePoint(vehicle physics.VehicleSnapshot, localX, localY, localZ float32) rl.Vector3 {
	center := toRenderVector(vehicle.Position, vehicle.Height+chassisH*0.5)
	forward, right, up := vehicleAxes(vehicle)
	point := center
	point = rl.Vector3Add(point, rl.Vector3Scale(forward, localX))
	point = rl.Vector3Add(point, rl.Vector3Scale(up, localY))
	point = rl.Vector3Add(point, rl.Vector3Scale(right, localZ))
	return point
}

func vehicleAxes(vehicle physics.VehicleSnapshot) (rl.Vector3, rl.Vector3, rl.Vector3) {
	forward := headingForward3D(vehicle.Heading)
	right := rl.NewVector3(-forward.Z, 0, forward.X)
	up := rl.NewVector3(0, 1, 0)

	forward = rotateAroundAxis(forward, right, vehicle.Pitch)
	up = rotateAroundAxis(up, right, vehicle.Pitch)
	right = rotateAroundAxis(right, forward, vehicle.Roll)
	up = rotateAroundAxis(up, forward, vehicle.Roll)

	return rl.Vector3Normalize(forward), rl.Vector3Normalize(right), rl.Vector3Normalize(up)
}

func rotateAroundAxis(vector rl.Vector3, axis rl.Vector3, angle float32) rl.Vector3 {
	axis = rl.Vector3Normalize(axis)
	cosAngle := float32(math.Cos(float64(angle)))
	sinAngle := float32(math.Sin(float64(angle)))

	term1 := rl.Vector3Scale(vector, cosAngle)
	term2 := rl.Vector3Scale(rl.Vector3CrossProduct(axis, vector), sinAngle)
	term3 := rl.Vector3Scale(axis, rl.Vector3DotProduct(axis, vector)*(1-cosAngle))

	return rl.Vector3Add(rl.Vector3Add(term1, term2), term3)
}

func lerp(a, b, t float32) float32 {
	return a + (b-a)*t
}

func lerpAngle(from, to, t float32) float32 {
	delta := normalizeAngle(to - from)
	return normalizeAngle(from + delta*t)
}

func normalizeAngle(angle float32) float32 {
	for angle > math.Pi {
		angle -= 2 * math.Pi
	}
	for angle < -math.Pi {
		angle += 2 * math.Pi
	}
	return angle
}
