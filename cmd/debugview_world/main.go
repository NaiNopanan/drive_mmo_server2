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
	baseTarget := rl.Vector3Add(toRenderVector(player.Position, player.Height+player.BodyHeight*0.5), rl.NewVector3(0, 1.2, 0))
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
	fillColor := rl.NewColor(86, 194, 255, 110)
	lineColor := rl.NewColor(86, 194, 255, 255)
	if snapshot.Player.BodyHitMap {
		fillColor = rl.NewColor(255, 164, 64, 140)
		lineColor = rl.NewColor(255, 164, 64, 255)
	}
	renderer.DrawVehicleCollider(snapshot.Player, fillColor, lineColor)
	drawKinematicDebug(snapshot.Player)
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
	start, end := capsuleEndpoints(vehicle.Position, vehicle.Height, vehicle.Heading, vehicle.Pitch, vehicle.Roll, vehicle.ColliderRadius, vehicle.ColliderHalfLength)
	rl.DrawSphere(start, vehicle.ColliderRadius, fillColor)
	rl.DrawSphere(end, vehicle.ColliderRadius, fillColor)
	drawCapsuleLongitudinalLines(start, end, vehicle, lineColor)

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

func drawKinematicDebug(vehicle physics.VehicleSnapshot) {
	center := capsuleCenter(vehicle.Position, vehicle.Height, vehicle.Heading, vehicle.Pitch, vehicle.Roll, vehicle.ColliderRadius, vehicle.ColliderHalfLength)
	drawDebugVector(center, vehicle.Kinematic.ForwardVector, 1.8, rl.NewColor(255, 255, 255, 255))
	drawDebugVector(center, vehicle.Kinematic.RightVector, 1.2, rl.NewColor(192, 192, 192, 220))
	drawDebugVector(center, vehicle.Kinematic.PlanarVelocity, 0.16, rl.NewColor(255, 96, 96, 255))
	drawDebugVector(center, vehicle.Kinematic.ProjectedVelocity, 0.16, rl.NewColor(96, 224, 255, 255))
	drawDebugVector(center, vehicle.Kinematic.MoveDelta, 8.0, rl.NewColor(255, 210, 96, 255))

	if vehicle.Kinematic.Grounded && vehicle.Kinematic.GroundNormal.LengthSquared() > 0 {
		drawDebugVector(center, vehicle.Kinematic.GroundNormal, 1.2, rl.NewColor(120, 255, 170, 255))
	}

	if vehicle.Kinematic.ContactCount > 0 && vehicle.Kinematic.ContactNormal.LengthSquared() > 0 {
		rl.DrawSphere(center, 0.06, rl.NewColor(255, 232, 96, 255))
		drawDebugVector(center, vehicle.Kinematic.ContactNormal, 1.4, rl.NewColor(255, 96, 96, 255))
	}

	if vehicle.Kinematic.GroundProbeHit {
		origin := toRenderVector3(vehicle.Kinematic.GroundProbeOrigin)
		point := toRenderVector3(vehicle.Kinematic.GroundProbePoint)
		rl.DrawLine3D(origin, point, rl.NewColor(96, 255, 220, 220))
		rl.DrawSphere(point, 0.04, rl.NewColor(96, 255, 220, 255))
		drawDebugVector(point, vehicle.Kinematic.GroundProbeNormal, 0.9, rl.NewColor(96, 255, 220, 255))
	}

	if vehicle.Kinematic.LookAheadHit {
		origin := toRenderVector3(vehicle.Kinematic.LookAheadOrigin)
		point := toRenderVector3(vehicle.Kinematic.LookAheadPoint)
		rl.DrawLine3D(origin, point, rl.NewColor(96, 160, 255, 220))
		rl.DrawSphere(point, 0.04, rl.NewColor(96, 160, 255, 255))
		drawDebugVector(point, vehicle.Kinematic.LookAheadNormal, 0.9, rl.NewColor(96, 160, 255, 255))
	}

	if vehicle.Kinematic.SnapApplied && vehicle.Kinematic.GroundProbeHit {
		point := toRenderVector3(vehicle.Kinematic.GroundProbePoint)
		rl.DrawSphere(point, 0.08, rl.NewColor(255, 240, 96, 120))
	}
}

func drawDebugVector(origin rl.Vector3, vector geom.Vec3, scale float32, color rl.Color) {
	if vector.LengthSquared() == 0 {
		return
	}

	end := rl.Vector3Add(origin, rl.NewVector3(vector.X*scale, vector.Y*scale, vector.Z*scale))
	rl.DrawLine3D(origin, end, color)
	rl.DrawSphere(end, 0.04, color)
}

func chassisCorners(vehicle physics.VehicleSnapshot) [8]rl.Vector3 {
	return poseCorners(vehicle.Length, vehicle.Width, vehicle.Position, vehicle.Height, vehicle.Heading, vehicle.Pitch, vehicle.Roll)
}

func drawCapsuleLongitudinalLines(start, end rl.Vector3, vehicle physics.VehicleSnapshot, color rl.Color) {
	_, right, up := vehicleAxes(vehicle)
	offsets := []rl.Vector3{
		rl.Vector3Scale(right, vehicle.ColliderRadius),
		rl.Vector3Scale(right, -vehicle.ColliderRadius),
		rl.Vector3Scale(up, vehicle.ColliderRadius),
		rl.Vector3Scale(up, -vehicle.ColliderRadius),
	}
	for _, offset := range offsets {
		rl.DrawLine3D(rl.Vector3Add(start, offset), rl.Vector3Add(end, offset), color)
	}
	rl.DrawLine3D(start, end, color)
}

func poseCorners(length, width float32, position geom.PlanarVec, height, heading, pitch, roll float32) [8]rl.Vector3 {
	halfLength := length * 0.5
	halfWidth := width * 0.5
	halfHeight := float32(chassisH * 0.5)
	centerHeight := height + chassisH*0.5

	var corners [8]rl.Vector3
	corners[0] = posePoint(position, centerHeight, heading, pitch, roll, halfLength, halfHeight, halfWidth)
	corners[1] = posePoint(position, centerHeight, heading, pitch, roll, halfLength, halfHeight, -halfWidth)
	corners[2] = posePoint(position, centerHeight, heading, pitch, roll, -halfLength, halfHeight, -halfWidth)
	corners[3] = posePoint(position, centerHeight, heading, pitch, roll, -halfLength, halfHeight, halfWidth)
	corners[4] = posePoint(position, centerHeight, heading, pitch, roll, halfLength, -halfHeight, halfWidth)
	corners[5] = posePoint(position, centerHeight, heading, pitch, roll, halfLength, -halfHeight, -halfWidth)
	corners[6] = posePoint(position, centerHeight, heading, pitch, roll, -halfLength, -halfHeight, -halfWidth)
	corners[7] = posePoint(position, centerHeight, heading, pitch, roll, -halfLength, -halfHeight, halfWidth)
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
	roofY := float32(vehicle.BodyHeight*0.5 + 0.05)

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
		"controller: kinematic capsule",
		fmt.Sprintf("speed: %.1f", snapshot.Player.Speed),
		fmt.Sprintf("vertical vel: %.2f", snapshot.Player.VerticalVel),
		fmt.Sprintf("car size: %.1fm x %.1fm", snapshot.Player.Length, snapshot.Player.Width),
		fmt.Sprintf("height: %.2fm / ground: %.2fm", snapshot.Player.Height, snapshot.Player.GroundHeight),
		fmt.Sprintf("grounded: %t", snapshot.Player.Kinematic.Grounded),
		fmt.Sprintf("support: %s (%d contacts)", snapshot.Player.SupportState, snapshot.Player.Kinematic.ContactCount),
		fmt.Sprintf("input: throttle=%.1f brake=%.1f steer=%.1f", snapshot.Player.Kinematic.Throttle, snapshot.Player.Kinematic.Brake, snapshot.Player.Kinematic.Steering),
		fmt.Sprintf("heading: %.2f", snapshot.Player.Heading),
		fmt.Sprintf("pitch/roll: %.2f / %.2f", snapshot.Player.Pitch, snapshot.Player.Roll),
		fmt.Sprintf("position: [%.1f, %.1f]", snapshot.Player.Position.X, snapshot.Player.Position.Z),
		fmt.Sprintf("world triangles: %d", len(snapshot.StaticMesh.Triangles)),
		fmt.Sprintf("world size: %.1fm x %.1fm", snapshot.StaticMesh.Max.X-snapshot.StaticMesh.Min.X, snapshot.StaticMesh.Max.Z-snapshot.StaticMesh.Min.Z),
	}
	lines = append(lines, "vectors: white=forward gray=right red=planar cyan=projected orange=move")
	if snapshot.Player.Kinematic.ContactCount > 0 {
		lines = append(lines, fmt.Sprintf(
			"contact normal: [%.2f %.2f %.2f]",
			snapshot.Player.Kinematic.ContactNormal.X,
			snapshot.Player.Kinematic.ContactNormal.Y,
			snapshot.Player.Kinematic.ContactNormal.Z,
		))
	} else {
		lines = append(lines, "contact normal: none")
	}
	if snapshot.Player.Kinematic.Grounded {
		lines = append(lines, fmt.Sprintf(
			"ground normal: [%.2f %.2f %.2f]",
			snapshot.Player.Kinematic.GroundNormal.X,
			snapshot.Player.Kinematic.GroundNormal.Y,
			snapshot.Player.Kinematic.GroundNormal.Z,
		))
	} else {
		lines = append(lines, "ground normal: airborne")
	}
	if snapshot.Player.Kinematic.GroundProbeHit {
		lines = append(lines, fmt.Sprintf(
			"ground probe: hit y=%.2f n=[%.2f %.2f %.2f]",
			snapshot.Player.Kinematic.GroundProbePoint.Y,
			snapshot.Player.Kinematic.GroundProbeNormal.X,
			snapshot.Player.Kinematic.GroundProbeNormal.Y,
			snapshot.Player.Kinematic.GroundProbeNormal.Z,
		))
	} else {
		lines = append(lines, "ground probe: miss")
	}
	if snapshot.Player.Kinematic.LookAheadHit {
		lines = append(lines, fmt.Sprintf(
			"look-ahead: hit y=%.2f n=[%.2f %.2f %.2f]",
			snapshot.Player.Kinematic.LookAheadPoint.Y,
			snapshot.Player.Kinematic.LookAheadNormal.X,
			snapshot.Player.Kinematic.LookAheadNormal.Y,
			snapshot.Player.Kinematic.LookAheadNormal.Z,
		))
	} else {
		lines = append(lines, "look-ahead: miss")
	}
	lines = append(lines, fmt.Sprintf("snap applied: %t", snapshot.Player.Kinematic.SnapApplied))

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
		ID:                 current.ID,
		Position:           geom.LerpPlanar(previous.Position, current.Position, alpha),
		Velocity:           geom.LerpPlanar(previous.Velocity, current.Velocity, alpha),
		Heading:            lerpAngle(previous.Heading, current.Heading, alpha),
		Pitch:              lerp(previous.Pitch, current.Pitch, alpha),
		Roll:               lerp(previous.Roll, current.Roll, alpha),
		Speed:              lerp(previous.Speed, current.Speed, alpha),
		Height:             lerp(previous.Height, current.Height, alpha),
		GroundHeight:       lerp(previous.GroundHeight, current.GroundHeight, alpha),
		VerticalVel:        lerp(previous.VerticalVel, current.VerticalVel, alpha),
		BodyHitMap:         current.BodyHitMap,
		OBBCCD:             interpolateOBBCCDDebug(previous.OBBCCD, current.OBBCCD, alpha),
		Kinematic:          interpolateKinematicDebug(previous.Kinematic, current.Kinematic, alpha),
		SupportState:       current.SupportState,
		SupportHits:        current.SupportHits,
		Length:             current.Length,
		Width:              current.Width,
		BodyHeight:         current.BodyHeight,
		ColliderRadius:     current.ColliderRadius,
		ColliderHalfLength: current.ColliderHalfLength,
		IsPlayer:           current.IsPlayer,
	}

	for index := range current.Wheels {
		snapshot.Wheels[index] = interpolateWheelSnapshot(previous.Wheels[index], current.Wheels[index], alpha)
	}

	return snapshot
}

func interpolateKinematicDebug(previous, current physics.KinematicDebug, alpha float32) physics.KinematicDebug {
	contactNormal := geom.LerpVec3(previous.ContactNormal, current.ContactNormal, alpha)
	if contactNormal.LengthSquared() > 0 {
		contactNormal = contactNormal.Normalize()
	}

	groundNormal := geom.LerpVec3(previous.GroundNormal, current.GroundNormal, alpha)
	if groundNormal.LengthSquared() > 0 {
		groundNormal = groundNormal.Normalize()
	}

	groundProbeNormal := geom.LerpVec3(previous.GroundProbeNormal, current.GroundProbeNormal, alpha)
	if groundProbeNormal.LengthSquared() > 0 {
		groundProbeNormal = groundProbeNormal.Normalize()
	}

	lookAheadNormal := geom.LerpVec3(previous.LookAheadNormal, current.LookAheadNormal, alpha)
	if lookAheadNormal.LengthSquared() > 0 {
		lookAheadNormal = lookAheadNormal.Normalize()
	}

	return physics.KinematicDebug{
		Grounded:          current.Grounded,
		ContactCount:      current.ContactCount,
		Substeps:          current.Substeps,
		Throttle:          current.Throttle,
		Brake:             current.Brake,
		Steering:          current.Steering,
		ForwardVector:     geom.LerpVec3(previous.ForwardVector, current.ForwardVector, alpha),
		RightVector:       geom.LerpVec3(previous.RightVector, current.RightVector, alpha),
		PlanarVelocity:    geom.LerpVec3(previous.PlanarVelocity, current.PlanarVelocity, alpha),
		ProjectedVelocity: geom.LerpVec3(previous.ProjectedVelocity, current.ProjectedVelocity, alpha),
		MoveDelta:         geom.LerpVec3(previous.MoveDelta, current.MoveDelta, alpha),
		ContactNormal:     contactNormal,
		GroundNormal:      groundNormal,
		GroundProbeHit:    current.GroundProbeHit,
		GroundProbeOrigin: geom.LerpVec3(previous.GroundProbeOrigin, current.GroundProbeOrigin, alpha),
		GroundProbePoint:  geom.LerpVec3(previous.GroundProbePoint, current.GroundProbePoint, alpha),
		GroundProbeNormal: groundProbeNormal,
		LookAheadHit:      current.LookAheadHit,
		LookAheadOrigin:   geom.LerpVec3(previous.LookAheadOrigin, current.LookAheadOrigin, alpha),
		LookAheadPoint:    geom.LerpVec3(previous.LookAheadPoint, current.LookAheadPoint, alpha),
		LookAheadNormal:   lookAheadNormal,
		SnapApplied:       current.SnapApplied,
	}
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

func interpolateOBBCCDDebug(previous, current physics.OBBCCDDebug, alpha float32) physics.OBBCCDDebug {
	normal := geom.LerpVec3(previous.Normal, current.Normal, alpha)
	if normal.LengthSquared() > 0 {
		normal = normal.Normalize()
	}
	return physics.OBBCCDDebug{
		Hit:      current.Hit,
		Time:     lerp(previous.Time, current.Time, alpha),
		Position: geom.LerpPlanar(previous.Position, current.Position, alpha),
		Height:   lerp(previous.Height, current.Height, alpha),
		Heading:  lerpAngle(previous.Heading, current.Heading, alpha),
		Pitch:    lerp(previous.Pitch, current.Pitch, alpha),
		Roll:     lerp(previous.Roll, current.Roll, alpha),
		Normal:   normal,
	}
}

func vehiclePoint(vehicle physics.VehicleSnapshot, localX, localY, localZ float32) rl.Vector3 {
	return posePoint(vehicle.Position, vehicle.Height+vehicle.BodyHeight*0.5, vehicle.Heading, vehicle.Pitch, vehicle.Roll, localX, localY, localZ)
}

func capsuleCenter(position geom.PlanarVec, height, heading, pitch, roll, radius, halfLength float32) rl.Vector3 {
	forward, _, _ := poseAxes(heading, pitch, roll)
	centerHeight := height + radius + float32(math.Abs(float64(forward.Y)))*halfLength
	return posePoint(position, centerHeight, heading, pitch, roll, 0, 0, 0)
}

func capsuleEndpoints(position geom.PlanarVec, height, heading, pitch, roll, radius, halfLength float32) (rl.Vector3, rl.Vector3) {
	forward, _, _ := poseAxes(heading, pitch, roll)
	centerHeight := height + radius + float32(math.Abs(float64(forward.Y)))*halfLength
	return posePoint(position, centerHeight, heading, pitch, roll, halfLength, 0, 0),
		posePoint(position, centerHeight, heading, pitch, roll, -halfLength, 0, 0)
}

func posePoint(position geom.PlanarVec, centerHeight, heading, pitch, roll, localX, localY, localZ float32) rl.Vector3 {
	center := toRenderVector(position, centerHeight)
	forward, right, up := poseAxes(heading, pitch, roll)
	point := center
	point = rl.Vector3Add(point, rl.Vector3Scale(forward, localX))
	point = rl.Vector3Add(point, rl.Vector3Scale(up, localY))
	point = rl.Vector3Add(point, rl.Vector3Scale(right, localZ))
	return point
}

func vehicleAxes(vehicle physics.VehicleSnapshot) (rl.Vector3, rl.Vector3, rl.Vector3) {
	return poseAxes(vehicle.Heading, vehicle.Pitch, vehicle.Roll)
}

func poseAxes(heading, pitch, roll float32) (rl.Vector3, rl.Vector3, rl.Vector3) {
	forward := headingForward3D(heading)
	right := rl.NewVector3(-forward.Z, 0, forward.X)
	up := rl.NewVector3(0, 1, 0)

	forward = rotateAroundAxis(forward, right, pitch)
	up = rotateAroundAxis(up, right, pitch)
	right = rotateAroundAxis(right, forward, roll)
	up = rotateAroundAxis(up, forward, roll)

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
