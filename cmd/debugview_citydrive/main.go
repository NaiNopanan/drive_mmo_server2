package main

import (
	"fmt"
	"math"

	rl "github.com/gen2brain/raylib-go/raylib"

	"server2/internal/fixed"
	"server2/internal/geom"
	"server2/internal/sim"
)

type FlyCamera struct {
	Position  rl.Vector3
	YawDeg    float32
	PitchDeg  float32
	MoveSpeed float32
	LookSpeed float32
}

func (c *FlyCamera) Forward() rl.Vector3 {
	yaw := float64(c.YawDeg) * math.Pi / 180.0
	pitch := float64(c.PitchDeg) * math.Pi / 180.0

	x := float32(math.Cos(pitch) * math.Sin(yaw))
	y := float32(math.Sin(pitch))
	z := float32(math.Cos(pitch) * math.Cos(yaw))

	return rl.NewVector3(x, y, z)
}

func (c *FlyCamera) Right() rl.Vector3 {
	f := c.Forward()
	up := rl.NewVector3(0, 1, 0)

	rx := f.Y*up.Z - f.Z*up.Y
	ry := f.Z*up.X - f.X*up.Z
	rz := f.X*up.Y - f.Y*up.X

	length := float32(math.Sqrt(float64(rx*rx + ry*ry + rz*rz)))
	if length == 0 {
		return rl.NewVector3(1, 0, 0)
	}

	return rl.NewVector3(rx/length, ry/length, rz/length)
}

func (c *FlyCamera) Update(dt float32) {
	turn := c.LookSpeed * dt
	move := c.MoveSpeed * dt

	if rl.IsKeyDown(rl.KeyLeftShift) {
		move *= 3.0
	}

	if rl.IsKeyDown(rl.KeyJ) {
		c.YawDeg -= turn
	}
	if rl.IsKeyDown(rl.KeyL) {
		c.YawDeg += turn
	}
	if rl.IsKeyDown(rl.KeyI) {
		c.PitchDeg += turn
	}
	if rl.IsKeyDown(rl.KeyK) {
		c.PitchDeg -= turn
	}

	if c.PitchDeg > 89 {
		c.PitchDeg = 89
	}
	if c.PitchDeg < -89 {
		c.PitchDeg = -89
	}

	f := c.Forward()
	r := c.Right()

	if rl.IsKeyDown(rl.KeyW) {
		c.Position.X += f.X * move
		c.Position.Y += f.Y * move
		c.Position.Z += f.Z * move
	}
	if rl.IsKeyDown(rl.KeyS) {
		c.Position.X -= f.X * move
		c.Position.Y -= f.Y * move
		c.Position.Z -= f.Z * move
	}
	if rl.IsKeyDown(rl.KeyA) {
		c.Position.X -= r.X * move
		c.Position.Y -= r.Y * move
		c.Position.Z -= r.Z * move
	}
	if rl.IsKeyDown(rl.KeyD) {
		c.Position.X += r.X * move
		c.Position.Y += r.Y * move
		c.Position.Z += r.Z * move
	}
	if rl.IsKeyDown(rl.KeyQ) {
		c.Position.Y -= move
	}
	if rl.IsKeyDown(rl.KeyE) {
		c.Position.Y += move
	}
}

func (c *FlyCamera) ToCamera3D() rl.Camera3D {
	f := c.Forward()
	target := rl.NewVector3(
		c.Position.X+f.X,
		c.Position.Y+f.Y,
		c.Position.Z+f.Z,
	)

	return rl.Camera3D{
		Position:   c.Position,
		Target:     target,
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       60,
		Projection: rl.CameraPerspective,
	}
}

type ChaseCamera struct {
	Position    rl.Vector3
	Target      rl.Vector3
	Initialized bool
}

func (c *ChaseCamera) Snap(v sim.Vehicle) {
	c.Position = chasePosition(v)
	c.Target = chaseTarget(v)
	c.Initialized = true
}

func (c *ChaseCamera) Update(v sim.Vehicle) {
	desiredPos := chasePosition(v)
	desiredTarget := chaseTarget(v)

	if !c.Initialized {
		c.Position = desiredPos
		c.Target = desiredTarget
		c.Initialized = true
		return
	}

	c.Position = lerpVector3(c.Position, desiredPos, 0.12)
	c.Target = lerpVector3(c.Target, desiredTarget, 0.18)
}

func (c *ChaseCamera) ToCamera3D() rl.Camera3D {
	return rl.Camera3D{
		Position:   c.Position,
		Target:     c.Target,
		Up:         rl.NewVector3(0, 1, 0),
		Fovy:       60,
		Projection: rl.CameraPerspective,
	}
}

func chasePosition(v sim.Vehicle) rl.Vector3 {
	up := vecToRL(v.UpWS.Scale(fixed.FromInt(4)))
	back := vecToRL(v.ForwardWS.Scale(fixed.FromInt(12)).Neg())
	body := vecToRL(v.Position)
	return rl.NewVector3(body.X+up.X+back.X, body.Y+up.Y+back.Y, body.Z+up.Z+back.Z)
}

func chaseTarget(v sim.Vehicle) rl.Vector3 {
	lookAhead := vecToRL(v.ForwardWS.Scale(fixed.FromInt(6)))
	up := vecToRL(v.UpWS.Scale(fixed.FromFraction(6, 5)))
	body := vecToRL(v.Position)
	return rl.NewVector3(body.X+lookAhead.X+up.X, body.Y+lookAhead.Y+up.Y, body.Z+lookAhead.Z+up.Z)
}

func fixedToF(v fixed.Fixed) float32 {
	return float32(float64(v.Raw()) / (1 << 32))
}

func vecToRL(v geom.Vec3) rl.Vector3 {
	return rl.NewVector3(fixedToF(v.X), fixedToF(v.Y), fixedToF(v.Z))
}

func lerpVector3(current, target rl.Vector3, alpha float32) rl.Vector3 {
	return rl.NewVector3(
		current.X+(target.X-current.X)*alpha,
		current.Y+(target.Y-current.Y)*alpha,
		current.Z+(target.Z-current.Z)*alpha,
	)
}

func approachFixed(current, target, maxDelta fixed.Fixed) fixed.Fixed {
	if current.Cmp(target) < 0 {
		next := current.Add(maxDelta)
		if next.Cmp(target) > 0 {
			return target
		}
		return next
	}
	if current.Cmp(target) > 0 {
		next := current.Sub(maxDelta)
		if next.Cmp(target) < 0 {
			return target
		}
		return next
	}
	return current
}

func drawTriangleFill(t geom.Triangle, color rl.Color) {
	rl.DrawTriangle3D(vecToRL(t.A), vecToRL(t.B), vecToRL(t.C), color)
}

func drawObstacleBox(obstacle sim.CityObstacle, color rl.Color) {
	rl.DrawCubeV(vecToRL(obstacle.Center()), vecToRL(obstacle.Size()), color)
	rl.DrawCubeWiresV(vecToRL(obstacle.Center()), vecToRL(obstacle.Size()), rl.Fade(rl.Black, 0.35))
}

func drawVehicleChassis(v sim.Vehicle, color rl.Color) {
	c := v.Position
	hx := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	hy := fixed.FromFraction(25, 100)
	hz := v.Tuning.WheelBase.Div(fixed.FromInt(2))

	forward := v.ForwardWS
	right := v.RightWS
	up := v.UpWS

	corners := [8]geom.Vec3{
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz.Neg())),
		c.Add(right.Scale(hx)).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz.Neg())),
		c.Add(right.Scale(hx)).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz)),
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz)),
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy)).Add(forward.Scale(hz.Neg())),
		c.Add(right.Scale(hx)).Add(up.Scale(hy)).Add(forward.Scale(hz.Neg())),
		c.Add(right.Scale(hx)).Add(up.Scale(hy)).Add(forward.Scale(hz)),
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy)).Add(forward.Scale(hz)),
	}

	edges := [][2]int{
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	}

	for _, edge := range edges {
		rl.DrawLine3D(vecToRL(corners[edge[0]]), vecToRL(corners[edge[1]]), color)
	}
}

func drawVehicle(v sim.Vehicle) {
	drawVehicleChassis(v, rl.DarkBlue)

	bodyPos := vecToRL(v.Position)
	fwdEnd := vecToRL(v.Position.Add(v.ForwardWS.Scale(fixed.FromInt(3))))
	rl.DrawLine3D(bodyPos, fwdEnd, rl.Green)

	for wi := range v.Wheels {
		w := v.Wheels[wi]
		def := v.WheelDefs[wi]

		anchorWS := v.Position.Add(v.LocalToWorld(def.LocalAnchor))
		rayOrigin := anchorWS.Add(geom.V3(fixed.Zero, v.Tuning.SuspensionMaxRaise, fixed.Zero))
		center := rayOrigin.Add(geom.V3(fixed.Zero, w.ContactDistance.Neg(), fixed.Zero))

		rl.DrawLine3D(vecToRL(anchorWS), vecToRL(center), rl.Gray)
		rl.DrawSphereWires(vecToRL(center), fixedToF(v.Tuning.WheelRadius), 12, 12, rl.DarkGray)

		if w.InContact {
			rl.DrawSphere(vecToRL(w.ContactPoint), 0.08, rl.Gold)
			rl.DrawLine3D(vecToRL(center), vecToRL(center.Add(w.WheelForwardWS.Scale(fixed.FromInt(1)))), rl.Blue)
		}
	}
}

func drawGround(bounds sim.WallBounds) {
	y := float32(-0.04)
	minX := fixedToF(bounds.MinX)
	maxX := fixedToF(bounds.MaxX)
	minZ := fixedToF(bounds.MinZ)
	maxZ := fixedToF(bounds.MaxZ)

	a := rl.NewVector3(minX, y, minZ)
	b := rl.NewVector3(maxX, y, minZ)
	c := rl.NewVector3(minX, y, maxZ)
	d := rl.NewVector3(maxX, y, maxZ)

	rl.DrawTriangle3D(a, c, b, rl.Color{R: 152, G: 177, B: 134, A: 255})
	rl.DrawTriangle3D(c, d, b, rl.Color{R: 152, G: 177, B: 134, A: 255})

	borderY := float32(0.06)
	rl.DrawLine3D(rl.NewVector3(minX, borderY, minZ), rl.NewVector3(maxX, borderY, minZ), rl.Maroon)
	rl.DrawLine3D(rl.NewVector3(maxX, borderY, minZ), rl.NewVector3(maxX, borderY, maxZ), rl.Maroon)
	rl.DrawLine3D(rl.NewVector3(maxX, borderY, maxZ), rl.NewVector3(minX, borderY, maxZ), rl.Maroon)
	rl.DrawLine3D(rl.NewVector3(minX, borderY, maxZ), rl.NewVector3(minX, borderY, minZ), rl.Maroon)
}

func drawRoadSurfaces(city sim.CityMap) {
	asphalt := rl.Color{R: 42, G: 45, B: 52, A: 255}
	for i := range city.RoadSurfaces {
		drawTriangleFill(city.RoadSurfaces[i], asphalt)
	}
}

func drawLaneHints(city sim.CityMap) {
	for i := range city.LanePaths {
		drawDashedPath(city.LanePaths[i].Points, rl.Color{R: 248, G: 226, B: 126, A: 255})
	}
}

func drawDashedPath(points []geom.Vec3, color rl.Color) {
	const dashLen = float32(2.8)
	const gapLen = float32(1.8)

	for i := 0; i+1 < len(points); i++ {
		a := vecToRL(points[i])
		b := vecToRL(points[i+1])
		dx := b.X - a.X
		dy := b.Y - a.Y
		dz := b.Z - a.Z
		length := float32(math.Sqrt(float64(dx*dx + dy*dy + dz*dz)))
		if length == 0 {
			continue
		}

		ux := dx / length
		uy := dy / length
		uz := dz / length

		for dist := float32(0); dist < length; dist += dashLen + gapLen {
			start := dist
			end := dist + dashLen
			if end > length {
				end = length
			}

			s := rl.NewVector3(a.X+ux*start, a.Y+uy*start+0.03, a.Z+uz*start)
			e := rl.NewVector3(a.X+ux*end, a.Y+uy*end+0.03, a.Z+uz*end)
			rl.DrawLine3D(s, e, color)
		}
	}
}

func drawCity(city sim.CityMap) {
	drawGround(city.Bounds)

	for i := range city.Sidewalks {
		drawObstacleBox(city.Sidewalks[i], rl.Color{R: 210, G: 206, B: 196, A: 255})
	}

	drawRoadSurfaces(city)
	drawLaneHints(city)

	for i := range city.Buildings {
		drawObstacleBox(city.Buildings[i], rl.Color{R: 120, G: 138, B: 160, A: 255})
	}
	for i := range city.GuardRails {
		drawObstacleBox(city.GuardRails[i], rl.Color{R: 184, G: 93, B: 72, A: 255})
	}
}

func main() {
	const screenWidth = 1480
	const screenHeight = 920

	rl.InitWindow(screenWidth, screenHeight, "server2 City Drive Debug Viewer")
	defer rl.CloseWindow()

	rl.SetTargetFPS(60)

	world := sim.NewCityWorld()
	dtFixed := fixed.FromFraction(1, 60)

	flycam := FlyCamera{
		Position:  rl.NewVector3(0, 18, -28),
		YawDeg:    0,
		PitchDeg:  -18,
		MoveSpeed: 18,
		LookSpeed: 90,
	}
	chaseCam := ChaseCamera{}
	chaseCam.Snap(world.Vehicle)

	useChaseCamera := true
	paused := false
	stepOnce := false
	invertSteerInput := true

	resetWorld := func() {
		world.Reset()
		chaseCam.Snap(world.Vehicle)
	}

	for !rl.WindowShouldClose() {
		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyR) {
			resetWorld()
		}
		if rl.IsKeyPressed(rl.KeyC) {
			useChaseCamera = !useChaseCamera
			if useChaseCamera {
				chaseCam.Snap(world.Vehicle)
			}
		}
		if rl.IsKeyPressed(rl.KeyV) {
			invertSteerInput = !invertSteerInput
		}

		if !useChaseCamera {
			flycam.Update(1.0 / 60.0)
		}

		target := sim.VehicleInput{}
		upDown := rl.IsKeyDown(rl.KeyUp)
		downDown := rl.IsKeyDown(rl.KeyDown)
		leftDown := rl.IsKeyDown(rl.KeyLeft)
		rightDown := rl.IsKeyDown(rl.KeyRight)

		if upDown && !downDown {
			target.Throttle = fixed.One
		} else if downDown && !upDown {
			target.Throttle = fixed.FromFraction(3, 5).Neg()
		}

		leftSteer := fixed.One.Neg()
		rightSteer := fixed.One
		if invertSteerInput {
			leftSteer = fixed.One
			rightSteer = fixed.One.Neg()
		}
		if leftDown && !rightDown {
			target.Steer = leftSteer
		} else if rightDown && !leftDown {
			target.Steer = rightSteer
		}

		if rl.IsKeyDown(rl.KeyLeftControl) || rl.IsKeyDown(rl.KeyRightControl) {
			target.Brake = fixed.One
		}

		in := &world.Vehicle.Input
		in.Throttle = approachFixed(in.Throttle, target.Throttle, fixed.FromFraction(1, 10))
		in.Steer = approachFixed(in.Steer, target.Steer, fixed.FromFraction(3, 20))
		in.Brake = approachFixed(in.Brake, target.Brake, fixed.FromFraction(1, 5))

		if !paused || stepOnce {
			world.Step(dtFixed)
			stepOnce = false
		}

		chaseCam.Update(world.Vehicle)

		camera := flycam.ToCamera3D()
		cameraModeLabel := "Flycam"
		if useChaseCamera {
			camera = chaseCam.ToCamera3D()
			cameraModeLabel = "Chase"
		}

		rl.BeginDrawing()
		rl.ClearBackground(rl.Color{R: 224, G: 235, B: 245, A: 255})

		rl.BeginMode3D(camera)
		drawCity(world.Map)
		drawVehicle(world.Vehicle)
		rl.EndMode3D()

		v := world.Vehicle
		rl.DrawRectangle(12, 12, 560, 236, rl.Fade(rl.White, 0.84))
		rl.DrawText("City Drive Debug", 22, 22, 28, rl.Black)
		rl.DrawText(fmt.Sprintf("Tick: %d", world.Tick), 22, 58, 18, rl.DarkGray)
		rl.DrawText(fmt.Sprintf("Camera: %s | Paused: %v", cameraModeLabel, paused), 22, 82, 18, rl.DarkGray)
		rl.DrawText(fmt.Sprintf("Pos: (%.2f, %.2f, %.2f)", fixedToF(v.Position.X), fixedToF(v.Position.Y), fixedToF(v.Position.Z)), 22, 110, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Vel: (%.2f, %.2f, %.2f)", fixedToF(v.Velocity.X), fixedToF(v.Velocity.Y), fixedToF(v.Velocity.Z)), 22, 136, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Speed: %.2f | Grounded wheels: %d | OnGround: %v", fixedToF(v.Velocity.Length()), v.GroundedWheels, v.OnGround), 22, 162, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Input T/B/S: %.2f %.2f %.2f | Invert steer: %v", fixedToF(v.Input.Throttle), fixedToF(v.Input.Brake), fixedToF(v.Input.Steer), invertSteerInput), 22, 188, 18, rl.Black)

		rl.DrawText("Drive: Up/Down forward+reverse, Left/Right steer, Ctrl brake", 22, 214, 18, rl.DarkBlue)
		rl.DrawText("Viewer: C camera, Space pause, N step, R reset, V invert steer", 22, 238, 18, rl.DarkBlue)
		rl.DrawText("Flycam: WASD/QE move, J/L yaw, I/K pitch, Shift speed up", 22, 262, 18, rl.DarkBlue)

		rl.EndDrawing()
	}
}
