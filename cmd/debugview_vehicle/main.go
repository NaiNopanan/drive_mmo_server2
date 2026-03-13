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

	// right = normalize(cross(forward, up))
	rx := f.Y*up.Z - f.Z*up.Y
	ry := f.Z*up.X - f.X*up.Z
	rz := f.X*up.Y - f.Y*up.X

	len := float32(math.Sqrt(float64(rx*rx + ry*ry + rz*rz)))
	if len == 0 {
		return rl.NewVector3(1, 0, 0)
	}

	return rl.NewVector3(rx/len, ry/len, rz/len)
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

func drawVehicleChassis(v sim.Vehicle, color rl.Color) {
	forward, right, up := sim.VehicleBasis(v)

	c := v.Body.Pos
	hx := v.Body.HalfSize.X
	hy := v.Body.HalfSize.Y
	hz := v.Body.HalfSize.Z

	// 8 corners
	corners := [8]geom.Vec3{
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz.Neg())), // 0
		c.Add(right.Scale(hx)).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz.Neg())),       // 1
		c.Add(right.Scale(hx)).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz)),              // 2
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy.Neg())).Add(forward.Scale(hz)),        // 3
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy)).Add(forward.Scale(hz.Neg())),        // 4
		c.Add(right.Scale(hx)).Add(up.Scale(hy)).Add(forward.Scale(hz.Neg())),              // 5
		c.Add(right.Scale(hx)).Add(up.Scale(hy)).Add(forward.Scale(hz)),                    // 6
		c.Add(right.Scale(hx.Neg())).Add(up.Scale(hy)).Add(forward.Scale(hz)),              // 7
	}

	edges := [][2]int{
		{0, 1}, {1, 2}, {2, 3}, {3, 0},
		{4, 5}, {5, 6}, {6, 7}, {7, 4},
		{0, 4}, {1, 5}, {2, 6}, {3, 7},
	}

	for _, e := range edges {
		rl.DrawLine3D(vecToRL(corners[e[0]]), vecToRL(corners[e[1]]), color)
	}
}

func drawVehicle(v sim.Vehicle, selected bool) {
	bodyColor := rl.DarkBlue
	wheelColor := rl.DarkGray
	contactColor := rl.Magenta

	if selected {
		bodyColor = rl.Red
		wheelColor = rl.Orange
		contactColor = rl.Gold
	}

	drawVehicleChassis(v, bodyColor)

	// forward vector
	bodyPos := vecToRL(v.Body.Pos)
	fwdEnd := vecToRL(v.Body.Pos.Add(v.Body.Forward.Scale(fixed.FromInt(3))))
	rl.DrawLine3D(bodyPos, fwdEnd, rl.Green)

	for wi := range v.Wheels {
		mount := sim.VehicleWheelMountWorld(v, wi)
		center := sim.VehicleWheelCenterWorld(v, wi)
		wheel := v.Wheels[wi]

		rl.DrawLine3D(vecToRL(mount), vecToRL(center), rl.Gray)
		rl.DrawSphereWires(vecToRL(center), fixedToFloat32(wheel.Radius), 12, 12, wheelColor)

		// mount point
		rl.DrawSphere(vecToRL(mount), 0.05, rl.Blue)

		if wheel.Contact {
			rl.DrawSphere(vecToRL(wheel.ContactPoint), 0.08, contactColor)

			nEnd := wheel.ContactPoint.Add(wheel.ContactNormal.Scale(fixed.FromInt(1)))
			rl.DrawLine3D(vecToRL(wheel.ContactPoint), vecToRL(nEnd), contactColor)
		}
	}
}

func makeVehicleWorld(flat bool) sim.VehicleWorld {
	var ground []geom.Triangle
	if flat {
		ground = sim.GroundFlatSmall()
	} else {
		ground = sim.GroundSlopeSmall()
	}

	vehicles := sim.SpawnVehicleGrid(3, 4, 10, 4)
	return sim.NewVehicleWorld(ground, vehicles)
}

func main() {
	const screenWidth = 1400
	const screenHeight = 900

	rl.InitWindow(screenWidth, screenHeight, "server2 Day 5 Vehicle Debug Viewer")
	defer rl.CloseWindow()

	rl.SetTargetFPS(60)

	cameraRig := FlyCamera{
		Position:  rl.NewVector3(0, 10, -18),
		YawDeg:    0,
		PitchDeg:  12,
		MoveSpeed: 12,
		LookSpeed: 80,
	}

	useFlatGround := true
	world := makeVehicleWorld(useFlatGround)

	selected := 0
	paused := false
	stepOnce := false

	for !rl.WindowShouldClose() {
		dt := 1.0 / 60.0

		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyR) {
			world = makeVehicleWorld(useFlatGround)
		}
		if rl.IsKeyPressed(rl.KeyOne) {
			useFlatGround = true
			world = makeVehicleWorld(useFlatGround)
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			useFlatGround = false
			world = makeVehicleWorld(useFlatGround)
		}
		if rl.IsKeyPressed(rl.KeyTab) {
			selected++
			if selected >= len(world.Vehicles) {
				selected = 0
			}
		}

		cameraRig.Update(float32(dt))

		// reset all inputs every frame
		for i := range world.Vehicles {
			world.Vehicles[i].Input = sim.VehicleInput{}
		}

		// simple deterministic autopilot for non-selected vehicles
		for i := range world.Vehicles {
			if i == selected {
				continue
			}

			switch i % 4 {
			case 0:
				world.Vehicles[i].Input.Throttle = fixed.FromFraction(3, 5)
			case 1:
				world.Vehicles[i].Input.Throttle = fixed.FromFraction(3, 5)
				world.Vehicles[i].Input.Steer = fixed.FromFraction(1, 4)
			case 2:
				world.Vehicles[i].Input.Throttle = fixed.FromFraction(3, 5)
				world.Vehicles[i].Input.Steer = fixed.FromFraction(-1, 4)
			default:
				if world.Tick%120 < 60 {
					world.Vehicles[i].Input.Brake = fixed.FromFraction(1, 3)
				}
			}
		}

		// user controls selected vehicle
		if selected >= 0 && selected < len(world.Vehicles) {
			if rl.IsKeyDown(rl.KeyUp) {
				world.Vehicles[selected].Input.Throttle = fixed.One
			}
			if rl.IsKeyDown(rl.KeyDown) {
				world.Vehicles[selected].Input.Brake = fixed.One
			}
			if rl.IsKeyDown(rl.KeyLeft) {
				world.Vehicles[selected].Input.Steer = fixed.One.Neg()
			}
			if rl.IsKeyDown(rl.KeyRight) {
				world.Vehicles[selected].Input.Steer = fixed.One
			}
		}

		if !paused || stepOnce {
			sim.StepVehicleWorld(&world)
			stepOnce = false
		}

		camera := cameraRig.ToCamera3D()

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode3D(camera)

		rl.DrawGrid(60, 1.0)

		for _, tri := range world.GroundTriangles {
			drawTriangle3D(tri, rl.Purple)
		}

		for i := range world.Vehicles {
			drawVehicle(world.Vehicles[i], i == selected)
		}

		rl.EndMode3D()

		// overlay
		rl.DrawRectangle(10, 10, 520, 310, rl.Fade(rl.SkyBlue, 0.18))

		state := "RUNNING"
		if paused {
			state = "PAUSED"
		}

		v := world.Vehicles[selected]
		px := fixedToFloat32(v.Body.Pos.X)
		py := fixedToFloat32(v.Body.Pos.Y)
		pz := fixedToFloat32(v.Body.Pos.Z)

		vx := fixedToFloat32(v.Body.Vel.X)
		vy := fixedToFloat32(v.Body.Vel.Y)
		vz := fixedToFloat32(v.Body.Vel.Z)

		fx := fixedToFloat32(v.Body.Forward.X)
		fz := fixedToFloat32(v.Body.Forward.Z)

		rl.DrawText(fmt.Sprintf("state: %s", state), 20, 20, 22, rl.Black)
		rl.DrawText(fmt.Sprintf("tick: %d", world.Tick), 20, 48, 22, rl.Black)
		rl.DrawText(fmt.Sprintf("vehicles: %d", len(world.Vehicles)), 20, 76, 22, rl.Black)
		rl.DrawText(fmt.Sprintf("selected vehicle: %d", selected), 20, 104, 22, rl.Black)

		rl.DrawText(fmt.Sprintf("pos: (%.3f, %.3f, %.3f)", px, py, pz), 20, 140, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("vel: (%.3f, %.3f, %.3f)", vx, vy, vz), 20, 166, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("forward: (%.3f, %.3f)", fx, fz), 20, 192, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("onGround: %v", v.Body.OnGround), 20, 218, 20, rl.Black)
		rl.DrawText(fmt.Sprintf("hash: %d", sim.HashVehicleWorld(world)), 20, 244, 18, rl.DarkGray)

		baseY := 272
		for wi := range v.Wheels {
			wheel := v.Wheels[wi]
			rl.DrawText(
				fmt.Sprintf(
					"W%d contact=%v comp=%.3f load=%.3f",
					wi,
					wheel.Contact,
					fixedToFloat32(wheel.Compression),
					fixedToFloat32(wheel.Load),
				),
				20,
				int32(baseY+wi*22),
				18,
				rl.Black,
			)
		}

		rl.DrawRectangle(10, 335, 700, 170, rl.Fade(rl.LightGray, 0.85))
		rl.DrawText("Vehicle Controls", 20, 345, 22, rl.Maroon)
		rl.DrawText("Arrow Keys = drive selected vehicle", 20, 375, 18, rl.Black)
		rl.DrawText("Space = pause | N = single tick | R = reset", 20, 398, 18, rl.Black)
		rl.DrawText("1 = flat ground | 2 = slope ground | Tab = next vehicle", 20, 421, 18, rl.Black)
		rl.DrawText("W/S/A/D = move camera | Q/E = down/up", 20, 444, 18, rl.Black)
		rl.DrawText("J/L = yaw camera | I/K = pitch camera | Shift = faster", 20, 467, 18, rl.Black)

		rl.EndDrawing()
	}
}
