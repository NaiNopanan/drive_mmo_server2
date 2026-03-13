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

func fixedToF(v fixed.Fixed) float32 {
	return float32(float64(v.Raw()) / (1 << 32))
}

func vecToRL(v geom.Vec3) rl.Vector3 {
	return rl.NewVector3(fixedToF(v.X), fixedToF(v.Y), fixedToF(v.Z))
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
	c := v.Position
	hx := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	hy := fixed.FromFraction(25, 100)
	hz := v.Tuning.WheelBase.Div(fixed.FromInt(2))

	forward := v.ForwardWS
	right := v.RightWS
	up := v.UpWS

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
	bodyPos := vecToRL(v.Position)
	fwdEnd := vecToRL(v.Position.Add(v.ForwardWS.Scale(fixed.FromInt(3))))
	rl.DrawLine3D(bodyPos, fwdEnd, rl.Green)

	for wi := range v.Wheels {
		w := v.Wheels[wi]
		def := v.WheelDefs[wi]
		
		anchorWS := v.Position.Add(v.LocalToWorld(def.LocalAnchor))
		
		// wheel center calculation
		rayOrigin := anchorWS.Add(geom.V3(fixed.Zero, v.Tuning.SuspensionMaxRaise, fixed.Zero))
		center := rayOrigin.Add(geom.V3(fixed.Zero, w.ContactDistance.Neg(), fixed.Zero))

		rl.DrawLine3D(vecToRL(anchorWS), vecToRL(center), rl.Gray)
		rl.DrawSphereWires(vecToRL(center), fixedToF(v.Tuning.WheelRadius), 12, 12, wheelColor)

		if w.InContact {
			rl.DrawSphere(vecToRL(w.ContactPoint), 0.08, contactColor)
			
			// Suspension force arrow (Cyan)
			rl.DrawLine3D(vecToRL(w.ContactPoint), vecToRL(w.ContactPoint.Add(w.ContactNormal.Scale(w.SuspensionForce.Div(fixed.FromInt(10000))))), rl.SkyBlue)
			
			// Drive/Brake/Total lateral arrow? Let's just do fwd/right vectors at wheel
			rl.DrawLine3D(vecToRL(center), vecToRL(center.Add(w.WheelForwardWS.Scale(fixed.FromInt(1)))), rl.Blue)
			rl.DrawLine3D(vecToRL(center), vecToRL(center.Add(w.WheelRightWS.Scale(fixed.FromInt(1)))), rl.Yellow)
		}
	}
}

func spawnGrid(rows, cols int) []sim.Vehicle {
	var vs []sim.Vehicle
	id := uint32(1)
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			vs = append(vs, sim.NewVehicle(id, geom.V3(
				fixed.FromInt(int64(c*4)),
				fixed.FromInt(15),
				fixed.FromInt(int64(r*6)),
			)))
			id++
		}
	}
	return vs
}

func main() {
	const screenWidth = 1400
	const screenHeight = 900

	rl.InitWindow(screenWidth, screenHeight, "server2 Day 6 Advanced Vehicle Debug Viewer")
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
	_ = useFlatGround
	world := sim.VehicleWorldDay6{
		Ground: sim.FlatGround{
			Y:    fixed.Zero,
			MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
			MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
		},
		Vehicles: spawnGrid(3, 4),
	}

	selected := 0
	paused := false
	stepOnce := false

	for !rl.WindowShouldClose() {
		dtFixed := fixed.FromFraction(1, 60)

		if rl.IsKeyPressed(rl.KeySpace) {
			paused = !paused
		}
		if rl.IsKeyPressed(rl.KeyN) {
			stepOnce = true
		}
		if rl.IsKeyPressed(rl.KeyR) {
			world.Vehicles = spawnGrid(3, 4)
			world.Tick = 0
		}
		if rl.IsKeyPressed(rl.KeyOne) {
			useFlatGround = true
			world.Ground = sim.FlatGround{
				Y:    fixed.Zero,
				MinX: fixed.FromInt(-50), MaxX: fixed.FromInt(50),
				MinZ: fixed.FromInt(-50), MaxZ: fixed.FromInt(50),
			}
			world.Vehicles = spawnGrid(3, 4)
		}
		if rl.IsKeyPressed(rl.KeyTwo) {
			useFlatGround = false
			world.Ground = sim.SlopeGround{
				BaseY: fixed.Zero,
				Slope: fixed.FromFraction(2, 10),
				MinX:  fixed.FromInt(-40), MaxX: fixed.FromInt(40),
				MinZ:  fixed.FromInt(-40), MaxZ: fixed.FromInt(40),
			}
			world.Vehicles = spawnGrid(3, 4)
		}
		if rl.IsKeyPressed(rl.KeyThree) {
			useFlatGround = false
			world.Ground = sim.WorldGroundQuery{
				Triangles: sim.GroundSlopeSmall(),
			}
			world.Vehicles = spawnGrid(3, 4)
		}
		if rl.IsKeyPressed(rl.KeyTab) {
			selected++
			if selected >= len(world.Vehicles) {
				selected = 0
			}
		}

		cameraRig.Update(1.0 / 60.0)

		// simple autopilot for non-selected
		for i := range world.Vehicles {
			if i == selected {
				world.Vehicles[i].Input = sim.VehicleInput{}
				continue
			}
			// basic hold throttle for fun
			world.Vehicles[i].Input.Throttle = fixed.FromFraction(1, 2)
		}

		if selected < len(world.Vehicles) {
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
			world.Step(dtFixed)
			stepOnce = false
		}

		camera := cameraRig.ToCamera3D()

		rl.BeginDrawing()
		rl.ClearBackground(rl.RayWhite)

		rl.BeginMode3D(camera)
		// Draw ground based on type
		switch g := world.Ground.(type) {
		case sim.FlatGround:
			// DrawGrid(100, 1.0) = 100 slices * 1.0 spacing = ±50 units, matching physics boundary
			rl.DrawGrid(100, 1.0)
			// Draw boundary rectangle in red so it's clearly visible
			if g.MaxX.Cmp(fixed.Zero) != 0 {
				minX := fixedToF(g.MinX)
				maxX := fixedToF(g.MaxX)
				minZ := fixedToF(g.MinZ)
				maxZ := fixedToF(g.MaxZ)
				y := fixedToF(g.Y) + 0.02 // slightly above ground
				rl.DrawLine3D(rl.NewVector3(minX, y, minZ), rl.NewVector3(maxX, y, minZ), rl.Red)
				rl.DrawLine3D(rl.NewVector3(maxX, y, minZ), rl.NewVector3(maxX, y, maxZ), rl.Red)
				rl.DrawLine3D(rl.NewVector3(maxX, y, maxZ), rl.NewVector3(minX, y, maxZ), rl.Red)
				rl.DrawLine3D(rl.NewVector3(minX, y, maxZ), rl.NewVector3(minX, y, minZ), rl.Red)
			}
		case sim.SlopeGround:
			// Draw a grid follow the slope
			minX := fixedToF(g.MinX)
			maxX := fixedToF(g.MaxX)
			minZ := fixedToF(g.MinZ)
			maxZ := fixedToF(g.MaxZ)
			color := rl.Fade(rl.Purple, 0.4)
			
			step := float32(2.0)
			for x := minX; x <= maxX; x += step {
				y0 := fixedToF(g.BaseY.Add(g.Slope.Mul(fixed.FromInt(int64(minZ)))))
				y1 := fixedToF(g.BaseY.Add(g.Slope.Mul(fixed.FromInt(int64(maxZ)))))
				rl.DrawLine3D(rl.NewVector3(x, y0, minZ), rl.NewVector3(x, y1, maxZ), color)
			}
			for z := minZ; z <= maxZ; z += step {
				y := fixedToF(g.BaseY.Add(g.Slope.Mul(fixed.FromInt(int64(z)))))
				rl.DrawLine3D(rl.NewVector3(minX, y, z), rl.NewVector3(maxX, y, z), color)
			}
			// Border
			yMin := fixedToF(g.BaseY.Add(g.Slope.Mul(fixed.FromInt(int64(minZ)))))
			yMax := fixedToF(g.BaseY.Add(g.Slope.Mul(fixed.FromInt(int64(maxZ)))))
			rl.DrawLine3D(rl.NewVector3(minX, yMin, minZ), rl.NewVector3(maxX, yMin, minZ), rl.Red)
			rl.DrawLine3D(rl.NewVector3(maxX, yMin, minZ), rl.NewVector3(maxX, yMax, maxZ), rl.Red)
			rl.DrawLine3D(rl.NewVector3(maxX, yMax, maxZ), rl.NewVector3(minX, yMax, maxZ), rl.Red)
			rl.DrawLine3D(rl.NewVector3(minX, yMax, maxZ), rl.NewVector3(minX, yMin, minZ), rl.Red)

		case sim.WorldGroundQuery:
			for _, tri := range g.Triangles {
				a := vecToRL(tri.A)
				b := vecToRL(tri.B)
				c := vecToRL(tri.C)
				rl.DrawLine3D(a, b, rl.DarkPurple)
				rl.DrawLine3D(b, c, rl.DarkPurple)
				rl.DrawLine3D(c, a, rl.DarkPurple)
				
				// Draw normal vector for each triangle to see surface better
				center := rl.NewVector3((a.X+b.X+c.X)/3, (a.Y+b.Y+c.Y)/3, (a.Z+b.Z+c.Z)/3)
				n := vecToRL(tri.Normal())
				rl.DrawLine3D(center, rl.NewVector3(center.X+n.X, center.Y+n.Y, center.Z+n.Z), rl.Yellow)
			}
		}

		for i := range world.Vehicles {
			drawVehicle(world.Vehicles[i], i == selected)
		}

		rl.EndMode3D()

		// status overlay
		rl.DrawRectangle(10, 10, 520, 360, rl.Fade(rl.SkyBlue, 0.18))
		v := world.Vehicles[selected]
		
		rl.DrawText(fmt.Sprintf("Tick: %d | Vehicle: %d", world.Tick, selected), 20, 20, 22, rl.Black)
		rl.DrawText(fmt.Sprintf("Pos: (%.2f, %.2f, %.2f)", fixedToF(v.Position.X), fixedToF(v.Position.Y), fixedToF(v.Position.Z)), 20, 48, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Vel: (%.2f, %.2f, %.2f) Speed: %.2f", fixedToF(v.Velocity.X), fixedToF(v.Velocity.Y), fixedToF(v.Velocity.Z), fixedToF(v.Velocity.Length())), 20, 72, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Yaw: %.2f YawVel: %.2f", fixedToF(v.Yaw), fixedToF(v.YawVelocity)), 20, 96, 18, rl.Black)
		rl.DrawText(fmt.Sprintf("Grounded: %d OnGround: %v", v.GroundedWheels, v.OnGround), 20, 120, 18, rl.Black)
		
		for i := range v.Wheels {
			w := v.Wheels[i]
			rl.DrawText(fmt.Sprintf("W%d: Contact=%v Comp=%.3f SuspF=%.0f LatF=%.0f Steer=%.1f", 
				i, w.InContact, fixedToF(w.Compression), fixedToF(w.SuspensionForce), fixedToF(w.LateralForce), 180*fixedToF(w.SteerAngleRad)/math.Pi), 
				20, int32(150+i*22), 16, rl.DarkGray)
		}

		rl.DrawText("Controls: arrows=drive, tab=cycle, space=pause, 1/2/3=ground", 20, 300, 18, rl.DarkBlue)
		rl.DrawText("Flycam: WASD/QE + JLI/K", 20, 325, 18, rl.DarkBlue)

		rl.EndDrawing()
	}
}
