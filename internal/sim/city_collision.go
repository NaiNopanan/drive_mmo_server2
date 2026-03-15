package sim

import "server2/internal/fixed"

func CollideVehicleWithObstacles(v *Vehicle, obstacles []CityObstacle) {
	if v == nil || len(obstacles) == 0 {
		return
	}

	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	halfHeight := fixed.FromFraction(1, 4)

	for iter := 0; iter < 4; iter++ {
		resolved := false

		for i := range obstacles {
			o := obstacles[i]

			vehicleMinY := v.Position.Y.Sub(halfHeight)
			vehicleMaxY := v.Position.Y.Add(halfHeight)
			if vehicleMinY.Cmp(o.TopY()) >= 0 || vehicleMaxY.Cmp(o.BaseY) <= 0 {
				continue
			}

			vehicleMinX := v.Position.X.Sub(halfWidth)
			vehicleMaxX := v.Position.X.Add(halfWidth)
			vehicleMinZ := v.Position.Z.Sub(halfLen)
			vehicleMaxZ := v.Position.Z.Add(halfLen)

			if vehicleMaxX.Cmp(o.MinX) <= 0 || vehicleMinX.Cmp(o.MaxX) >= 0 ||
				vehicleMaxZ.Cmp(o.MinZ) <= 0 || vehicleMinZ.Cmp(o.MaxZ) >= 0 {
				continue
			}

			moveLeft := vehicleMaxX.Sub(o.MinX)
			moveRight := o.MaxX.Sub(vehicleMinX)
			moveBack := vehicleMaxZ.Sub(o.MinZ)
			moveForward := o.MaxZ.Sub(vehicleMinZ)

			axis := 0
			best := moveLeft

			if moveRight.Cmp(best) < 0 {
				axis = 1
				best = moveRight
			}
			if moveBack.Cmp(best) < 0 {
				axis = 2
				best = moveBack
			}
			if moveForward.Cmp(best) < 0 {
				axis = 3
			}

			switch axis {
			case 0:
				v.Position.X = o.MinX.Sub(halfWidth)
				if v.Velocity.X.Cmp(fixed.Zero) > 0 {
					v.Velocity.X = fixed.Zero
				}
			case 1:
				v.Position.X = o.MaxX.Add(halfWidth)
				if v.Velocity.X.Cmp(fixed.Zero) < 0 {
					v.Velocity.X = fixed.Zero
				}
			case 2:
				v.Position.Z = o.MinZ.Sub(halfLen)
				if v.Velocity.Z.Cmp(fixed.Zero) > 0 {
					v.Velocity.Z = fixed.Zero
				}
			case 3:
				v.Position.Z = o.MaxZ.Add(halfLen)
				if v.Velocity.Z.Cmp(fixed.Zero) < 0 {
					v.Velocity.Z = fixed.Zero
				}
			}

			resolved = true
		}

		if !resolved {
			return
		}
	}
}
