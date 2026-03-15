package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type VehicleObstacleContact struct {
	Hit    bool
	Normal geom.Vec3
	Depth  fixed.Fixed
}

func CollideVehicleWithObstacles(v *Vehicle, obstacles []CityObstacle) {
	if v == nil || len(obstacles) == 0 {
		return
	}

	depenetrationSlop := fixed.FromFraction(1, 1000)

	for iter := 0; iter < 4; iter++ {
		resolved := false

		for i := range obstacles {
			contact := vehicleObstacleContact(*v, obstacles[i])
			if !contact.Hit {
				continue
			}

			v.Position = v.Position.Add(contact.Normal.Scale(contact.Depth.Add(depenetrationSlop)))

			planarVelocity := geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z)
			intoObstacle := planarVelocity.Dot(contact.Normal)
			if intoObstacle.Cmp(fixed.Zero) < 0 {
				cancelIntoObstacle := contact.Normal.Scale(intoObstacle.Neg())
				v.Velocity.X = v.Velocity.X.Add(cancelIntoObstacle.X)
				v.Velocity.Z = v.Velocity.Z.Add(cancelIntoObstacle.Z)
			}

			resolved = true
		}

		if !resolved {
			return
		}
	}
}

func vehicleObstacleContact(v Vehicle, obstacle CityObstacle) VehicleObstacleContact {
	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	halfHeight := fixed.FromFraction(1, 4)

	vehicleMinY := v.Position.Y.Sub(halfHeight)
	vehicleMaxY := v.Position.Y.Add(halfHeight)
	if vehicleMinY.Cmp(obstacle.TopY()) >= 0 || vehicleMaxY.Cmp(obstacle.BaseY) <= 0 {
		return VehicleObstacleContact{}
	}

	obstacleCenter := geom.V3(
		obstacle.MinX.Add(obstacle.MaxX).Div(fixed.FromInt(2)),
		fixed.Zero,
		obstacle.MinZ.Add(obstacle.MaxZ).Div(fixed.FromInt(2)),
	)
	vehicleCenter := geom.V3(v.Position.X, fixed.Zero, v.Position.Z)
	delta := vehicleCenter.Sub(obstacleCenter)

	forward := geom.V3(v.ForwardWS.X, fixed.Zero, v.ForwardWS.Z).Normalize()
	right := geom.V3(v.RightWS.X, fixed.Zero, v.RightWS.Z).Normalize()
	if forward.LengthSq().Cmp(fixed.Zero) == 0 || right.LengthSq().Cmp(fixed.Zero) == 0 {
		forward, right = HeadingFromYaw(v.Yaw)
		forward = geom.V3(forward.X, fixed.Zero, forward.Z).Normalize()
		right = geom.V3(right.X, fixed.Zero, right.Z).Normalize()
	}
	obstacleForward, obstacleRight, obstacleHalfForward, obstacleHalfRight := obstacle.OrientedAxes()

	axes := [4]geom.Vec3{
		obstacleRight,
		obstacleForward,
		right,
		forward,
	}

	best := VehicleObstacleContact{}
	for i := range axes {
		axis := axes[i]
		if axis.LengthSq().Cmp(fixed.Zero) == 0 {
			continue
		}

		centerDelta := delta.Dot(axis)
		vehicleRadius := halfWidth.Mul(right.Dot(axis).Abs()).
			Add(halfLen.Mul(forward.Dot(axis).Abs()))
		obstacleRadius := obstacleHalfRight.Mul(obstacleRight.Dot(axis).Abs()).
			Add(obstacleHalfForward.Mul(obstacleForward.Dot(axis).Abs()))
		overlap := vehicleRadius.Add(obstacleRadius).Sub(centerDelta.Abs())
		if overlap.Cmp(fixed.Zero) <= 0 {
			return VehicleObstacleContact{}
		}

		normal := axis
		if centerDelta.Cmp(fixed.Zero) < 0 {
			normal = axis.Neg()
		} else if centerDelta.Cmp(fixed.Zero) == 0 {
			normal = axis.Neg()
		}

		if !best.Hit || overlap.Cmp(best.Depth) < 0 {
			best = VehicleObstacleContact{
				Hit:    true,
				Normal: normal,
				Depth:  overlap,
			}
		}
	}

	return best
}
