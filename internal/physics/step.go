package physics

import (
	"math"

	"server2/internal/worldmesh"
	"server2/pkg/geom"
)

const spawnDropGravity float32 = 18

// Step จะขยับโลกไปข้างหน้า 1 tick ตาม fixed dt ของ world
func (w *PhysicsWorld) Step(input DriveInput) {
	previous := w.player
	w.player.BodyHitMap = false
	w.player = simulateVehicle(w.player, input, w.config.FixedDT)
	w.player = w.applyWorldConstraints(w.player)
	w.player = w.applyGrounding(w.player, w.config.FixedDT)
	if w.bodyOBBIntersectsMap(w.player) {
		w.player = previous
		w.player.BodyHitMap = true
	}
	w.tick++
}

// simulateVehicle เป็นฟิสิกส์แบบ arcade ง่าย ๆ สำหรับใช้เป็นฐานเริ่มระบบรถใหม่
func simulateVehicle(vehicle VehicleBody, input DriveInput, dt float32) VehicleBody {
	params := vehicle.Params

	forward := geom.FromHeading(vehicle.Heading)
	right := geom.Planar(-forward.Z, forward.X)

	forwardSpeed := vehicle.Velocity.Dot(forward)
	lateralSpeed := vehicle.Velocity.Dot(right)

	accel := input.Throttle*params.AccelRate - input.Brake*params.BrakeRate
	if input.Nitro {
		accel += 60
	}

	dragForce := forwardSpeed * params.Drag
	rolling := params.RollingResistance
	if forwardSpeed < 0 {
		rolling = -rolling
	}

	forwardSpeed += (accel - dragForce - rolling) * dt
	forwardSpeed = clamp(forwardSpeed, -params.MaxReverseSpeed, params.MaxForwardSpeed)

	speedRatio := float32(math.Abs(float64(forwardSpeed / max(params.MaxForwardSpeed, 1))))
	steerDamp := 1 - speedRatio*params.HighSpeedSteerDamp
	if steerDamp < 0.2 {
		steerDamp = 0.2
	}

	// ใช้ hysteresis กันการสลับโหมดเลี้ยวไปมาช่วงความเร็วใกล้ศูนย์
	if vehicle.ReverseSteer {
		if forwardSpeed > 0.8 {
			vehicle.ReverseSteer = false
		}
	} else if forwardSpeed < -1.2 {
		vehicle.ReverseSteer = true
	}

	steerDirection := float32(1)
	if vehicle.ReverseSteer {
		steerDirection = -1
	}

	targetYawRate := input.Steering * steerDirection * params.MaxSteerRate * steerDamp
	vehicle.YawRate += (targetYawRate - vehicle.YawRate) * min(1, params.SteerResponse*dt)
	vehicle.Heading = normalizeAngle(vehicle.Heading + vehicle.YawRate*dt)

	grip := params.Grip
	if input.Handbrake {
		grip = params.DriftGrip
	}

	lateralGrip := float32(math.Exp(float64(-grip * dt)))
	lateralSpeed *= lateralGrip

	forward = geom.FromHeading(vehicle.Heading)
	right = geom.Planar(-forward.Z, forward.X)
	vehicle.Velocity = forward.MulScalar(forwardSpeed).Add(right.MulScalar(lateralSpeed))
	vehicle.Position = vehicle.Position.Add(vehicle.Velocity.MulScalar(dt))
	vehicle.Speed = vehicle.Velocity.Length()

	return vehicle
}

// applyGrounding ใช้ raycast ของล้อทั้ง 4 จุด เพื่อให้ตัวถังลอยจาก wheel radius + rest length จริง
func (w *PhysicsWorld) applyGrounding(vehicle VehicleBody, dt float32) VehicleBody {
	vehicle.Wheels = w.sampleWheelStates(vehicle)

	support := solveSupport(vehicle)
	vehicle.SupportState = support.State
	vehicle.SupportHits = support.HitCount
	vehicle.GroundHeight = support.GroundHeight

	if support.State != SupportStateFalling {
		if vehicle.Height <= support.BodyHeight && vehicle.VerticalVel <= 0 {
			vehicle.Height = support.BodyHeight
			vehicle.VerticalVel = 0
		} else {
			vehicle.VerticalVel -= spawnDropGravity * dt
			vehicle.Height += vehicle.VerticalVel * dt
			if vehicle.Height <= support.BodyHeight {
				vehicle.Height = support.BodyHeight
				vehicle.VerticalVel = 0
			}
		}
	} else {
		vehicle.VerticalVel -= spawnDropGravity * dt
		vehicle.Height += vehicle.VerticalVel * dt
	}

	vehicle = applyBodyTilt(vehicle, support)
	return vehicle
}

func (w *PhysicsWorld) sampleWheelStates(vehicle VehicleBody) [wheelCount]WheelState {
	suspension := vehicle.Params.Suspension
	localOffsets := wheelLocalOffsets(suspension)
	wheelLabels := [wheelCount]string{"FL", "FR", "RL", "RR"}
	maxDistance := suspension.WheelRadius + suspension.RestLength + suspension.MaxTravel

	var wheels [wheelCount]WheelState
	for index, localOffset := range localOffsets {
		mountPoint := supportMountPoint(vehicle, localOffset.X, suspension.MountHeight, localOffset.Z)
		hit := w.queryGroundHit(mountPoint, maxDistance)

		wheel := WheelState{
			Label:       wheelLabels[index],
			MountPoint:  mountPoint,
			Hit:         hit.Hit,
			HitPoint:    hit.Point,
			HitDistance: hit.Distance,
			WheelCenter: supportMountPoint(vehicle, localOffset.X, suspension.MountHeight-(suspension.RestLength+suspension.MaxTravel), localOffset.Z),
		}

		if hit.Hit {
			suspensionLength := hit.Distance - suspension.WheelRadius
			if suspensionLength < 0 {
				suspensionLength = 0
			}

			wheel.SuspensionLength = suspensionLength
			wheel.Compression = suspension.RestLength - suspensionLength
			wheel.WheelCenter = geom.V3(hit.Point.X, hit.Point.Y+suspension.WheelRadius, hit.Point.Z)
		}

		wheels[index] = wheel
	}

	return wheels
}

func wheelLocalOffsets(suspension SuspensionParams) [wheelCount]geom.PlanarVec {
	return [wheelCount]geom.PlanarVec{
		geom.Planar(suspension.FrontAxleOffset, -suspension.HalfTrackWidth),
		geom.Planar(suspension.FrontAxleOffset, suspension.HalfTrackWidth),
		geom.Planar(-suspension.RearAxleOffset, -suspension.HalfTrackWidth),
		geom.Planar(-suspension.RearAxleOffset, suspension.HalfTrackWidth),
	}
}

func supportMountPoint(vehicle VehicleBody, localX, localY, localZ float32) geom.Vec3 {
	base := geom.V3(vehicle.Position.X, vehicle.Height, vehicle.Position.Z)
	forward, right, up := vehicleAxes(vehicle.Heading, vehicle.Pitch, vehicle.Roll)
	point := base
	point = point.Add(forward.MulScalar(localX))
	point = point.Add(up.MulScalar(localY))
	point = point.Add(right.MulScalar(localZ))
	return point
}

func wheelSupportBodyHeight(suspension SuspensionParams, groundHeight float32) float32 {
	return groundHeight + suspension.WheelRadius + suspension.RestLength - suspension.MountHeight
}

type supportSolution struct {
	State        SupportState
	HitCount     int
	GroundHeight float32
	BodyHeight   float32
	TargetPitch  float32
	TargetRoll   float32
}

func solveSupport(vehicle VehicleBody) supportSolution {
	suspension := vehicle.Params.Suspension
	wheelBase := suspension.FrontAxleOffset + suspension.RearAxleOffset
	trackWidth := suspension.HalfTrackWidth * 2
	hitPoints := make([]geom.Vec3, 0, wheelCount)
	totalGroundHeight := float32(0)
	totalBodyHeight := float32(0)
	for _, wheel := range vehicle.Wheels {
		if !wheel.Hit {
			continue
		}
		hitPoints = append(hitPoints, wheel.HitPoint)
		totalGroundHeight += wheel.HitPoint.Y
		totalBodyHeight += wheelSupportBodyHeight(suspension, wheel.HitPoint.Y)
	}

	solution := supportSolution{
		State:        SupportStateFalling,
		HitCount:     len(hitPoints),
		GroundHeight: 0,
		BodyHeight:   vehicle.Height,
	}
	if len(hitPoints) > 0 {
		invCount := 1 / float32(len(hitPoints))
		solution.GroundHeight = totalGroundHeight * invCount
		solution.BodyHeight = totalBodyHeight * invCount
	}

	frontAverage, frontCount := averageWheelSupportHeight(vehicle.Wheels, vehicle.Params.Suspension, 0, 1)
	rearAverage, rearCount := averageWheelSupportHeight(vehicle.Wheels, vehicle.Params.Suspension, 2, 3)
	leftAverage, leftCount := averageWheelSupportHeight(vehicle.Wheels, vehicle.Params.Suspension, 0, 2)
	rightAverage, rightCount := averageWheelSupportHeight(vehicle.Wheels, vehicle.Params.Suspension, 1, 3)

	if len(hitPoints) >= 3 {
		normal := supportPlaneNormal(hitPoints[0], hitPoints[1], hitPoints[2])
		if normal.Y > 0 {
			solution.State = SupportStateStable
			solution.TargetPitch, solution.TargetRoll = tiltFromNormal(vehicle.Heading, normal)
			return solution
		}
	}

	if len(hitPoints) >= 2 {
		solution.State = SupportStateEdge
		if frontCount > 0 && rearCount > 0 && wheelBase > 0 {
			solution.TargetPitch = float32(math.Atan2(float64(frontAverage-rearAverage), float64(wheelBase)))
		}
		if leftCount > 0 && rightCount > 0 && trackWidth > 0 {
			solution.TargetRoll = float32(math.Atan2(float64(leftAverage-rightAverage), float64(trackWidth)))
		}
		return solution
	}

	return solution
}

func applyBodyTilt(vehicle VehicleBody, support supportSolution) VehicleBody {
	targetPitch := float32(0)
	targetRoll := float32(0)
	smooth := float32(0.18)

	if support.State == SupportStateFalling {
		// ตอนรถลอยให้งุ้มหัวลงเล็กน้อยพอให้ดูมีน้ำหนัก แต่ไม่ทิ่มแรงเกินไป
		noseDown := clamp(0.05+max(-vehicle.VerticalVel, 0)*0.025+vehicle.Speed*0.001, 0.05, 0.22)
		targetPitch = -noseDown
		smooth = 0.14
	} else {
		targetPitch = clamp(support.TargetPitch, -0.6, 0.6)
		targetRoll = clamp(support.TargetRoll, -0.6, 0.6)
		smooth = 0.35
	}

	vehicle.Pitch += (targetPitch - vehicle.Pitch) * smooth
	vehicle.Roll += (targetRoll - vehicle.Roll) * smooth
	return vehicle
}

func averageWheelSupportHeight(wheels [wheelCount]WheelState, suspension SuspensionParams, indices ...int) (float32, int) {
	total := float32(0)
	count := 0
	for _, index := range indices {
		wheel := wheels[index]
		if !wheel.Hit {
			continue
		}
		total += wheelSupportBodyHeight(suspension, wheel.HitPoint.Y)
		count++
	}

	if count == 0 {
		return 0, 0
	}
	return total / float32(count), count
}

func supportPlaneNormal(a, b, c geom.Vec3) geom.Vec3 {
	ab := b.Sub(a)
	ac := c.Sub(a)
	normal := ab.Cross(ac).Normalize()
	if normal.Y < 0 {
		normal = normal.MulScalar(-1)
	}
	return normal
}

func vehicleAxes(heading, pitch, roll float32) (geom.Vec3, geom.Vec3, geom.Vec3) {
	forward := geom.V3(
		float32(math.Cos(float64(heading))),
		0,
		float32(math.Sin(float64(heading))),
	)
	right := geom.V3(-forward.Z, 0, forward.X)
	up := geom.V3(0, 1, 0)

	forward = rotateAroundAxis(forward, right, pitch)
	up = rotateAroundAxis(up, right, pitch)
	right = rotateAroundAxis(right, forward, roll)
	up = rotateAroundAxis(up, forward, roll)

	return forward.Normalize(), right.Normalize(), up.Normalize()
}

func rotateAroundAxis(vector, axis geom.Vec3, angle float32) geom.Vec3 {
	axis = axis.Normalize()
	cosAngle := float32(math.Cos(float64(angle)))
	sinAngle := float32(math.Sin(float64(angle)))

	term1 := vector.MulScalar(cosAngle)
	term2 := axis.Cross(vector).MulScalar(sinAngle)
	term3 := axis.MulScalar(axis.Dot(vector) * (1 - cosAngle))
	return term1.Add(term2).Add(term3)
}

func tiltFromNormal(heading float32, normal geom.Vec3) (float32, float32) {
	forward := geom.FromHeading(heading)
	right := geom.Planar(-forward.Z, forward.X)
	forwardDot := forward.X*normal.X + forward.Z*normal.Z
	rightDot := right.X*normal.X + right.Z*normal.Z
	pitch := float32(-math.Atan2(float64(forwardDot), float64(normal.Y)))
	roll := float32(math.Atan2(float64(rightDot), float64(normal.Y)))
	return pitch, roll
}

func (w *PhysicsWorld) queryGroundHit(origin geom.Vec3, maxDistance float32) worldmesh.RaycastHit {
	if len(w.staticMesh.Triangles) == 0 {
		return worldmesh.RaycastHit{}
	}

	return w.staticMesh.RaycastDown(origin, maxDistance)
}

func (w *PhysicsWorld) applyWorldConstraints(vehicle VehicleBody) VehicleBody {
	bounds := w.config.WorldBounds
	clamped := bounds.ClampPoint(vehicle.Position)
	hitBounds := clamped != vehicle.Position
	vehicle.Position = clamped

	if hitBounds {
		// ถ้ารถชนขอบโลกให้หยุดแรงและค้างอยู่ในพื้นที่ sandbox
		vehicle.Velocity = geom.PlanarVec{}
		vehicle.Speed = 0
		vehicle.YawRate = 0
	}

	vehicle.LastSafePos = vehicle.Position
	vehicle.Speed = vehicle.Velocity.Length()
	return vehicle
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

func max(a, b float32) float32 {
	if a > b {
		return a
	}
	return b
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
