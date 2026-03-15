package sim

import (
	"testing"

	"server2/internal/fixed"
	"server2/internal/geom"
)

type driveSample struct {
	Pos            geom.Vec3
	SpeedXZ        fixed.Fixed
	ForwardSpeed   fixed.Fixed
	SideSpeed      fixed.Fixed
	Yaw            fixed.Fixed
	YawVel         fixed.Fixed
	OnGround       bool
	GroundedWheels int
	VelY           fixed.Fixed
}

type driveProfile struct {
	Name string

	ForwardMinProgressZ fixed.Fixed
	ForwardMaxYawAbs    fixed.Fixed
	ForwardGroundedMin  fixed.Fixed

	SteerMinYawAbs      fixed.Fixed
	SteerMinArcXAbs     fixed.Fixed
	SteerGroundedMin    fixed.Fixed
	SteerReleaseKeepMin fixed.Fixed // fraction of yaw rate still remaining after release window

	ReverseMinForwardSpeed fixed.Fixed // negative
	ReverseMinProgressZ    fixed.Fixed // negative
	ReverseMaxYawAbs       fixed.Fixed
	ReverseGroundedMin     fixed.Fixed

	UphillMinProgressZ fixed.Fixed
	UphillMinGainY     fixed.Fixed
	UphillGroundedMin  fixed.Fixed
	UphillMaxAbsVelY   fixed.Fixed

	DownhillMinProgressZ fixed.Fixed // negative
	DownhillGroundedMin  fixed.Fixed
	DownhillMaxAbsVelY   fixed.Fixed

	SlopeSteerMinProgressZ fixed.Fixed
	SlopeSteerMinArcXAbs   fixed.Fixed
	SlopeSteerMinGainY     fixed.Fixed
	SlopeSteerGroundedMin  fixed.Fixed
	SlopeSteerMinMaxYawVel fixed.Fixed
}

func driveabilityFlatGround() FlatGround {
	return FlatGround{
		Y:    fixed.Zero,
		MinX: fixed.FromInt(-1000),
		MaxX: fixed.FromInt(1000),
		MinZ: fixed.FromInt(-1000),
		MaxZ: fixed.FromInt(1000),
	}
}

func driveabilitySlopeGround() SlopeGround {
	return SlopeGround{
		BaseY: fixed.Zero,
		Slope: fixed.FromFraction(1, 10), // 0.1
		MinX:  fixed.FromInt(-1000),
		MaxX:  fixed.FromInt(1000),
		MinZ:  fixed.FromInt(-1000),
		MaxZ:  fixed.FromInt(1000),
	}
}

func settleDriveabilityVehicle(v *Vehicle, g GroundQuery, dt fixed.Fixed, ticks int) {
	v.Input = VehicleInput{}
	for i := 0; i < ticks; i++ {
		v.Step(dt, g)
	}
}

func captureDriveSample(v *Vehicle) driveSample {
	return driveSample{
		Pos:            v.Position,
		SpeedXZ:        geom.V3(v.Velocity.X, fixed.Zero, v.Velocity.Z).Length(),
		ForwardSpeed:   v.Velocity.Dot(v.ForwardWS),
		SideSpeed:      v.Velocity.Dot(v.RightWS),
		Yaw:            v.Yaw,
		YawVel:         v.YawVelocity,
		OnGround:       v.OnGround,
		GroundedWheels: v.GroundedWheels,
		VelY:           v.Velocity.Y,
	}
}

func traceDriveability(v *Vehicle, g GroundQuery, dt fixed.Fixed, ticks int, inputAt func(int) VehicleInput) []driveSample {
	out := make([]driveSample, 0, ticks)
	for i := 0; i < ticks; i++ {
		v.Input = inputAt(i)
		v.Step(dt, g)
		out = append(out, captureDriveSample(v))
	}
	return out
}

func groundedRatio(samples []driveSample) fixed.Fixed {
	grounded := int64(0)
	for _, s := range samples {
		if s.OnGround && s.GroundedWheels >= 2 {
			grounded++
		}
	}
	if len(samples) == 0 {
		return fixed.Zero
	}
	return fixed.FromFraction(grounded, int64(len(samples)))
}

func maxAbsYawVel(samples []driveSample) fixed.Fixed {
	maxv := fixed.Zero
	for _, s := range samples {
		v := s.YawVel.Abs()
		if v.Cmp(maxv) > 0 {
			maxv = v
		}
	}
	return maxv
}

func maxAbsVelY(samples []driveSample) fixed.Fixed {
	maxv := fixed.Zero
	for _, s := range samples {
		v := s.VelY.Abs()
		if v.Cmp(maxv) > 0 {
			maxv = v
		}
	}
	return maxv
}

func runDriveabilityForwardSmoothFlat(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilityFlatGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	settleDriveabilityVehicle(&v, g, dt, 120)
	startZ := v.Position.Z

	trace := traceDriveability(&v, g, dt, 180, func(_ int) VehicleInput {
		return VehicleInput{Throttle: fixed.One}
	})

	checkpoints := []int{29, 59, 89, 119, 149, 179}
	for i := 1; i < len(checkpoints); i++ {
		prev := trace[checkpoints[i-1]].SpeedXZ
		curr := trace[checkpoints[i]].SpeedXZ
		if curr.Cmp(prev) <= 0 {
			t.Fatalf("[%s] forward speed not ramping: t%d=%v t%d=%v",
				p.Name, checkpoints[i-1], prev, checkpoints[i], curr)
		}
	}

	deltaZ := trace[len(trace)-1].Pos.Z.Sub(startZ)
	if deltaZ.Cmp(p.ForwardMinProgressZ) <= 0 {
		t.Fatalf("[%s] forward progress too small: got=%v need>%v", p.Name, deltaZ, p.ForwardMinProgressZ)
	}

	gr := groundedRatio(trace)
	if gr.Cmp(p.ForwardGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low in forward run: got=%v need>=%v", p.Name, gr, p.ForwardGroundedMin)
	}

	yawAbs := trace[len(trace)-1].Yaw.Abs()
	if yawAbs.Cmp(p.ForwardMaxYawAbs) > 0 {
		t.Fatalf("[%s] straight-line yaw drift too large: got=%v max=%v", p.Name, yawAbs, p.ForwardMaxYawAbs)
	}
}

func runDriveabilitySteeringSmoothFlat(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilityFlatGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	settleDriveabilityVehicle(&v, g, dt, 120)

	_ = traceDriveability(&v, g, dt, 120, func(_ int) VehicleInput {
		return VehicleInput{Throttle: fixed.One}
	})

	steerTrace := traceDriveability(&v, g, dt, 120, func(_ int) VehicleInput {
		return VehicleInput{
			Throttle: fixed.One,
			Steer:    fixed.FromFraction(1, 2),
		}
	})

	finalYawAbs := steerTrace[len(steerTrace)-1].Yaw.Abs()
	if finalYawAbs.Cmp(p.SteerMinYawAbs) <= 0 {
		t.Fatalf("[%s] steering did not build enough yaw: got=%v need>%v", p.Name, finalYawAbs, p.SteerMinYawAbs)
	}

	finalXAbs := steerTrace[len(steerTrace)-1].Pos.X.Abs()
	if finalXAbs.Cmp(p.SteerMinArcXAbs) <= 0 {
		t.Fatalf("[%s] steering arc too small: got=%v need>%v", p.Name, finalXAbs, p.SteerMinArcXAbs)
	}

	gr := groundedRatio(steerTrace)
	if gr.Cmp(p.SteerGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low while steering: got=%v need>=%v", p.Name, gr, p.SteerGroundedMin)
	}

	releaseYaw := steerTrace[len(steerTrace)-1].YawVel.Abs()
	if releaseYaw.Cmp(fixed.FromFraction(5, 100)) <= 0 {
		t.Fatalf("[%s] steering phase did not build enough yaw rate to test release: got=%v", p.Name, releaseYaw)
	}

	releaseTrace := traceDriveability(&v, g, dt, 6, func(_ int) VehicleInput {
		return VehicleInput{Throttle: fixed.One}
	})

	after := releaseTrace[4].YawVel.Abs()
	minKeep := releaseYaw.Mul(p.SteerReleaseKeepMin)
	if after.Cmp(minKeep) < 0 {
		t.Fatalf("[%s] yaw dies too fast after release: got=%v need>=%v (from start=%v)",
			p.Name, after, minKeep, releaseYaw)
	}
}

func runDriveabilityReverseSmoothFlat(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilityFlatGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.Zero))

	settleDriveabilityVehicle(&v, g, dt, 120)
	startZ := v.Position.Z

	trace := traceDriveability(&v, g, dt, 180, func(_ int) VehicleInput {
		return VehicleInput{Throttle: fixed.FromFraction(3, 5).Neg()}
	})

	finalForward := trace[len(trace)-1].ForwardSpeed
	if finalForward.Cmp(p.ReverseMinForwardSpeed) > 0 {
		t.Fatalf("[%s] reverse speed too weak: got=%v need<=%v", p.Name, finalForward, p.ReverseMinForwardSpeed)
	}

	deltaZ := trace[len(trace)-1].Pos.Z.Sub(startZ)
	if deltaZ.Cmp(p.ReverseMinProgressZ) > 0 {
		t.Fatalf("[%s] reverse progress too small: got=%v need<=%v", p.Name, deltaZ, p.ReverseMinProgressZ)
	}

	gr := groundedRatio(trace)
	if gr.Cmp(p.ReverseGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low in reverse: got=%v need>=%v", p.Name, gr, p.ReverseGroundedMin)
	}

	yawAbs := trace[len(trace)-1].Yaw.Abs()
	if yawAbs.Cmp(p.ReverseMaxYawAbs) > 0 {
		t.Fatalf("[%s] reverse straight-line yaw drift too large: got=%v max=%v", p.Name, yawAbs, p.ReverseMaxYawAbs)
	}
}

func runDriveabilityUphillSmooth(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilitySlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-30)))

	settleDriveabilityVehicle(&v, g, dt, 180)
	startZ := v.Position.Z
	startY := v.Position.Y

	trace := traceDriveability(&v, g, dt, 300, func(i int) VehicleInput {
		return VehicleInput{Throttle: fixed.One}
	})

	checkpoints := []int{59, 119, 179, 239, 299}
	lastZ := startZ
	for _, idx := range checkpoints {
		if trace[idx].Pos.Z.Cmp(lastZ) <= 0 {
			t.Fatalf("[%s] vehicle stopped or rolled back uphill at tick=%d: prev=%v curr=%v",
				p.Name, idx, lastZ, trace[idx].Pos.Z)
		}
		lastZ = trace[idx].Pos.Z
	}

	deltaZ := trace[len(trace)-1].Pos.Z.Sub(startZ)
	if deltaZ.Cmp(p.UphillMinProgressZ) <= 0 {
		t.Fatalf("[%s] uphill progress too small: got=%v need>%v", p.Name, deltaZ, p.UphillMinProgressZ)
	}

	deltaY := trace[len(trace)-1].Pos.Y.Sub(startY)
	if deltaY.Cmp(p.UphillMinGainY) <= 0 {
		t.Fatalf("[%s] uphill altitude gain too small: got=%v need>%v", p.Name, deltaY, p.UphillMinGainY)
	}

	gr := groundedRatio(trace)
	if gr.Cmp(p.UphillGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low uphill: got=%v need>=%v", p.Name, gr, p.UphillGroundedMin)
	}

	maxVy := maxAbsVelY(trace[60:])
	if maxVy.Cmp(p.UphillMaxAbsVelY) > 0 {
		t.Fatalf("[%s] uphill bounce too large: got=%v max=%v", p.Name, maxVy, p.UphillMaxAbsVelY)
	}
}

func runDriveabilityDownhillSmooth(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilitySlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(6), fixed.FromInt(20)))

	settleDriveabilityVehicle(&v, g, dt, 180)
	startZ := v.Position.Z

	trace := traceDriveability(&v, g, dt, 240, func(_ int) VehicleInput {
		return VehicleInput{}
	})

	deltaZ := trace[len(trace)-1].Pos.Z.Sub(startZ)
	if deltaZ.Cmp(p.DownhillMinProgressZ) >= 0 {
		t.Fatalf("[%s] downhill progress too small: got=%v need<%v", p.Name, deltaZ, p.DownhillMinProgressZ)
	}

	gr := groundedRatio(trace)
	if gr.Cmp(p.DownhillGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low downhill: got=%v need>=%v", p.Name, gr, p.DownhillGroundedMin)
	}

	maxVy := maxAbsVelY(trace[30:])
	if maxVy.Cmp(p.DownhillMaxAbsVelY) > 0 {
		t.Fatalf("[%s] downhill vertical motion too large: got=%v max=%v", p.Name, maxVy, p.DownhillMaxAbsVelY)
	}
}

func runDriveabilitySteerOnSlopeSmooth(t *testing.T, p driveProfile) {
	t.Helper()

	g := driveabilitySlopeGround()
	dt := fixed.FromFraction(1, 60)
	v := NewVehicle(1, geom.V3(fixed.Zero, fixed.FromInt(3), fixed.FromInt(-25)))

	settleDriveabilityVehicle(&v, g, dt, 180)
	startX := v.Position.X
	startZ := v.Position.Z
	startY := v.Position.Y

	trace := traceDriveability(&v, g, dt, 240, func(i int) VehicleInput {
		return VehicleInput{
			Throttle: fixed.One,
			Steer:    fixed.FromFraction(15, 100), // 0.15 instead of 0.40
		}
	})

	deltaZ := trace[len(trace)-1].Pos.Z.Sub(startZ)
	if deltaZ.Cmp(p.SlopeSteerMinProgressZ) <= 0 {
		t.Fatalf("[%s] slope-steer forward progress too small: got=%v need>%v", p.Name, deltaZ, p.SlopeSteerMinProgressZ)
	}

	deltaXAbs := trace[len(trace)-1].Pos.X.Sub(startX).Abs()
	if deltaXAbs.Cmp(p.SlopeSteerMinArcXAbs) <= 0 {
		t.Fatalf("[%s] slope-steer arc too small: got=%v need>%v", p.Name, deltaXAbs, p.SlopeSteerMinArcXAbs)
	}

	deltaY := trace[len(trace)-1].Pos.Y.Sub(startY)
	if deltaY.Cmp(p.SlopeSteerMinGainY) <= 0 {
		t.Fatalf("[%s] slope-steer altitude gain too small: got=%v need>%v", p.Name, deltaY, p.SlopeSteerMinGainY)
	}

	gr := groundedRatio(trace)
	if gr.Cmp(p.SlopeSteerGroundedMin) < 0 {
		t.Fatalf("[%s] grounded ratio too low on slope-steer: got=%v need>=%v", p.Name, gr, p.SlopeSteerGroundedMin)
	}

	maxYawVel := maxAbsYawVel(trace)
	if maxYawVel.Cmp(p.SlopeSteerMinMaxYawVel) <= 0 {
		t.Fatalf("[%s] slope-steer yaw rate too weak: got=%v need>%v", p.Name, maxYawVel, p.SlopeSteerMinMaxYawVel)
	}
}
