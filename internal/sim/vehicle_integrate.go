package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

func (v *Vehicle) Integrate(dt fixed.Fixed, gravity geom.Vec3) {
	// Gravity force
	gravityForce := gravity.Scale(v.Tuning.Mass)
	v.TotalForce = v.TotalForce.Add(gravityForce)

	// Linear acceleration
	acc := v.TotalForce.Scale(v.Tuning.InvMass)
	v.Velocity = v.Velocity.Add(acc.Scale(dt))

	// Clamp speed (XZ only for road feel)
	v.Velocity = clampSpeedXZ(v.Velocity, v.Tuning.MaxSpeed)

	// Update position
	v.Position = v.Position.Add(v.Velocity.Scale(dt))

	// Yaw dynamics
	yawAcc := v.TotalTorqueY.Mul(v.Tuning.InvYawInertia)
	v.YawVelocity = v.YawVelocity.Add(yawAcc.Mul(dt))
	
	// Apply yaw drag (10% per tick) and snap very small values to zero
	v.YawVelocity = v.YawVelocity.Mul(fixed.FromFraction(90, 100))
	yawSnapThreshold := fixed.FromFraction(1, 1000) // 0.001 rad/s
	if v.YawVelocity.Abs().Cmp(yawSnapThreshold) < 0 {
		v.YawVelocity = fixed.Zero
	}

	v.Yaw = v.Yaw.Add(v.YawVelocity.Mul(dt))

	// Apply very light XZ velocity damping (0.2% per tick) to ensure the car
	// eventually comes to rest after releasing controls. Without this, lateral
	// velocity from steering has no decay path when LatSpeed drops below the threshold.
	damping := fixed.FromFraction(998, 1000) // 0.2% per tick
	v.Velocity.X = v.Velocity.X.Mul(damping)
	v.Velocity.Z = v.Velocity.Z.Mul(damping)

	// Snap very small XZ velocity to zero (avoids infinite micro-jitter)
	velSnapThreshold := fixed.FromFraction(1, 100) // 0.01 m/s
	if v.Velocity.X.Abs().Cmp(velSnapThreshold) < 0 {
		v.Velocity.X = fixed.Zero
	}
	if v.Velocity.Z.Abs().Cmp(velSnapThreshold) < 0 {
		v.Velocity.Z = fixed.Zero
	}
}

func clampSpeedXZ(v geom.Vec3, max fixed.Fixed) geom.Vec3 {
	h := geom.V3(v.X, fixed.Zero, v.Z)
	ls := h.LengthSq()
	if ls.Cmp(max.Mul(max)) > 0 {
		l := ls.Sqrt()
		scale := max.Div(l)
		v.X = v.X.Mul(scale)
		v.Z = v.Z.Mul(scale)
	}
	return v
}
