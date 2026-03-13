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
	
	// Apply drag to yaw to stop rotation eventually
	v.YawVelocity = v.YawVelocity.Mul(fixed.FromFraction(95, 100)) 

	v.Yaw = v.Yaw.Add(v.YawVelocity.Mul(dt))
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
