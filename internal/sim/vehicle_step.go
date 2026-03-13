package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

var (
	VehicleGravity = geom.V3(fixed.Zero, fixed.FromFraction(-98, 10), fixed.Zero)
)

func (v *Vehicle) Step(dt fixed.Fixed, g GroundQuery) {
	v.UpdateBasisFromYaw()
	v.UpdateSteering(dt)

	v.TotalForce = geom.Zero()
	v.TotalTorqueY = fixed.Zero

	for i := range v.Wheels {
		v.queryWheelGround(i, g)
		v.computeWheelSuspensionForce(i, dt)

		if !v.Wheels[i].InContact {
			continue
		}

		f := v.computeWheelMotionForces(i)
		v.TotalForce = v.TotalForce.Add(f)

		// torque = cross(r, f).Y
		r := v.Wheels[i].ContactPoint.Sub(v.Position)
		torque := r.X.Mul(f.Z).Sub(r.Z.Mul(f.X))
		v.TotalTorqueY = v.TotalTorqueY.Add(torque)
	}

	v.Integrate(dt, VehicleGravity)
	v.finalizeSuspensionState()
}

type VehicleWorldDay6 struct {
	Tick            uint64
	GroundTriangles []geom.Triangle
	Vehicles        []Vehicle
}

func (w *VehicleWorldDay6) Step(dt fixed.Fixed) {
	query := WorldGroundQuery{Triangles: w.GroundTriangles}
	for i := range w.Vehicles {
		w.Vehicles[i].Step(dt, query)
	}
	w.Tick++
}
