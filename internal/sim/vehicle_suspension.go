package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

var (
	vecDown = geom.V3(fixed.Zero, fixed.One.Neg(), fixed.Zero)
)

func (v *Vehicle) wheelAnchorWorld(i int) geom.Vec3 {
	offset := v.LocalToWorld(v.WheelDefs[i].LocalAnchor)
	return v.Position.Add(offset)
}

func (v *Vehicle) queryWheelGround(i int, g GroundQuery) {
	w := &v.Wheels[i]

	anchor := v.wheelAnchorWorld(i)
	// Ray starts slightly above the anchor
	rayOrigin := anchor.Add(geom.V3(fixed.Zero, v.Tuning.SuspensionMaxRaise, fixed.Zero))
	rayLen := v.Tuning.SuspensionRestLength.Add(v.Tuning.SuspensionMaxDrop).
		Add(v.Tuning.SuspensionMaxRaise).Add(v.Tuning.WheelRadius)

	hit := g.Raycast(rayOrigin, vecDown, rayLen)

	if !hit.Hit {
		w.InContact = false
		w.Compression = fixed.Zero
		w.SuspensionForce = fixed.Zero
		return
	}

	w.InContact = true
	w.ContactPoint = hit.Point
	w.ContactNormal = hit.Normal
	w.ContactDistance = hit.Distance

	// compression calculation
	// contactDistance is from rayOrigin to ground.
	// We want distance from anchor to ground.
	distFromAnchor := hit.Distance.Sub(v.Tuning.SuspensionMaxRaise)
	// compression = rest - (distFromAnchor - radius)
	compression := v.Tuning.SuspensionRestLength.Sub(distFromAnchor.Sub(v.Tuning.WheelRadius))
	
	maxComp := v.Tuning.SuspensionMaxDrop.Add(v.Tuning.SuspensionMaxRaise)
	w.Compression = fixed.Clamp(compression, fixed.Zero, maxComp)
}

func (v *Vehicle) computeWheelSuspensionForce(i int, dt fixed.Fixed) {
	w := &v.Wheels[i]
	if !w.InContact {
		return
	}

	w.SpringForce = w.Compression.Mul(v.Tuning.SuspensionStiffness)
	
	compVel := w.Compression.Sub(w.PrevCompression).Div(dt)
	w.DamperForce = compVel.Mul(v.Tuning.SuspensionDamping)

	total := w.SpringForce.Add(w.DamperForce)
	w.SuspensionForce = fixed.Clamp(total, fixed.Zero, v.Tuning.MaxSuspensionForce)
}

func (v *Vehicle) finalizeSuspensionState() {
	v.GroundedWheels = 0
	for i := range v.Wheels {
		if v.Wheels[i].InContact {
			v.GroundedWheels++
		}
		v.Wheels[i].PrevCompression = v.Wheels[i].Compression
	}
	v.OnGround = v.GroundedWheels > 0
}
