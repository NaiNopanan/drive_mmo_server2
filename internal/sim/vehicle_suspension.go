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
		hit = stickyGroundHit(rayOrigin, rayLen, w, g)
	}

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

func stickyGroundHit(rayOrigin geom.Vec3, rayLen fixed.Fixed, w *WheelState, g GroundQuery) GroundHit {
	const (
		stickyLenNum = 3
		stickyLenDen = 20 // 0.15m
		stickyXZNum  = 7
		stickyXZDen  = 20 // 0.35m
	)

	extendedLen := rayLen.Add(fixed.FromFraction(stickyLenNum, stickyLenDen))

	switch gg := g.(type) {
	case WorldGroundQuery:
		if hit := sampleGroundAtXZ(gg.Triangles, rayOrigin.X, rayOrigin.Z, rayOrigin.Y); hit.Hit {
			dist := rayOrigin.Y.Sub(hit.Point.Y)
			if dist.Cmp(fixed.Zero) >= 0 && dist.Cmp(extendedLen) <= 0 {
				return GroundHit{
					Hit:      true,
					Point:    hit.Point,
					Normal:   hit.Normal,
					Distance: dist,
				}
			}
		}
	case *WorldGroundQuery:
		if hit := sampleGroundAtXZ(gg.Triangles, rayOrigin.X, rayOrigin.Z, rayOrigin.Y); hit.Hit {
			dist := rayOrigin.Y.Sub(hit.Point.Y)
			if dist.Cmp(fixed.Zero) >= 0 && dist.Cmp(extendedLen) <= 0 {
				return GroundHit{
					Hit:      true,
					Point:    hit.Point,
					Normal:   hit.Normal,
					Distance: dist,
				}
			}
		}
	default:
		return GroundHit{}
	}

	if !w.InContact {
		return GroundHit{}
	}

	maxXZDrift := fixed.FromFraction(stickyXZNum, stickyXZDen)
	dx := rayOrigin.X.Sub(w.ContactPoint.X).Abs()
	dz := rayOrigin.Z.Sub(w.ContactPoint.Z).Abs()
	dist := rayOrigin.Y.Sub(w.ContactPoint.Y)

	if dx.Cmp(maxXZDrift) > 0 || dz.Cmp(maxXZDrift) > 0 {
		return GroundHit{}
	}
	if dist.Cmp(fixed.Zero) < 0 || dist.Cmp(extendedLen) > 0 {
		return GroundHit{}
	}

	return GroundHit{
		Hit:      true,
		Point:    geom.V3(rayOrigin.X, w.ContactPoint.Y, rayOrigin.Z),
		Normal:   w.ContactNormal,
		Distance: dist,
	}
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
