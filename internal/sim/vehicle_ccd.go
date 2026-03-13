package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type VehiclePose struct {
	Position geom.Vec3
	Yaw      fixed.Fixed
}

type TOIResult struct {
	Hit    bool
	Time   fixed.Fixed // 0..1
	Wheel  int
	Normal geom.Vec3
	Depth  fixed.Fixed
}

func (v *Vehicle) minSuspensionLength() fixed.Fixed {
	minLen := v.Tuning.SuspensionRestLength.Sub(v.Tuning.SuspensionMaxRaise)
	if minLen.Cmp(fixed.Zero) < 0 {
		return fixed.Zero
	}
	return minLen
}

func (v *Vehicle) wheelAnchorWorldFromPose(i int, pose VehiclePose) geom.Vec3 {
	fwd, right := HeadingFromYaw(pose.Yaw)
	local := v.WheelDefs[i].LocalAnchor

	return geom.V3(
		pose.Position.X.Add(right.X.Mul(local.X)).Add(fwd.X.Mul(local.Z)),
		pose.Position.Y.Add(local.Y),
		pose.Position.Z.Add(right.Z.Mul(local.X)).Add(fwd.Z.Mul(local.Z)),
	)
}

func (v *Vehicle) predictFromCurrentState(dt fixed.Fixed, gravity geom.Vec3) (nextPos, nextVel geom.Vec3, nextYaw, nextYawVel fixed.Fixed) {
	gravityForce := gravity.Scale(v.Tuning.Mass)
	totalForce := v.TotalForce.Add(gravityForce)

	acc := totalForce.Scale(v.Tuning.InvMass)
	nextVel = v.Velocity.Add(acc.Scale(dt))
	nextVel = clampSpeedXZ(nextVel, v.Tuning.MaxSpeed)

	nextPos = v.Position.Add(nextVel.Scale(dt))

	yawAcc := v.TotalTorqueY.Mul(v.Tuning.InvYawInertia)
	nextYawVel = v.YawVelocity.Add(yawAcc.Mul(dt))

	// Yaw drag: 10% per tick (standardized with Integrate)
	nextYawVel = nextYawVel.Mul(fixed.FromFraction(90, 100))

	// Snap yaw velocity only if extremely small
	yawSnap := fixed.FromFraction(1, 1000) // 0.001 rad/s
	if nextYawVel.Abs().Cmp(yawSnap) < 0 {
		nextYawVel = fixed.Zero
	}

	nextYaw = v.Yaw.Add(nextYawVel.Mul(dt))

	return
}

func (v *Vehicle) FindGroundTOI(pose0, pose1 VehiclePose, g GroundQuery) TOIResult {
	switch gg := g.(type) {
	case FlatGround:
		return v.findGroundTOIFlat(pose0, pose1, gg)
	case *FlatGround:
		return v.findGroundTOIFlat(pose0, pose1, *gg)
	case SlopeGround:
		return v.findGroundTOISlope(pose0, pose1, gg)
	case *SlopeGround:
		return v.findGroundTOISlope(pose0, pose1, *gg)
	case WorldGroundQuery:
		return v.findGroundTOIWorld(pose0, pose1, gg)
	case *WorldGroundQuery:
		return v.findGroundTOIWorld(pose0, pose1, *gg)
	default:
		// Unknown GroundQuery type
		return TOIResult{}
	}
}

func (v *Vehicle) findGroundTOIWorld(pose0, pose1 VehiclePose, g WorldGroundQuery) TOIResult {
	bestT := fixed.One
	bestWheel := -1
	var bestNormal geom.Vec3
	var bestDepth fixed.Fixed

	allowedOffset := v.Tuning.WheelRadius.Add(v.minSuspensionLength())

	for i := range v.WheelDefs {
		a0 := v.wheelAnchorWorldFromPose(i, pose0)
		a1 := v.wheelAnchorWorldFromPose(i, pose1)

		// 1) Sample ground at start
		hit0 := sampleGroundAtXZ(g.Triangles, a0.X, a0.Z, a0.Y.Add(fixed.FromInt(5))) // look slightly above
		allowedY0 := fixed.FromInt(-10000)
		if hit0.Hit {
			allowedY0 = hit0.Point.Y.Add(allowedOffset)
		}

		if hit0.Hit && a0.Y.Cmp(allowedY0) <= 0 {
			// Already penetrating
			depth := allowedY0.Sub(a0.Y)
			return TOIResult{Hit: true, Time: fixed.Zero, Wheel: i, Normal: hit0.Normal, Depth: depth}
		}

		// 2) Check if end is below ground
		hit1 := sampleGroundAtXZ(g.Triangles, a1.X, a1.Z, a1.Y.Add(fixed.FromInt(5)))
		if !hit1.Hit {
			continue // No ground here at destination
		}
		allowedY1 := hit1.Point.Y.Add(allowedOffset)

		if a1.Y.Cmp(allowedY1) < 0 {
			// Crosses ground!
			// Iterative search for T (2 iterations is usually enough for fixed-point vehicle physics)
			t := fixed.Zero
			low := fixed.Zero
			high := fixed.One

			for iter := 0; iter < 2; iter++ {
				mid := low.Add(high).Mul(fixed.FromFraction(5, 10))
				aMid := a0.Add(a1.Sub(a0).Scale(mid))
				hitMid := sampleGroundAtXZ(g.Triangles, aMid.X, aMid.Z, aMid.Y.Add(fixed.FromInt(5)))

				if hitMid.Hit && aMid.Y.Cmp(hitMid.Point.Y.Add(allowedOffset)) < 0 {
					high = mid
				} else {
					low = mid
				}
			}
			t = high
			if t.Cmp(bestT) < 0 {
				bestT = t
				bestWheel = i
				bestNormal = hit1.Normal
				bestDepth = fixed.Zero // Hit time 0+ has minimal depth
			}
		}
	}

	if bestWheel < 0 {
		return TOIResult{}
	}
	return TOIResult{Hit: true, Time: bestT, Wheel: bestWheel, Normal: bestNormal, Depth: bestDepth}
}

func (v *Vehicle) findGroundTOIFlat(pose0, pose1 VehiclePose, g FlatGround) TOIResult {
	bestT := fixed.One
	bestWheel := -1

	allowedOffset := v.Tuning.WheelRadius.Add(v.minSuspensionLength())
	normal := geom.V3(fixed.Zero, fixed.One, fixed.Zero)

	for i := range v.WheelDefs {
		a0 := v.wheelAnchorWorldFromPose(i, pose0)
		a1 := v.wheelAnchorWorldFromPose(i, pose1)

		allowedY := g.Y.Add(allowedOffset)

		if a0.Y.Cmp(allowedY) <= 0 {
			// Already penetrating - but only if inside boundary
			if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
				if a0.X.Cmp(g.MinX) < 0 || a0.X.Cmp(g.MaxX) > 0 || a0.Z.Cmp(g.MinZ) < 0 || a0.Z.Cmp(g.MaxZ) > 0 {
					continue // off the edge, skip this wheel
				}
			}
			depth := allowedY.Sub(a0.Y)
			return TOIResult{Hit: true, Time: fixed.Zero, Wheel: i, Normal: normal, Depth: depth}
		}

		if a0.Y.Cmp(allowedY) > 0 && a1.Y.Cmp(allowedY) < 0 {
			denom := a0.Y.Sub(a1.Y)
			if denom.Cmp(fixed.Zero) > 0 {
				t := a0.Y.Sub(allowedY).Div(denom)
				if t.Cmp(bestT) < 0 {
					// Check boundaries at collision point
					hitX := a0.X.Add(a1.X.Sub(a0.X).Mul(t))
					hitZ := a0.Z.Add(a1.Z.Sub(a0.Z).Mul(t))

					if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
						if hitX.Cmp(g.MinX) >= 0 && hitX.Cmp(g.MaxX) <= 0 && hitZ.Cmp(g.MinZ) >= 0 && hitZ.Cmp(g.MaxZ) <= 0 {
							bestT = t
							bestWheel = i
						}
					} else {
						// Infinite ground
						bestT = t
						bestWheel = i
					}
				}
			}
		}
	}

	if bestWheel < 0 {
		return TOIResult{}
	}
	return TOIResult{Hit: true, Time: bestT, Wheel: bestWheel, Normal: normal}
}

func (v *Vehicle) findGroundTOISlope(pose0, pose1 VehiclePose, g SlopeGround) TOIResult {
	bestT := fixed.One
	bestWheel := -1
	normal := geom.V3(fixed.Zero, fixed.One, g.Slope.Neg()).Normalize()

	allowedOffset := v.Tuning.WheelRadius.Add(v.minSuspensionLength())

	for i := range v.WheelDefs {
		a0 := v.wheelAnchorWorldFromPose(i, pose0)
		a1 := v.wheelAnchorWorldFromPose(i, pose1)

		groundY0 := g.BaseY.Add(g.Slope.Mul(a0.Z))
		allowedY0 := groundY0.Add(allowedOffset)

		if a0.Y.Cmp(allowedY0) <= 0 {
			// Already penetrating - but only if inside boundary
			if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
				if a0.X.Cmp(g.MinX) < 0 || a0.X.Cmp(g.MaxX) > 0 || a0.Z.Cmp(g.MinZ) < 0 || a0.Z.Cmp(g.MaxZ) > 0 {
					continue // off the edge, skip this wheel
				}
			}
			depth := allowedY0.Sub(a0.Y)
			return TOIResult{Hit: true, Time: fixed.Zero, Wheel: i, Normal: normal, Depth: depth}
		}

		t, ok := solveSlopeTOI(a0.Y, a1.Y, a0.Z, a1.Z, g.BaseY, g.Slope, allowedOffset)
		if !ok {
			continue
		}

		if t.Cmp(bestT) < 0 {
			// Check boundaries at collision point
			hitX := a0.X.Add(a1.X.Sub(a0.X).Mul(t))
			hitZ := a0.Z.Add(a1.Z.Sub(a0.Z).Mul(t))

			if g.MinX.Cmp(fixed.Zero) != 0 || g.MaxX.Cmp(fixed.Zero) != 0 || g.MinZ.Cmp(fixed.Zero) != 0 || g.MaxZ.Cmp(fixed.Zero) != 0 {
				if hitX.Cmp(g.MinX) >= 0 && hitX.Cmp(g.MaxX) <= 0 && hitZ.Cmp(g.MinZ) >= 0 && hitZ.Cmp(g.MaxZ) <= 0 {
					bestT = t
					bestWheel = i
				}
			} else {
				// Infinite ground
				bestT = t
				bestWheel = i
			}
		}
	}

	if bestWheel < 0 {
		return TOIResult{}
	}
	return TOIResult{Hit: true, Time: bestT, Wheel: bestWheel, Normal: normal}
}

func solveSlopeTOI(y0, y1, z0, z1, baseY, slope, offset fixed.Fixed) (fixed.Fixed, bool) {
	dy := y1.Sub(y0)
	dz := z1.Sub(z0)

	lhs := dy.Sub(slope.Mul(dz))
	rhs := baseY.Add(slope.Mul(z0)).Add(offset).Sub(y0)

	if lhs.Cmp(fixed.Zero) == 0 {
		return fixed.Zero, false
	}

	t := rhs.Div(lhs)
	if t.Cmp(fixed.Zero) < 0 || t.Cmp(fixed.One) > 0 {
		return fixed.Zero, false
	}
	return t, true
}

func (v *Vehicle) resolveGroundImpactVelocity(n geom.Vec3) {
	vn := v.Velocity.Dot(n)
	if vn.Cmp(fixed.Zero) < 0 {
		v.Velocity = v.Velocity.Sub(n.Scale(vn))
	}
	if v.Velocity.Y.Cmp(fixed.Zero) < 0 {
		v.Velocity.Y = fixed.Zero
	}
}

func (v *Vehicle) applyCCDSeparationBias(n geom.Vec3, depth fixed.Fixed) {
	eps := fixed.FromFraction(2, 1000).Add(depth) // 0.002 + penetration depth
	v.Position = v.Position.Add(n.Scale(eps))
}
