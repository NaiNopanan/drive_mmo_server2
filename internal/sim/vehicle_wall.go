package sim

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

// WallAxis defines which axis a wall is on.
type WallAxis int

const (
	WallAxisX WallAxis = iota // wall perpendicular to X axis (blocks X movement)
	WallAxisZ                  // wall perpendicular to Z axis (blocks Z movement)
)

// WallPlane is an axis-aligned vertical wall.
// A wall at X=10 with positive side means the wall's normal points in -X
// (i.e., it is on the +X boundary and pushes vehicles back in -X direction).
type WallPlane struct {
	Axis     WallAxis
	Position fixed.Fixed // position along the axis
	Sign     int         // +1 = wall is on positive side (MaxX / MaxZ), -1 = negative side (MinX / MinZ)
}

// WallBounds is a set of 4 axis-aligned walls forming a rectangular arena.
type WallBounds struct {
	MinX, MaxX, MinZ, MaxZ fixed.Fixed
	Restitution            fixed.Fixed // bounce coefficient, 0 = no bounce, 1 = full bounce
}

// CollideVehicleWithWalls applies simple wall collision (position clamping + velocity reflection)
// to the vehicle. Should be called after each physics step.
func CollideVehicleWithWalls(v *Vehicle, wb WallBounds) {
	// Half-size of vehicle body derived from tuning dimensions
	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))

	// X boundaries
	if wb.MaxX.Cmp(fixed.Zero) != 0 || wb.MinX.Cmp(fixed.Zero) != 0 {
		limitMaxX := wb.MaxX.Sub(halfWidth)
		limitMinX := wb.MinX.Add(halfWidth)

		if v.Position.X.Cmp(limitMaxX) > 0 {
			v.Position.X = limitMaxX
			if v.Velocity.X.Cmp(fixed.Zero) > 0 {
				bounce := v.Velocity.X.Mul(wb.Restitution).Neg()
				v.Velocity.X = bounce
			}
		} else if v.Position.X.Cmp(limitMinX) < 0 {
			v.Position.X = limitMinX
			if v.Velocity.X.Cmp(fixed.Zero) < 0 {
				bounce := v.Velocity.X.Mul(wb.Restitution).Neg()
				v.Velocity.X = bounce
			}
		}
	}

	// Z boundaries
	if wb.MaxZ.Cmp(fixed.Zero) != 0 || wb.MinZ.Cmp(fixed.Zero) != 0 {
		limitMaxZ := wb.MaxZ.Sub(halfLen)
		limitMinZ := wb.MinZ.Add(halfLen)

		if v.Position.Z.Cmp(limitMaxZ) > 0 {
			v.Position.Z = limitMaxZ
			if v.Velocity.Z.Cmp(fixed.Zero) > 0 {
				bounce := v.Velocity.Z.Mul(wb.Restitution).Neg()
				v.Velocity.Z = bounce
			}
		} else if v.Position.Z.Cmp(limitMinZ) < 0 {
			v.Position.Z = limitMinZ
			if v.Velocity.Z.Cmp(fixed.Zero) < 0 {
				bounce := v.Velocity.Z.Mul(wb.Restitution).Neg()
				v.Velocity.Z = bounce
			}
		}
	}
}

// WallContact represents where a vehicle touched a wall.
type WallContact struct {
	Hit      bool
	Normal   geom.Vec3 // points away from wall (into the arena)
	Depth    fixed.Fixed
}

// CheckWallContact returns the deepest wall penetration for debug visualization.
func CheckWallContact(v *Vehicle, wb WallBounds) WallContact {
	halfWidth := v.Tuning.TrackWidth.Div(fixed.FromInt(2))
	halfLen := v.Tuning.WheelBase.Div(fixed.FromInt(2))
	best := WallContact{}

	checkAndSet := func(depth fixed.Fixed, normal geom.Vec3) {
		if depth.Cmp(fixed.Zero) > 0 {
			if !best.Hit || depth.Cmp(best.Depth) > 0 {
				best.Hit = true
				best.Depth = depth
				best.Normal = normal
			}
		}
	}

	if wb.MaxX.Cmp(fixed.Zero) != 0 {
		checkAndSet(v.Position.X.Add(halfWidth).Sub(wb.MaxX), geom.V3(fixed.One.Neg(), fixed.Zero, fixed.Zero))
		checkAndSet(wb.MinX.Sub(v.Position.X.Sub(halfWidth)), geom.V3(fixed.One, fixed.Zero, fixed.Zero))
	}
	if wb.MaxZ.Cmp(fixed.Zero) != 0 {
		checkAndSet(v.Position.Z.Add(halfLen).Sub(wb.MaxZ), geom.V3(fixed.Zero, fixed.Zero, fixed.One.Neg()))
		checkAndSet(wb.MinZ.Sub(v.Position.Z.Sub(halfLen)), geom.V3(fixed.Zero, fixed.Zero, fixed.One))
	}

	return best
}
