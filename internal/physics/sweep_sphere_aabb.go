package physics

import (
	"server2/internal/fixed"
	"server2/internal/geometry"
)

type SweepSphereAxisAlignedBoundingBoxContact struct {
	Hit          bool
	TimeOfImpact fixed.Fixed
	Normal       geometry.Vector3
	Position     geometry.Vector3
	ContactPoint geometry.Vector3
}

func SweepSphereAxisAlignedBoundingBox(
	startPosition geometry.Vector3,
	velocity geometry.Vector3,
	radius fixed.Fixed,
	dt fixed.Fixed,
	bounds geometry.AxisAlignedBoundingBox,
) SweepSphereAxisAlignedBoundingBoxContact {
	expandedBounds := geometry.NewAxisAlignedBoundingBox(
		bounds.Min.Sub(geometry.NewVector3(radius, radius, radius)),
		bounds.Max.Add(geometry.NewVector3(radius, radius, radius)),
	)
	delta := velocity.Scale(dt)
	tEnter := fixed.Zero
	tExit := fixed.One
	hitNormal := geometry.ZeroVector3()

	updateAxis := func(start, movement, minValue, maxValue fixed.Fixed, minNormal, maxNormal geometry.Vector3) bool {
		if movement == fixed.Zero {
			return start.Cmp(minValue) >= 0 && start.Cmp(maxValue) <= 0
		}

		t1 := minValue.Sub(start).Div(movement)
		t2 := maxValue.Sub(start).Div(movement)
		enterNormal := minNormal
		exitNormal := maxNormal
		if t1.Cmp(t2) > 0 {
			t1, t2 = t2, t1
			enterNormal, exitNormal = exitNormal, enterNormal
		}

		if t1.Cmp(tEnter) > 0 {
			tEnter = t1
			hitNormal = enterNormal
		}
		if t2.Cmp(tExit) < 0 {
			tExit = t2
		}

		return tEnter.Cmp(tExit) <= 0
	}

	if !updateAxis(
		startPosition.X,
		delta.X,
		expandedBounds.Min.X,
		expandedBounds.Max.X,
		geometry.NewVector3(fixed.One.Neg(), fixed.Zero, fixed.Zero),
		geometry.NewVector3(fixed.One, fixed.Zero, fixed.Zero),
	) {
		return SweepSphereAxisAlignedBoundingBoxContact{}
	}
	if !updateAxis(
		startPosition.Y,
		delta.Y,
		expandedBounds.Min.Y,
		expandedBounds.Max.Y,
		geometry.NewVector3(fixed.Zero, fixed.One.Neg(), fixed.Zero),
		geometry.NewVector3(fixed.Zero, fixed.One, fixed.Zero),
	) {
		return SweepSphereAxisAlignedBoundingBoxContact{}
	}
	if !updateAxis(
		startPosition.Z,
		delta.Z,
		expandedBounds.Min.Z,
		expandedBounds.Max.Z,
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One.Neg()),
		geometry.NewVector3(fixed.Zero, fixed.Zero, fixed.One),
	) {
		return SweepSphereAxisAlignedBoundingBoxContact{}
	}

	if tEnter.Cmp(fixed.Zero) < 0 || tEnter.Cmp(fixed.One) > 0 {
		return SweepSphereAxisAlignedBoundingBoxContact{}
	}

	position := startPosition.Add(delta.Scale(tEnter))
	return SweepSphereAxisAlignedBoundingBoxContact{
		Hit:          true,
		TimeOfImpact: tEnter,
		Normal:       hitNormal,
		Position:     position,
		ContactPoint: position.Sub(hitNormal.Scale(radius)),
	}
}
