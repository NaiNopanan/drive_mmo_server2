package geometry

type AxisAlignedBoundingBox struct {
	Min Vector3
	Max Vector3
}

func NewAxisAlignedBoundingBox(min, max Vector3) AxisAlignedBoundingBox {
	if min.X > max.X || min.Y > max.Y || min.Z > max.Z {
		panic("invalid axis-aligned bounding box: min > max")
	}
	return AxisAlignedBoundingBox{Min: min, Max: max}
}

func (a AxisAlignedBoundingBox) Intersects(b AxisAlignedBoundingBox) bool {
	if a.Max.X < b.Min.X || a.Min.X > b.Max.X {
		return false
	}
	if a.Max.Y < b.Min.Y || a.Min.Y > b.Max.Y {
		return false
	}
	if a.Max.Z < b.Min.Z || a.Min.Z > b.Max.Z {
		return false
	}
	return true
}

func (a AxisAlignedBoundingBox) ContainsPoint(p Vector3) bool {
	if p.X < a.Min.X || p.X > a.Max.X {
		return false
	}
	if p.Y < a.Min.Y || p.Y > a.Max.Y {
		return false
	}
	if p.Z < a.Min.Z || p.Z > a.Max.Z {
		return false
	}
	return true
}
