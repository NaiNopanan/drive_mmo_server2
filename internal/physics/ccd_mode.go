package physics

type CCDMode int

const (
	CCDModeDiscrete CCDMode = iota
	CCDModeSweepAxisAlignedBoundingBox
	CCDModeSweepTriangleMesh
	CCDModeSweepRotatingOrientedBoxAxisAlignedBoundingBox
	CCDModeSweepRotatingOrientedBoxTriangleMesh
)

func (mode CCDMode) String() string {
	switch mode {
	case CCDModeSweepAxisAlignedBoundingBox:
		return "sweep-aabb"
	case CCDModeSweepTriangleMesh:
		return "sweep-mesh"
	case CCDModeSweepRotatingOrientedBoxAxisAlignedBoundingBox:
		return "sweep-obb-aabb"
	case CCDModeSweepRotatingOrientedBoxTriangleMesh:
		return "sweep-obb-mesh"
	default:
		return "discrete"
	}
}
