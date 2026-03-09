package world

import (
	"server2/internal/fixed"
	"server2/internal/fixed3d"
)

type Config struct {
	FixedDt fixed.Fixed
	Gravity fixed3d.Vec3
}

func DefaultConfig() Config {
	return Config{
		FixedDt: fixed.FromRatio(1, 60),
		Gravity: fixed3d.V3(
			fixed.Zero,
			fixed.FromRatio(-981, 100), // -9.81
			fixed.Zero,
		),
	}
}
