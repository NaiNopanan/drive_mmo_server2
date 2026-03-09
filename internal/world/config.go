package world

import (
	"server2/internal/fixed"
	"server2/internal/geom"
)

type Config struct {
	FixedDt fixed.Fixed
	Gravity geom.Vec3
}

func DefaultConfig() Config {
	return Config{
		FixedDt: fixed.FromFraction(1, 60),
		Gravity: geom.V3(
			fixed.Zero,
			fixed.FromFraction(-981, 100), // -9.81
			fixed.Zero,
		),
	}
}
