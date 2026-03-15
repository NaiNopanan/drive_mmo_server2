package scenario

import (
	"encoding/binary"
	"hash/fnv"

	"server2/internal/fixed"
	"server2/internal/geometry"
	"server2/internal/physics"
)

func HashSceneState(state SceneState) uint64 {
	hasher := fnv.New64a()
	buffer := make([]byte, 8)

	writeUint64 := func(value uint64) {
		binary.LittleEndian.PutUint64(buffer, value)
		_, _ = hasher.Write(buffer)
	}

	writeInt64 := func(value int64) {
		binary.LittleEndian.PutUint64(buffer, uint64(value))
		_, _ = hasher.Write(buffer)
	}

	writeBool := func(value bool) {
		if value {
			writeUint64(1)
			return
		}
		writeUint64(0)
	}

	writeFixed := func(value fixed.Fixed) {
		writeInt64(value.Raw())
	}

	writeVector3 := func(value geometry.Vector3) {
		writeFixed(value.X)
		writeFixed(value.Y)
		writeFixed(value.Z)
	}

	writeTriangle := func(value geometry.Triangle) {
		writeVector3(value.A)
		writeVector3(value.B)
		writeVector3(value.C)
	}

	writeContact := func(value physics.SphereTriangleContact) {
		writeBool(value.Hit)
		writeVector3(value.Point)
		writeVector3(value.Normal)
		writeFixed(value.Penetration)
	}

	writeUint64(state.Tick)
	writeVector3(state.Sphere.Motion.Position)
	writeVector3(state.Sphere.Motion.Velocity)
	writeVector3(state.Sphere.Motion.AccumulatedForce)
	writeFixed(state.Sphere.Motion.Mass)
	writeFixed(state.Sphere.Motion.InverseMass)
	writeFixed(state.Sphere.Radius)
	writeFixed(state.Sphere.Restitution)
	writeBool(state.Sphere.Grounded)
	writeBool(state.EverTouchedGround)
	writeBool(state.BounceDetected)
	writeFixed(state.PeakBounceHeight)
	writeContact(state.LastContact)
	writeUint64(uint64(len(state.Spheres)))
	for _, sphere := range state.Spheres {
		writeVector3(sphere.Motion.Position)
		writeVector3(sphere.Motion.Velocity)
		writeVector3(sphere.Motion.AccumulatedForce)
		writeFixed(sphere.Motion.Mass)
		writeFixed(sphere.Motion.InverseMass)
		writeFixed(sphere.Radius)
		writeFixed(sphere.Restitution)
		writeBool(sphere.Grounded)
	}
	writeUint64(uint64(len(state.LastContacts)))
	for _, contact := range state.LastContacts {
		writeContact(contact)
	}
	writeUint64(uint64(len(state.GroundTriangles)))
	for _, triangle := range state.GroundTriangles {
		writeTriangle(triangle)
	}

	return hasher.Sum64()
}
