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
	writeBool(state.SphereSphereCollisionDetected)
	writeUint64(uint64(len(state.BounceDetectedSet)))
	for _, value := range state.BounceDetectedSet {
		writeBool(value)
	}
	writeFixed(state.PeakBounceHeight)
	writeUint64(uint64(len(state.PeakBounceHeights)))
	for _, value := range state.PeakBounceHeights {
		writeFixed(value)
	}
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
	writeVector3(state.RigidSphere.Motion.Position)
	writeVector3(state.RigidSphere.Motion.Velocity)
	writeVector3(state.RigidSphere.Motion.AccumulatedForce)
	writeFixed(state.RigidSphere.Motion.Mass)
	writeFixed(state.RigidSphere.Motion.InverseMass)
	writeFixed(state.RigidSphere.Radius)
	writeFixed(state.RigidSphere.Restitution)
	writeFixed(state.RigidSphere.Friction)
	writeBool(state.RigidSphere.Grounded)
	writeFixed(state.RigidSphere.Orientation.W)
	writeFixed(state.RigidSphere.Orientation.X)
	writeFixed(state.RigidSphere.Orientation.Y)
	writeFixed(state.RigidSphere.Orientation.Z)
	writeVector3(state.RigidSphere.AngularVelocity)
	writeFixed(state.RigidSphere.InverseInertia)
	writeBool(state.RigidSphereBounceDetected)
	writeFixed(state.RigidSpherePeakBounceHeight)
	writeBool(state.RigidSphereRotationChanged)
	writeUint64(uint64(len(state.RigidSphereTouchedGroundSet)))
	for _, value := range state.RigidSphereTouchedGroundSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.RigidSpheres)))
	for _, sphere := range state.RigidSpheres {
		writeVector3(sphere.Motion.Position)
		writeVector3(sphere.Motion.Velocity)
		writeVector3(sphere.Motion.AccumulatedForce)
		writeFixed(sphere.Motion.Mass)
		writeFixed(sphere.Motion.InverseMass)
		writeFixed(sphere.Radius)
		writeFixed(sphere.Restitution)
		writeFixed(sphere.Friction)
		writeBool(sphere.Grounded)
		writeFixed(sphere.Orientation.W)
		writeFixed(sphere.Orientation.X)
		writeFixed(sphere.Orientation.Y)
		writeFixed(sphere.Orientation.Z)
		writeVector3(sphere.AngularVelocity)
		writeFixed(sphere.InverseInertia)
	}
	writeUint64(uint64(len(state.RigidSphereBounceDetectedSet)))
	for _, value := range state.RigidSphereBounceDetectedSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.RigidSpherePeakBounceHeights)))
	for _, value := range state.RigidSpherePeakBounceHeights {
		writeFixed(value)
	}
	writeUint64(uint64(len(state.RigidSphereRotationChangedSet)))
	for _, value := range state.RigidSphereRotationChangedSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.LastContacts)))
	for _, contact := range state.LastContacts {
		writeContact(contact)
	}
	writeVector3(state.Box.Motion.Position)
	writeVector3(state.Box.Motion.Velocity)
	writeVector3(state.Box.Motion.AccumulatedForce)
	writeFixed(state.Box.Motion.Mass)
	writeFixed(state.Box.Motion.InverseMass)
	writeVector3(state.Box.HalfExtents)
	writeFixed(state.Box.Restitution)
	writeFixed(state.Box.RotationZ)
	writeFixed(state.Box.AngularVelocityZ)
	writeFixed(state.Box.InverseInertiaZ)
	writeBool(state.Box.Grounded)
	writeUint64(uint64(len(state.Boxes)))
	for _, box := range state.Boxes {
		writeVector3(box.Motion.Position)
		writeVector3(box.Motion.Velocity)
		writeVector3(box.Motion.AccumulatedForce)
		writeFixed(box.Motion.Mass)
		writeFixed(box.Motion.InverseMass)
		writeVector3(box.HalfExtents)
		writeFixed(box.Restitution)
		writeFixed(box.RotationZ)
		writeFixed(box.AngularVelocityZ)
		writeFixed(box.InverseInertiaZ)
		writeBool(box.Grounded)
	}
	writeUint64(uint64(len(state.GroundBoxes)))
	for _, box := range state.GroundBoxes {
		writeVector3(box.Min)
		writeVector3(box.Max)
	}
	writeUint64(uint64(len(state.BoxBounceDetectedSet)))
	for _, value := range state.BoxBounceDetectedSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.BoxPeakBounceHeights)))
	for _, value := range state.BoxPeakBounceHeights {
		writeFixed(value)
	}
	writeUint64(uint64(len(state.BoxInitialRotations)))
	for _, value := range state.BoxInitialRotations {
		writeFixed(value)
	}
	writeUint64(uint64(len(state.BoxRotationChangedSet)))
	for _, value := range state.BoxRotationChangedSet {
		writeBool(value)
	}
	writeVector3(state.RigidBox.Motion.Position)
	writeVector3(state.RigidBox.Motion.Velocity)
	writeVector3(state.RigidBox.Motion.AccumulatedForce)
	writeFixed(state.RigidBox.Motion.Mass)
	writeFixed(state.RigidBox.Motion.InverseMass)
	writeVector3(state.RigidBox.HalfExtents)
	writeFixed(state.RigidBox.Restitution)
	writeBool(state.RigidBox.Grounded)
	writeFixed(state.RigidBox.Orientation.W)
	writeFixed(state.RigidBox.Orientation.X)
	writeFixed(state.RigidBox.Orientation.Y)
	writeFixed(state.RigidBox.Orientation.Z)
	writeVector3(state.RigidBox.AngularVelocity)
	writeVector3(state.RigidBox.InverseInertiaBody)
	writeBool(state.RigidBoxBounceDetected)
	writeFixed(state.RigidBoxPeakBounceHeight)
	writeBool(state.RigidBoxRotationChanged)
	writeUint64(uint64(len(state.RigidBoxes)))
	for _, box := range state.RigidBoxes {
		writeVector3(box.Motion.Position)
		writeVector3(box.Motion.Velocity)
		writeVector3(box.Motion.AccumulatedForce)
		writeFixed(box.Motion.Mass)
		writeFixed(box.Motion.InverseMass)
		writeVector3(box.HalfExtents)
		writeFixed(box.Restitution)
		writeBool(box.Grounded)
		writeFixed(box.Orientation.W)
		writeFixed(box.Orientation.X)
		writeFixed(box.Orientation.Y)
		writeFixed(box.Orientation.Z)
		writeVector3(box.AngularVelocity)
		writeVector3(box.InverseInertiaBody)
	}
	writeUint64(uint64(len(state.RigidBoxBounceDetectedSet)))
	for _, value := range state.RigidBoxBounceDetectedSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.RigidBoxPeakBounceHeights)))
	for _, value := range state.RigidBoxPeakBounceHeights {
		writeFixed(value)
	}
	writeUint64(uint64(len(state.RigidBoxRotationChangedSet)))
	for _, value := range state.RigidBoxRotationChangedSet {
		writeBool(value)
	}
	writeUint64(uint64(len(state.GroundTriangles)))
	for _, triangle := range state.GroundTriangles {
		writeTriangle(triangle)
	}

	return hasher.Sum64()
}
