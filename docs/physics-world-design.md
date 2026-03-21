# Physics World Design

## Scope

This document defines the `physics world` for a driving game server prototype.

Focus only on:

- world representation
- simulation loop
- vehicle motion
- collisions
- road and lane constraints
- triggers
- NPC traffic simulation

Explicitly out of scope:

- network protocol
- player session handling
- matchmaking
- persistence details
- live-service deployment

The goal is to design a simulation core that can later be embedded into a world server.

## Design Goal

The target is not full realistic car physics.

The target is:

- stable
- deterministic enough for server use
- cheap to simulate
- believable for arcade driving
- easy to tune per vehicle class

For a RayCity-like game, the right answer is usually `arcade physics with strong road constraints`, not rigid-body realism.

## Core Principles

1. Fixed-step simulation
   - Update the world at a constant timestep.
   - Never tie gameplay physics to wall-clock frame rate.

2. Kinematic vehicle model over full rigid body
   - Represent the car with speed, heading, yaw rate, and a simplified body footprint.
   - Do not simulate suspension, wheel colliders, or per-wheel friction in the first version.

3. Collision as gameplay correction, not perfect realism
   - Resolve overlaps and invalid states in a stable, predictable way.
   - Use collision to keep cars inside the playable space and support traffic interactions.

4. Road-aware movement
   - The world should know where roads, lanes, sidewalks, and blocked areas are.
   - Vehicle behavior changes by surface and road segment.

5. Cheap broadphase, simple narrowphase
   - Use spatial partitioning to avoid checking all objects against all objects.
   - Use simple shapes: circles, capsules, AABBs, oriented boxes.

## Simulation Rate

Recommended starting values:

- physics tick: 20 Hz
- fixed dt: 50 ms
- optional substep for collision-heavy situations: 2 x 25 ms

Why 20 Hz:

- cheap enough for many entities
- stable enough for vehicle motion and trigger checks
- easier to reason about than variable-rate updates

If vehicles feel too coarse:

- keep the world at 20 Hz
- add interpolation outside the physics core later

## High-Level World Model

The physics world should own:

- static map geometry used for gameplay
- drivable road graph
- dynamic entities
- collision layers
- trigger volumes
- spatial index

### World Structure

```go
type PhysicsWorld struct {
    Tick          uint64
    FixedDT       float32
    Config        WorldConfig
    Map           *MapData
    Dynamic       *DynamicState
    Spatial       *SpatialIndex
    Triggers      *TriggerIndex
}
```

### MapData

```go
type MapData struct {
    Bounds        AABB
    RoadGraph     *RoadGraph
    StaticColliders []StaticCollider
    SurfaceZones  []SurfaceZone
    SpawnPoints   []SpawnPoint
}
```

### DynamicState

```go
type DynamicState struct {
    Vehicles      map[EntityID]*VehicleBody
    NPCVehicles   map[EntityID]*NPCBody
    DynamicObs    map[EntityID]*DynamicObstacle
}
```

## Coordinate System

Use a simple world-space coordinate system:

- `X`, `Z`: ground plane
- `Y`: height

For the first version:

- most gameplay can be treated as 2.5D
- simulate horizontal motion in `X/Z`
- sample or constrain height from road/map metadata

This reduces complexity dramatically.

## Static World Representation

Do not use full render meshes for server physics.

Build a stripped gameplay map representation:

- road centerlines
- lane segments
- drivable polygons or road bands
- curb/barrier colliders
- building/blocker colliders
- trigger volumes

### Preferred Static Collider Shapes

Use only a few primitives:

- AABB for buildings and blocked zones
- oriented box for barriers and long walls
- capsule for curved divider segments
- polygon only where absolutely necessary

This keeps collision queries fast and implementation manageable.

## Road Graph

The road graph is the backbone of the world.

Each road segment should describe:

- unique id
- centerline path
- width
- lane count
- speed class
- surface type
- allowed travel direction
- adjacency to neighboring segments

### Why a Road Graph Matters

It gives you:

- road-aware speed rules
- NPC traffic paths
- route planning
- off-road detection
- simple recovery when a vehicle becomes invalid

### Segment Shape

Start with piecewise-linear segments:

```go
type RoadSegment struct {
    ID           SegmentID
    Points       []PlanarVec
    HalfWidth    float32
    LaneCount    int
    SpeedLimit   float32
    Surface      SurfaceType
    Next         []SegmentID
}
```

You do not need splines in the first version.

## Surface Model

Vehicles should react differently depending on what they are on.

Suggested surface types:

- `Road`
- `Dirt`
- `Grass`
- `Sidewalk`
- `Water`
- `Blocked`

Each surface can define modifiers:

- traction multiplier
- max speed multiplier
- steering response multiplier
- damage or penalty flag

Example:

- road: normal grip
- dirt: less grip, lower max speed
- sidewalk: allowed briefly but penalized
- blocked: collision / invalid placement

## Dynamic Entity Types

### Player Vehicle

```go
type VehicleBody struct {
    ID            EntityID
    Transform     Transform
    Velocity      PlanarVec
    Speed         float32
    Heading       float32
    YawRate       float32
    Input         DriveInput
    Params        VehicleParams
    State         VehicleState
    LastValid     Transform
}
```

### NPC Vehicle

```go
type NPCBody struct {
    ID            EntityID
    Transform     Transform
    Speed         float32
    Heading       float32
    CurrentLane   LaneRef
    LaneProgress  float32
    Params        NPCParams
}
```

### Dynamic Obstacle

Keep room for things like:

- temporary barricade
- mission blocker
- spawned hazard

But do not overbuild this before needed.

## Vehicle Parameters

Each vehicle should be tuned by parameters, not hardcoded formulas.

```go
type VehicleParams struct {
    MaxForwardSpeed    float32
    MaxReverseSpeed    float32
    AccelRate          float32
    BrakeRate          float32
    Drag               float32
    RollingResistance  float32
    MaxSteerAngle      float32
    SteerResponse      float32
    HighSpeedSteerDamp float32
    Grip               float32
    DriftAllowance     float32
    BodyLength         float32
    BodyWidth          float32
}
```

This gives you enough range to create:

- compact city car
- delivery van
- sports car
- heavy SUV

## Vehicle Input Model

The physics world should consume normalized input:

```go
type DriveInput struct {
    Throttle  float32
    Brake     float32
    Steering  float32
    Handbrake bool
    Nitro     bool
}
```

Ranges:

- throttle: `0..1`
- brake: `0..1`
- steering: `-1..1`

The physics layer should not care where input came from.

## Vehicle Motion Model

Use an `arcade bicycle-like model`, simplified for stability.

Per tick:

1. derive longitudinal acceleration from throttle/brake
2. apply drag and rolling resistance
3. compute effective steering based on input and current speed
4. update yaw rate
5. update heading
6. update horizontal velocity
7. integrate position
8. clamp or correct against world constraints

### Longitudinal Motion

Use a simple equation:

```text
accel = throttle * AccelRate
      - brake * BrakeRate
      - drag(speed)
      - rollingResistance
```

Then clamp:

- forward speed to `MaxForwardSpeed`
- reverse speed to `MaxReverseSpeed`

### Steering

Steering should weaken at high speed.

Conceptually:

```text
effectiveSteer = inputSteer * MaxSteerAngle * speedDamping
```

where `speedDamping` shrinks steering authority as speed rises.

This prevents twitchy unstable motion.

### Grip and Slip

You do not need full tire simulation.

Approximate grip by:

- aligning velocity toward forward heading every tick
- limiting lateral velocity
- relaxing that limit when handbrake or drift mode is active

This gives a controllable arcade feel.

## Suggested Update Formula

Conceptual flow:

```text
forward = unit vector from heading
right   = perpendicular(forward)

forwardSpeed = dot(velocity, forward)
lateralSpeed = dot(velocity, right)

forwardSpeed += accel * dt
lateralSpeed *= lateralGripFactor

heading += yawRate * dt
velocity = forward * forwardSpeed + right * lateralSpeed
position += velocity * dt
```

This is simple, stable, and easy to tune.

## Height Handling

Avoid full 3D rigid-body height simulation in v1.

Use one of these:

1. lane/road sampled height
2. heightmap query
3. flat-world assumption per local district

Recommended starting point:

- store height on road segments or map zones
- snap vehicle `Y` from the road it is currently on

This is enough unless the map has extreme vertical complexity.

## Collision Layers

Define strict collision layers early.

Suggested layers:

- player vehicle
- NPC vehicle
- world barrier
- trigger volume
- sidewalk/block zone
- recovery zone

Each layer pair should clearly define:

- collide physically
- overlap only
- ignore

Example:

- player vs world barrier: physical correction
- player vs trigger: overlap
- trigger vs barrier: ignore

## Broadphase

Use a uniform spatial grid for collision and trigger lookup.

Each cell stores:

- dynamic vehicles
- NPC vehicles
- dynamic obstacles
- references to nearby static colliders
- trigger ids

Benefits:

- cheap neighborhood query
- predictable performance
- easy implementation

Recommended cell size:

- around 32m to 64m for physics queries

This is smaller than visibility cells because collision queries are more local.

## Narrowphase Shapes

Use simple body shapes.

### Vehicle Shape

Recommended starting shape:

- circle for very first prototype
- oriented rectangle or capsule for the second step

Best compromise:

- capsule aligned with heading

Reason:

- closer to a car footprint than a circle
- much easier than arbitrary convex polygons

### Static Shape

For static world:

- AABB
- OBB
- segment or capsule walls

Avoid mesh-mesh collision completely.

## Collision Resolution

The design should favor stability over realism.

### Player Vehicle vs Static World

On penetration:

- compute minimum correction vector
- push vehicle out
- remove inward velocity component
- damp speed

If deeply invalid:

- restore `LastValid` transform

### Vehicle vs Vehicle

For the first server prototype, choose one of two models:

1. soft collision
   - vehicles separate
   - speed is damped
   - no realistic impulse exchange

2. overlap with penalty
   - no hard body collision
   - only reduce speed or flag contact

Recommended first choice:

- soft collision for player vs NPC
- optional ignore or soft-only between player vehicles

Reason:

- player-player rigid collision is expensive and causes griefing edge cases
- many driving MMOs treat traffic/world as more important than exact PvP body physics

## Invalid State Recovery

You need a recovery policy for:

- stuck in wall
- flipped orientation
- outside road bounds
- in blocked area

Recommended recovery layers:

1. minor correction: push out and keep moving
2. snap to last valid state
3. reset to nearest safe road anchor

`nearest safe road anchor` should come from road graph metadata, not ad hoc search every time.

## Trigger Volumes

Physics world should support overlap queries for:

- mission pickup zones
- race checkpoints
- shop areas
- repair pads
- teleport/reset pads

Each trigger should define:

- shape
- layer mask
- enter/exit callback key

The physics layer should detect overlaps and emit events. It should not contain mission logic itself.

## NPC Traffic Simulation

NPC traffic should follow lane logic, not free-body physics.

Each NPC vehicle should have:

- lane reference
- progress along lane
- desired speed
- current speed
- stop/yield state

Update loop:

1. sample current lane tangent
2. steer heading toward lane tangent
3. accelerate toward desired speed
4. slow for obstacle ahead
5. move forward along lane
6. switch to next lane at junction

### Obstacle Handling

Use very simple logic:

- cast forward distance
- if blocked, brake
- if lane free, accelerate

Avoid advanced local avoidance in the first version.

## Tick Order

Recommended order inside each fixed step:

1. apply queued control input
2. update vehicle surface context
3. simulate player vehicles
4. simulate NPC vehicles
5. rebuild moved entities in spatial grid
6. resolve dynamic-static collisions
7. resolve dynamic-dynamic interactions
8. evaluate trigger overlaps
9. mark valid transforms and recovery anchors

Why this order:

- motion happens before collision correction
- triggers see corrected final positions
- recovery state updates only after a stable result

## Determinism and Numeric Stability

Full cross-platform deterministic physics is hard.

For the first version, target `practical determinism`:

- fixed timestep
- stable update order
- avoid random iteration over maps
- keep formulas simple
- clamp extreme values

Practical rules:

- iterate entities in sorted or stable slice order
- avoid NaN propagation by sanitizing input
- cap maximum delta movement per tick

## Map Tooling Requirements

To make this physics world usable, the content pipeline should eventually export:

- road segments
- lane connections
- static blocker colliders
- trigger definitions
- spawn points
- recovery anchors
- surface zones

Do not build the runtime around raw art assets. Build it around exported gameplay data.

## Recommended Go Package Layout

```text
/internal/physics
  world.go
  step.go
  config.go
  types.go

/internal/physics/collision
  broadphase.go
  narrowphase.go
  resolve.go

/internal/physics/vehicle
  integrate.go
  control.go
  params.go
  validate.go

/internal/physics/road
  graph.go
  lane.go
  query.go

/internal/physics/trigger
  index.go
  overlap.go

/internal/physics/npc
  traffic.go
  steering.go

/pkg/geom
  vec2.go
  aabb.go
  obb.go
  capsule.go
```

## Suggested Core Interfaces

```go
type RoadQuery interface {
    FindNearestSegment(pos PlanarVec) (SegmentRef, bool)
    SampleHeight(pos PlanarVec) (float32, bool)
    FindRecoveryAnchor(pos PlanarVec) (Transform, bool)
}
```

```go
type TriggerSink interface {
    OnEnter(entityID EntityID, triggerID TriggerID)
    OnExit(entityID EntityID, triggerID TriggerID)
}
```

```go
type SpatialQuery interface {
    NearbyDynamic(pos PlanarVec, radius float32) []EntityID
    NearbyStatic(pos PlanarVec, radius float32) []StaticColliderID
    NearbyTriggers(pos PlanarVec, radius float32) []TriggerID
}
```

## What Not To Build Yet

Avoid these in the first version:

- full rigid-body solver
- wheel colliders
- suspension simulation
- per-triangle map collision
- detailed crash deformation
- fully realistic drifting
- arbitrary dynamic props everywhere

These systems are expensive and do not help you get a working MMO driving core quickly.

## MVP Acceptance Criteria

The physics world is good enough for the first integration when:

1. a player vehicle can drive around the city stably
2. turning behavior is tunable per vehicle class
3. vehicles stay inside drivable space or recover cleanly
4. trigger volumes fire reliably
5. NPC traffic follows lanes without obvious chaos
6. simulation cost remains predictable as entity count grows

## Recommended Build Order

1. geometry primitives
2. fixed-step `PhysicsWorld`
3. road graph and surface query
4. simple vehicle integrator
5. spatial grid
6. static collision resolution
7. trigger overlap system
8. NPC lane-following
9. recovery anchors and invalid-state correction

This sequence gets a drivable world running quickly while keeping the architecture clean.
