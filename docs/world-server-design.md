# World Server Design

## Scope

This document defines the first server process to build:

- one executable
- one shard
- one zone
- authoritative world simulation
- player sessions connected directly to the world server
- enough structure to grow into gateway + multi-zone later

The goal is not a full live-service backend yet. The goal is a world server that can run the city simulation and keep multiple players synchronized correctly.

## First Version Boundary

Put these features inside the first world server:

- accept player connections
- authenticate with a temporary token or stub account id
- spawn one player vehicle into the world
- receive driving input
- simulate movement authoritatively
- replicate nearby players
- run basic NPC traffic
- run trigger volumes such as mission pickup/dropoff or shop area
- save/load player snapshot

Do not put these into the first version:

- matchmaking service
- guild/clan
- ranked race backend
- cash shop
- microservice split
- full map streaming

## Process Shape

For the first milestone, use one process:

```text
cmd/world
  |- network accept loop
  |- session manager
  |- world loop
  |- persistence workers
  |- telemetry
```

This is intentionally a modular monolith. Keep clean interfaces, but do not split into multiple deployables too early.

## Ownership Model

Use a single-writer model for simulation state:

- one goroutine owns the world state
- all gameplay mutations happen in that goroutine
- network goroutines and persistence workers communicate with the world loop through channels/queues

This avoids locks in the hot path and makes bugs easier to reason about.

### Rule

If code changes:

- player position
- vehicle state
- zone membership
- visible entity lists
- mission trigger state

then it should run inside the world loop goroutine.

## Runtime Components

### Network Frontend

Responsibilities:

- accept TCP or WebSocket clients
- decode packets
- validate packet size and message type
- push client commands into the world input queue
- send outbound snapshots/events to clients

Recommended first choice:

- TCP with length-prefixed binary packets

Reason:

- easier to debug
- stable enough for prototype
- simpler reconnect story

### Session Manager

Responsibilities:

- track connected sessions
- map session id to player id
- heartbeat timeout
- disconnect cleanup
- reconnect handoff

For v1 this can live in the same process.

### World Loop

Responsibilities:

- apply player input
- advance vehicles
- update spatial index
- evaluate triggers
- advance NPC traffic
- build replication snapshots
- emit persistence events

This is the core server.

### Persistence Worker

Responsibilities:

- async save of player snapshot
- load character/garage on login
- append economy or mission events later

Important:

- the world loop must never block on disk or DB

## Tick Model

Use fixed tick simulation.

Recommended starting values:

- simulation tick: 15 Hz
- outbound snapshot rate: 10 Hz
- heartbeat timeout: 10 seconds
- persistence snapshot: every 60 seconds

### Tick Order

Each tick should run in this order:

1. drain inbound commands up to a budget
2. apply connection/disconnection events
3. update player input state
4. simulate vehicle movement
5. update NPC traffic
6. update spatial cells
7. evaluate trigger volumes
8. build changed entity sets
9. queue outbound messages
10. emit persistence and telemetry events

Reason for this order:

- input is applied before movement
- spatial index is refreshed after movement
- visibility is built from the latest positions

## Data Flow

### Inbound

```text
Client Packet
  -> network reader
  -> decode
  -> validate
  -> world input queue
  -> world loop
```

### Outbound

```text
World loop
  -> snapshot/event builder
  -> per-session outbound queue
  -> network writer
  -> client
```

### Persistence

```text
World loop
  -> save request queue
  -> persistence worker
  -> database or local file
```

## In-Memory State

The first version should keep a compact world state.

### WorldState

```go
type WorldState struct {
    Tick          uint64
    Now           time.Time
    Config        WorldConfig
    Players       map[PlayerID]*Player
    Sessions      map[SessionID]*Session
    Vehicles      map[EntityID]*Vehicle
    NPCVehicles   map[EntityID]*NPCVehicle
    Triggers      map[TriggerID]*Trigger
    Spatial       *GridIndex
    Outbox        *Outbox
}
```

### Player

```go
type Player struct {
    ID             PlayerID
    SessionID      SessionID
    CharacterID    CharacterID
    Name           string
    VehicleID      EntityID
    Input          DriveInput
    LastAckTick    uint64
    VisibleSet     map[EntityID]struct{}
    SaveDirty      bool
    LastSaveTick   uint64
}
```

### Vehicle

```go
type Vehicle struct {
    ID             EntityID
    OwnerPlayerID  PlayerID
    Transform      Transform
    Speed          float32
    Heading        float32
    Stats          VehicleStats
    Flags          VehicleFlags
    LastValidState Transform
}
```

### Session

```go
type Session struct {
    ID              SessionID
    PlayerID        PlayerID
    Conn            ConnHandle
    LastHeartbeatAt time.Time
    RemoteAddr      string
}
```

### Trigger

```go
type Trigger struct {
    ID       TriggerID
    Kind     TriggerKind
    Bounds   AABB
    Metadata map[string]string
}
```

## Spatial Partition

Use a simple fixed-size grid first.

Recommended:

- square cells
- size around 96 to 128 meters
- map entity id to cell id
- map cell id to entity set

Why grid first:

- easy to implement
- constant-time neighborhood lookup
- enough for city driving gameplay

Only consider quadtree later if profiling shows the grid is insufficient.

### Visibility Rule

For each player, replicate:

- same cell
- adjacent cells
- party/race linked entities even if farther

Distance tiers:

- near: 10 Hz
- medium: 5 Hz
- far: 2 Hz or cull

The first version can simplify this to one fixed frequency per visible entity.

## Vehicle Simulation

The server should simulate gameplay-valid movement, not full client physics.

### Input Model

```go
type DriveInput struct {
    Seq       uint32
    Throttle  float32
    Brake     float32
    Steering  float32
    Handbrake bool
    Nitro     bool
}
```

### Simulation Model

Per tick:

- compute target acceleration from throttle/brake
- apply drag
- clamp to vehicle max speed
- adjust heading from steering and speed
- move position forward
- validate against world limits and simple road rules

### Validation Rules

- reject impossible acceleration spikes
- reject extreme heading snaps
- reject teleport distance greater than allowed
- clamp off-road speed if needed
- snap back to last valid state when state is invalid

For v1, do not attempt authoritative wheel suspension or drift simulation.

## NPC Traffic Design

NPC traffic should be path-following, not expensive physics.

Each NPC vehicle needs:

- current lane segment
- progress on segment
- target speed
- simple braking rule
- next segment choice

Update model:

- active only near players
- sleep when no player is nearby
- respawn from lane spawners when density is low

This gives the map life without heavy CPU cost.

## Trigger System

A trigger is any world region that causes gameplay logic when entered.

Examples:

- mission pickup
- mission dropoff
- shop entrance
- race start pad
- repair zone

### Trigger Evaluation

Per tick:

- query player vehicle cell
- check overlapping triggers in relevant cells
- emit enter/exit events
- run trigger handler

Keep the trigger engine generic. Game rules can be handlers on top.

## Message Types

For the first world server, keep protocol small.

### Client -> Server

- `LoginRequest`
- `Heartbeat`
- `DriveInput`
- `InteractRequest`
- `ChatLocal`

### Server -> Client

- `LoginAccepted`
- `SpawnPlayer`
- `WorldSnapshot`
- `EntityEnter`
- `EntityLeave`
- `MissionEvent`
- `ErrorMessage`

### Suggested Snapshot Shape

```go
type EntityState struct {
    ID      EntityID
    Kind    EntityKind
    PosX    float32
    PosY    float32
    PosZ    float32
    Heading float32
    Speed   float32
    Flags   uint32
}
```

Do not over-design deltas in the first implementation. A compact snapshot is acceptable at low player counts.

## Reconnect and Disconnect

### Disconnect

When a client disconnects:

- mark session closed
- keep player entity for a short grace period, for example 15-30 seconds
- save player snapshot
- remove from world if reconnect does not happen

### Reconnect

If the same player reconnects within the grace window:

- rebind session to player
- resend current spawn/world state
- continue with existing entity if still present

This is important for unstable connections.

## Persistence Model

### First Storage Choice

Use file-backed JSON or SQLite for the very first prototype if speed of iteration matters more than scale.

If you want a more production-shaped start:

- PostgreSQL for durable state

### What to Save

Player snapshot:

- player id
- vehicle id and type
- position
- heading
- speed
- money and progress summary
- current zone

### When to Save

- successful login load
- disconnect
- periodic checkpoint
- mission completion

Save requests should be idempotent and versioned if possible.

## Concurrency Design

Keep concurrency strict.

### Safe goroutines

- one world loop goroutine
- network reader goroutine per connection
- network writer goroutine per connection
- one or more persistence workers

### Shared state rule

Only the world loop goroutine can mutate world state maps.

Other goroutines may:

- read immutable config
- enqueue commands
- dequeue outbound packets

Do not sprinkle mutexes around gameplay state. That usually means the ownership model is already drifting.

## Failure Handling

### Packet abuse

- size limits
- rate limits
- invalid packet counter
- disconnect on malformed spam

### Slow clients

- bounded outbound queue
- drop low-priority updates first
- disconnect if client cannot keep up

### Persistence failure

- log failure
- retry asynchronously
- do not stall the world tick

## Observability

Add these metrics from day one:

- current players
- tick duration
- max tick duration
- inbound queue depth
- outbound queue depth
- visible entities per player
- save queue depth
- disconnect reason counts

Add structured logs for:

- login
- disconnect
- teleport correction
- trigger enter/exit
- persistence failure

## Proposed Go Package Layout

```text
/cmd/world
  main.go

/internal/world
  server.go
  loop.go
  state.go
  config.go

/internal/session
  manager.go
  types.go

/internal/netcode
  listener.go
  reader.go
  writer.go
  codec.go

/internal/vehicle
  simulate.go
  validate.go
  stats.go

/internal/spatial
  grid.go
  cell.go

/internal/npc
  traffic.go
  lane_graph.go

/internal/trigger
  engine.go
  handlers.go

/internal/persistence
  store.go
  worker.go
  models.go

/pkg/protocol
  messages.go
  ids.go
```

## Suggested Core Interfaces

Keep interfaces narrow.

```go
type Store interface {
    LoadPlayer(ctx context.Context, playerID PlayerID) (*PlayerSnapshot, error)
    SavePlayer(ctx context.Context, snapshot PlayerSnapshot) error
}
```

```go
type Codec interface {
    Decode([]byte) (protocol.Message, error)
    Encode(protocol.Message) ([]byte, error)
}
```

```go
type OutboundSink interface {
    Send(sessionID SessionID, msg protocol.Message) error
}
```

## Startup Sequence

1. load config
2. initialize store
3. initialize world state
4. start persistence worker
5. start network listener
6. start world loop ticker
7. accept connections

Keep startup deterministic and fail fast if config or storage is invalid.

## MVP Acceptance Criteria

The first world server is good enough when it can do all of the following:

1. 20-50 players can connect to one zone
2. players can drive simultaneously without major desync
3. nearby players appear and disappear correctly
4. disconnect/reconnect preserves player state
5. tick time stays below budget most of the time
6. no DB or disk operation blocks the simulation loop

## Recommended Next Step After This Design

Implement in this order:

1. `pkg/protocol`
2. `internal/world/state.go`
3. `internal/spatial/grid.go`
4. `internal/vehicle/simulate.go`
5. `internal/netcode` with stub login
6. `cmd/world/main.go`
7. persistence worker

This ordering gives you a running world quickly, then adds safety and durability around it.
