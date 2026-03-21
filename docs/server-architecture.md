# MMO Driving Server Architecture

## Goal

Design a server-authoritative backend for an online driving game in the style of RayCity:

- many players in the same city map
- smooth vehicle movement in an open world
- missions, delivery, racing, and NPC traffic
- chat, parties, matchmaking, and progression
- horizontal growth from small prototype to live service

This document is written for the current Go project (`server2`) as a practical starting point.

## Core Principles

1. Server authoritative
   - The client sends input and intent, not final position or rewards.
   - The server decides movement validity, collisions that matter for gameplay, mission results, rewards, and economy changes.

2. Cheap simulation, expensive validation only where needed
   - MMO driving does not need full physics parity with the client for every frame.
   - Use simplified server physics for position, speed, heading, road adherence, and collision zones.
   - Reserve detailed checks for races, mission triggers, anti-cheat, and important interactions.

3. Area-based interest management
   - Players only receive nearby entities.
   - The city is divided into sectors/cells so bandwidth scales with local density instead of total online players.

4. Single-writer per world shard
   - One simulation owner updates one shard/zone state.
   - This keeps logic deterministic and avoids heavy locking.

5. Async side effects
   - Persistence, analytics, mail, notifications, and audit logs should be event-driven and out of the simulation hot path.

## Suggested High-Level Architecture

```text
Client
  -> Gateway
  -> Session Service
  -> World Coordinator
  -> Zone Simulation Servers
       |- movement
       |- visibility
       |- NPC traffic
       |- mission triggers
       |- local chat
  -> Domain Services
       |- account/profile
       |- inventory/garage
       |- quest/mission
       |- party/friends
       |- matchmaking/race
       |- economy
  -> Data Layer
       |- Redis cache/presence
       |- PostgreSQL/MySQL
       |- object storage for logs/replays
       |- message bus
```

## Runtime Components

### 1. Gateway

Responsibilities:

- terminate TCP/WebSocket/UDP transport
- decode packets
- authenticate session tokens
- rate-limit abusive clients
- forward messages to the correct shard/service

Recommendation:

- Start with TCP or WebSocket for simplicity.
- Add UDP later only for high-frequency movement if profiling proves it necessary.
- For a first playable build, TCP is good enough if packets are compact and updates are interest-filtered.

### 2. Session Service

Responsibilities:

- login and reconnect
- account-to-character binding
- duplicate login handling
- session heartbeat
- presence state

Keep this stateless where possible, with session data cached in Redis.

### 3. World Coordinator

Responsibilities:

- assign player to channel/shard
- route zone transfer
- keep registry of active zone servers
- enforce population caps

Think of this as air traffic control, not the place where game logic lives.

### 4. Zone Simulation Server

This is the most important process.

Responsibilities:

- authoritative player movement
- nearby entity replication
- spatial partition updates
- NPC traffic and ambient objects
- trigger volumes for shops, missions, races, teleporters
- short-lived local events

Suggested loop:

- tick at 10-20 Hz for simulation
- send state snapshots/deltas at 5-15 Hz depending on distance and importance
- process inbound input commands with sequence numbers

For a driving MMO, 15 Hz server simulation is a reasonable starting point. Client-side interpolation hides most of the lower tick rate.

### 5. Domain Services

Keep heavy business logic out of zone servers unless it must run in real time.

Services to split cleanly:

- `profile`: level, exp, licenses, money
- `garage`: owned cars, upgrades, fuel/durability if used
- `inventory`: consumables, parts, rewards
- `mission`: deliveries, chase tasks, time trials
- `race`: lobbies, countdown, finish order, reward settlement
- `social`: friends, party, guild/clan, chat channels
- `economy`: purchases, sinks, anti-duplication checks

## World Model

### Shards and Zones

Use this hierarchy:

- `Region`: deployment region, like SEA or NA
- `Shard`: one world copy/channel for concurrency and social grouping
- `Zone`: a simulation unit within a shard, usually a city district or map partition
- `Cell`: spatial index used inside a zone for interest management

Example:

```text
Region: SEA
  Shard: Alpha-1
    Zone: Downtown
      Cells: 128m x 128m grid
    Zone: Harbor
    Zone: Highway-East
```

Why this works:

- `Shard` limits how many players share one economy/chat space.
- `Zone` keeps each simulation process small enough to reason about.
- `Cell` makes nearby-player lookup cheap.

### Interest Management

Each entity belongs to one cell. Each player subscribes to:

- current cell
- adjacent cells
- special tracked entities like party members or race participants

Update policy:

- close/high-speed entities: frequent updates
- far/slow/static entities: lower frequency
- remove entities when outside subscription radius

This matters more than raw network protocol choice.

## Network Model

### Client to Server

Send intent/input, for example:

- throttle
- brake
- steering
- handbrake
- boost/nitro use
- interaction request
- mission accept/complete request

Include:

- client timestamp
- input sequence
- last acknowledged server snapshot

### Server to Client

Send:

- authoritative player state
- nearby entity snapshots/deltas
- mission state changes
- inventory/economy updates
- chat/social events

Movement packet shape can be simple:

```text
entity_id
pos_x, pos_y, pos_z
heading
speed
state_flags
server_tick
```

Do not replicate raw physics internals unless clients truly need them.

## Vehicle and Movement Simulation

The server should not run full visual physics. It should run validation-oriented movement.

Recommended state per vehicle:

- transform
- velocity scalar/vector
- heading
- current road segment or nav area
- vehicle class/stats
- movement flags
- last valid state

Recommended checks:

- max acceleration/deceleration envelope
- max steering change over time
- impossible teleport detection
- road/off-road speed rules
- checkpoint order for races
- trigger overlap for shops/missions

This is enough to prevent most obvious cheats while keeping CPU cost under control.

## NPC Traffic

Do not simulate all city traffic globally at full fidelity.

Use layered NPC simulation:

1. Ambient low-cost traffic in active zones only
2. Higher-fidelity traffic near players
3. Despawn or sleep traffic in empty areas

Represent NPC traffic with path-following on road graphs:

- lane graph
- speed profile
- stop rules
- simple avoidance

Keep collisions between player and traffic gameplay-driven, not physically perfect.

## Missions and Activities

Split content into two categories.

### Zone-local activities

Good fit inside the zone server:

- pickup/dropoff triggers
- courier routes
- checkpoint time trials
- patrol/chase events
- instant shop interactions

### Cross-zone or session-based activities

Better in separate services:

- queued races
- ranked events
- party matchmaking
- global seasonal events

Rule:

- if it needs frame-level location authority, keep it near the zone simulation
- if it mostly coordinates players and rewards, move it to a domain service

## Persistence

### Database

Use PostgreSQL or MySQL for durable game state:

- accounts
- characters
- garage
- inventory
- mission progression
- mail
- economy ledger

Use Redis for:

- sessions
- presence
- short-lived caches
- shard registry
- pub/sub or stream fanout if needed

### Save Strategy

Avoid writing character state every tick.

Persist on:

- logout
- zone transfer
- mission completion
- purchase/reward settlement
- periodic snapshot, for example every 30-120 seconds

Keep an append-only economy ledger for money/items that matter. This is critical for investigating exploits.

## Anti-Cheat and Trust Boundaries

Minimum protections for an MVP:

- speed/teleport envelope checks
- sequence validation and packet sanity checks
- server-side reward calculation
- inventory/economy idempotency keys
- cooldown validation on skill/item usage
- race checkpoint validation

Later:

- replay logs for suspicious sessions
- anomaly detection on earnings and route times
- challenge-response for movement outliers

Do not trust:

- client position
- client rewards
- client timers for races or deliveries
- client inventory counts

## Failure Model

Design for process failure early.

### On gateway failure

- clients reconnect and resume session

### On zone failure

- players reconnect to coordinator
- restore last durable checkpoint
- optionally restore from recent in-memory snapshot if available

### On domain service timeout

- zone should degrade gracefully
- queue reward settlement instead of blocking the simulation loop

The simulation loop must never wait on a database query.

## Recommended Go Project Layout

```text
/cmd
  /gateway
  /world
  /zone
  /worker

/internal
  /netcode
  /session
  /world
  /zone
  /spatial
  /vehicle
  /mission
  /race
  /economy
  /social
  /persistence
  /config
  /telemetry

/pkg
  /protocol
  /gameid
  /vec

/docs
  server-architecture.md
```

Guidelines:

- `cmd/*` contains process entrypoints only
- `internal/zone` owns the hot simulation loop
- `pkg/protocol` contains packet structs and serialization contracts shared by tools or clients
- `internal/persistence` should expose repositories and event sinks, not SQL leaking everywhere

## Suggested First Milestone

Build the smallest vertical slice that proves the architecture:

1. login/session
2. one shard
3. one zone
4. one drivable vehicle per player
5. nearby player replication
6. one NPC traffic type
7. one delivery mission
8. save/load character and garage

If this slice works with 50-100 concurrent players in one zone, the architecture is on the right track.

## Scaling Roadmap

### Phase 1: Prototype

- one process for gateway + world + zone
- in-memory state
- periodic DB saves
- simple grid-based interest management

### Phase 2: Playable Alpha

- separate gateway and zone processes
- Redis for sessions and shard registry
- one DB for durable state
- mission/economy split into services or worker queues

### Phase 3: Live Service

- multiple shards/channels
- independent zone processes
- message bus for async events
- observability, replay logs, live ops tooling
- GM/admin tools

Do not start with microservices everywhere. Start with modular monolith boundaries and split by pressure.

## Operational Metrics

Track these from the beginning:

- connected players per shard/zone
- tick duration and tick overruns
- inbound/outbound packets per second
- visible entities per player
- DB write latency
- zone transfer latency
- reconnect success rate
- mission completion failure rate

If tick duration becomes unstable, gameplay quality collapses quickly.

## Notes for This Repo

- The current module uses `raylib-go`, which is better suited for client or debug visualization than the main server runtime.
- For the actual production server, keep the runtime headless and deterministic.
- If visual debugging is useful, make a separate debug viewer process that subscribes to world snapshots instead of coupling rendering into the server.

## Recommended Next Implementation Order

1. define protocol messages for login, input, snapshots, and mission events
2. build a single-process authoritative zone loop
3. add grid-based spatial partitioning
4. add session management and reconnect handling
5. persist character/garage state
6. add one mission type end-to-end
7. split gateway and zone into separate binaries only after the prototype is stable
