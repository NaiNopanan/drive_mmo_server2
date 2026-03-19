package main

import (
    "fmt"
    "sort"
    "time"

    "server2/internal/scenario"
)

type result struct {
    name string
    avg  float64
    p95  float64
    max  float64
    ticks int
}

func ms(d time.Duration) float64 {
    return float64(d.Microseconds()) / 1000.0
}

func measure(def scenario.ScenarioDefinition) result {
    r := scenario.NewScenarioRunner(def)
    durations := make([]float64, 0, def.MaxTicks)
    for !r.Finished {
        r.Step()
        if r.Tick > 0 {
            durations = append(durations, ms(r.LastStepDuration))
        }
    }

    sum := 0.0
    maxv := 0.0
    sorted := append([]float64(nil), durations...)
    sort.Float64s(sorted)
    for _, v := range durations {
        sum += v
        if v > maxv {
            maxv = v
        }
    }
    avg := 0.0
    if len(durations) > 0 {
        avg = sum / float64(len(durations))
    }
    p95 := 0.0
    if len(sorted) > 0 {
        idx := (95*len(sorted) + 99) / 100
        if idx <= 0 {
            idx = 1
        }
        if idx > len(sorted) {
            idx = len(sorted)
        }
        p95 = sorted[idx-1]
    }

    return result{name: def.Name, avg: avg, p95: p95, max: maxv, ticks: len(durations)}
}

func main() {
    defs := []scenario.ScenarioDefinition{
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAllCCDScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxHybridCCDOptimizedScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDOptimizedScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckOptimizedScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxAdaptiveCCDPrecheckHysteresisOptimizedScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxPersistentPairsIslandsWarmStartOptimizedScenario(),
        scenario.NewHundredRigidSpheresAndHundredRigidBoxesInBoxSolverManifoldWarmStartOptimizedScenario(),
    }

    for _, def := range defs {
        res := measure(def)
        fmt.Printf("%s\n", res.name)
        fmt.Printf("  ticks=%d avg=%0.3f ms p95=%0.3f ms max=%0.3f ms\n", res.ticks, res.avg, res.p95, res.max)
    }
}
