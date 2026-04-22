package sim

import (
	"math"
	"math/rand"
	"testing"
)

func TestUpdatePedestriansSpawnsFromDeadEnds(t *testing.T) {
	rand.Seed(1)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(12, 0)},
	}
	nextID := 1
	pedestrians, timers := updatePedestrians(nil, paths, &nextID, nil, pedestrianSpawnIntervalS*(1+pedestrianSpawnJitterFrac)+1, nil, nil, nil)

	if len(pedestrians) != 2 {
		t.Fatalf("expected one pedestrian from each dead end, got %d", len(pedestrians))
	}
	if len(timers) != 2 {
		t.Fatalf("expected two spawn timers, got %d", len(timers))
	}
	for i, ped := range pedestrians {
		if ped.ID != i+1 {
			t.Fatalf("expected pedestrian ID %d, got %d", i+1, ped.ID)
		}
		if ped.PathIndex != 0 {
			t.Fatalf("expected pedestrian on path 0, got path %d", ped.PathIndex)
		}
		if ped.Distance < pedestrianSpawnInsetM {
			t.Fatalf("expected spawned pedestrian to start inside path, got distance %.3f", ped.Distance)
		}
	}
}

func TestUpdatePedestriansBranchAndDespawnAtDeadEnd(t *testing.T) {
	rand.Seed(2)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(2, 0)},
		{P0: NewVec2(2, 0), P1: NewVec2(3, 1)},
		{P0: NewVec2(2, 0), P1: NewVec2(3, -1)},
	}
	pedestrians := []Pedestrian{
		{
			ID:        1,
			PathIndex: 0,
			Distance:  1.85,
			Forward:   true,
			Speed:     1.4,
			BaseSpeed: 1.4,
			Radius:    pedestrianRadiusM,
		},
	}

	pedestrians, timers := updatePedestrians(pedestrians, paths, nil, nil, 0.3, nil, nil, nil)
	if len(pedestrians) != 1 {
		t.Fatalf("expected pedestrian to survive branch, got %d", len(pedestrians))
	}
	if !pedestrians[0].TransitionActive {
		t.Fatal("expected pedestrian to enter a junction transition")
	}
	chosenPath := pedestrians[0].TransitionNextPath
	if chosenPath != 1 && chosenPath != 2 {
		t.Fatalf("expected pedestrian to choose one of the branch edges, got %d", chosenPath)
	}

	for step := 0; step < 5 && len(pedestrians) > 0; step++ {
		pedestrians, timers = updatePedestrians(pedestrians, paths, nil, timers, 0.4, nil, nil, nil)
	}
	if len(pedestrians) != 0 {
		t.Fatalf("expected pedestrian to despawn at branch dead end, still have %d", len(pedestrians))
	}
}

func TestUpdatePedestriansSoftAvoidanceKeepsThemMoving(t *testing.T) {
	rand.Seed(3)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	pedestrians := []Pedestrian{
		{
			ID:            1,
			PathIndex:     0,
			Distance:      9.0,
			Forward:       true,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: 0,
		},
		{
			ID:            2,
			PathIndex:     0,
			Distance:      9.0,
			Forward:       false,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: 0,
		},
	}

	pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.5, nil, nil, nil)
	if len(pedestrians) != 2 {
		t.Fatalf("expected both pedestrians to remain active, got %d", len(pedestrians))
	}
	if pedestrians[0].Distance <= 9.0 || pedestrians[1].Distance <= 9.0 {
		t.Fatalf("expected both pedestrians to keep moving, got distances %.3f and %.3f", pedestrians[0].Distance, pedestrians[1].Distance)
	}
	if pedestrians[0].LateralOffset <= 0.1 {
		t.Fatalf("expected forward pedestrian to shift to one side, got lateral offset %.3f", pedestrians[0].LateralOffset)
	}
	if pedestrians[1].LateralOffset >= -0.1 {
		t.Fatalf("expected reverse pedestrian to shift to the opposite side, got lateral offset %.3f", pedestrians[1].LateralOffset)
	}
	if pedestrians[0].Speed <= 0 || pedestrians[1].Speed <= 0 {
		t.Fatalf("expected soft avoidance to preserve forward motion, got speeds %.3f and %.3f", pedestrians[0].Speed, pedestrians[1].Speed)
	}
}

func TestPedestrianPoseTraversesTurnConnector(t *testing.T) {
	rand.Seed(4)

	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(2, 0)},
		{P0: NewVec2(2, 0), P1: NewVec2(2, 2)},
	}
	pedestrians := []Pedestrian{
		{
			ID:            1,
			PathIndex:     0,
			Distance:      1.95,
			Forward:       true,
			Speed:         1.4,
			BaseSpeed:     1.4,
			Radius:        pedestrianRadiusM,
			LateralOffset: pedestrianPreferredOffsetM,
		},
	}

	foundMidTransition := false
	ped := Pedestrian{}
	for step := 0; step < 12; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.15, nil, nil, nil)
		if len(pedestrians) != 1 {
			t.Fatalf("expected pedestrian to remain active, got %d", len(pedestrians))
		}
		ped = pedestrians[0]
		if ped.TransitionActive && ped.TransitionLength > 0 && ped.TransitionDistance/ped.TransitionLength >= 0.35 {
			foundMidTransition = true
			break
		}
	}
	if !foundMidTransition {
		t.Fatal("expected pedestrian to reach the middle of a turn transition")
	}
	pos, heading, ok := PedestrianPose(paths, ped)
	if !ok {
		t.Fatal("expected turn-connector pedestrian pose")
	}
	if pos.X <= 1.2 || pos.X >= 2.75 || pos.Y <= -0.8 || pos.Y >= 1.8 {
		t.Fatalf("expected connector position near the corner, got (%.3f, %.3f)", pos.X, pos.Y)
	}
	if absf(heading.X) <= 0.2 || absf(heading.Y) <= 0.2 {
		t.Fatalf("expected connector heading between old and new edges, got (%.3f, %.3f)", heading.X, heading.Y)
	}
}

func buildCrossingTestSpline(id int) Spline {
	return NewSpline(id,
		NewVec2(10, -10),
		NewVec2(10, -3),
		NewVec2(10, 3),
		NewVec2(10, 10),
	)
}

func TestComputePedestrianCrossingsFindsIntersection(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	if len(crossings) != 1 {
		t.Fatalf("expected 1 crossing, got %d", len(crossings))
	}
	c := crossings[0]
	if c.SplineID != 1 || c.PathIndex != 0 {
		t.Fatalf("unexpected crossing ids: %+v", c)
	}
	if absf(c.DistOnPath-10) > 0.1 {
		t.Fatalf("expected crossing at path distance ~10, got %.3f", c.DistOnPath)
	}
	if absf(c.DistOnSpline-10) > 0.5 {
		t.Fatalf("expected crossing at spline distance ~10, got %.3f", c.DistOnSpline)
	}
}

func TestApproachingPedestrianBlocksCarsAtCrossing(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})

	approaching := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 8.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	blocked := buildPedestrianBlockedSplineDists(crossings, paths, approaching, nil)
	if len(blocked[1]) == 0 {
		t.Fatal("expected spline 1 to be blocked by pedestrian near the crossing")
	}

	far := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 1.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	if got := buildPedestrianBlockedSplineDists(crossings, paths, far, nil); len(got[1]) != 0 {
		t.Fatalf("expected no block when pedestrian is well before the crossing, got %v", got)
	}
}

func TestCarWithRoomBrakesForBlockedCrossing(t *testing.T) {
	blocked := map[int][]float32{1: {25}}
	spline := NewSpline(1,
		NewVec2(0, 0),
		NewVec2(10, 0),
		NewVec2(20, 0),
		NewVec2(30, 0),
	)
	graph := NewRoadGraph([]Spline{spline}, nil)
	car := Car{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 0, Accel: 3.0, Speed: 5.0, MaxSpeed: 20.0}
	storedSpline, _ := graph.splinePtrByID(1)
	cap := computePedestrianCrossingSpeedCap(car, storedSpline, graph, blocked)
	if cap >= float32(math.MaxFloat32) {
		t.Fatal("expected a speed cap when approaching a blocked crossing with plenty of room")
	}

	close := Car{ID: 2, CurrentSplineID: 1, DistanceOnSpline: 24, Accel: 3.0, Speed: 12.0, MaxSpeed: 20.0}
	capClose := computePedestrianCrossingSpeedCap(close, storedSpline, graph, blocked)
	if capClose != 0 {
		t.Fatalf("expected a zero cap (cannot stop in time) when within the stop buffer, got %.3f", capClose)
	}
}

func TestPedestrianWaitsForCarOccupyingCrossing(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	cars := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 10, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 7.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 30; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, cars, nil)
		if len(pedestrians) != 1 {
			t.Fatalf("expected pedestrian to remain active, got %d", len(pedestrians))
		}
	}
	stopAt := float32(10.0) - pedestrianCrossingStopBufferM
	if pedestrians[0].Distance > stopAt+0.1 {
		t.Fatalf("expected pedestrian to stop before crossing (<= %.3f), got %.3f", stopAt, pedestrians[0].Distance)
	}
	if pedestrians[0].Distance < stopAt-0.8 {
		t.Fatalf("expected pedestrian to reach the stop line (~%.3f), got %.3f", stopAt, pedestrians[0].Distance)
	}
	if pedestrians[0].Speed > 0.05 {
		t.Fatalf("expected pedestrian speed to settle near zero while waiting, got %.3f", pedestrians[0].Speed)
	}
}

func TestPedestrianResumesAfterCarClears(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	cars := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 10, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 7.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 15; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, cars, nil)
	}
	stoppedAt := pedestrians[0].Distance
	// Car drives away; no longer occupies the crossing.
	carsClear := []Car{
		{ID: 1, CurrentSplineID: 1, DistanceOnSpline: 30, Length: 4.5, Width: 1.8, Speed: 5.0},
	}
	for step := 0; step < 10; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, crossings, carsClear, nil)
	}
	if pedestrians[0].Distance <= stoppedAt+0.5 {
		t.Fatalf("expected pedestrian to resume moving after car clears, still at %.3f (was %.3f)", pedestrians[0].Distance, stoppedAt)
	}
}

func redPedestrianLightCycle(cycleID, lightID int) []TrafficCycle {
	// Two phases: phase 0 (current) keeps the light NOT green → red. Phase 1
	// would turn it green but we don't tick time in these tests.
	return []TrafficCycle{
		{
			ID:         cycleID,
			LightIDs:   []int{lightID},
			Phases:     []TrafficPhase{{DurationSecs: 5}, {DurationSecs: 5, GreenLightIDs: []int{lightID}}},
			Timer:      0,
			PhaseIndex: 0,
			Enabled:    true,
		},
	}
}

func greenPedestrianLightCycle(cycleID, lightID int) []TrafficCycle {
	return []TrafficCycle{
		{
			ID:         cycleID,
			LightIDs:   []int{lightID},
			Phases:     []TrafficPhase{{DurationSecs: 5, GreenLightIDs: []int{lightID}}, {DurationSecs: 5}},
			Timer:      0,
			PhaseIndex: 0,
			Enabled:    true,
		},
	}
}

func TestRedPedestrianLightStopsMatchingDirection(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 10, PedestrianForward: true, WorldPos: NewVec2(10, 3)},
	}
	cycles := redPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	if len(stopping[0]) != 1 {
		t.Fatalf("expected pedestrian light to be stopping, got %d", len(stopping[0]))
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 6.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 30; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, nil, nil, stopping)
	}
	stopLine := float32(10.0) - pedestrianCrossingStopBufferM
	if pedestrians[0].Distance > stopLine+0.1 {
		t.Fatalf("expected pedestrian to wait before red light (<= %.3f), got %.3f", stopLine, pedestrians[0].Distance)
	}
	if pedestrians[0].Distance < stopLine-1.0 {
		t.Fatalf("expected pedestrian close to the stop line (~%.3f), got %.3f", stopLine, pedestrians[0].Distance)
	}
}

func TestRedPedestrianLightIgnoresOppositeDirection(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	// Light stops only forward pedestrians; the one we're testing is backward.
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 10, PedestrianForward: true, WorldPos: NewVec2(10, 3)},
	}
	cycles := redPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 14.0, Forward: false, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	start := pedestrians[0].Distance
	for step := 0; step < 12; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, nil, nil, stopping)
	}
	// Backward pedestrians track progress in ped.Distance measured from P1,
	// so a pedestrian walking unimpeded has Distance increasing over time.
	if pedestrians[0].Distance <= start+1.0 {
		t.Fatalf("expected backward pedestrian to pass unaffected, moved from %.3f to %.3f", start, pedestrians[0].Distance)
	}
}

func TestRedPedestrianLightStopsBackwardDirection(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	// Light affects pedestrians walking P1→P0 (Forward=false).
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 10, PedestrianForward: false, WorldPos: NewVec2(10, -3)},
	}
	cycles := redPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	// Backward ped starts at Distance=6 (path-x=14), walking toward P0 (x=0).
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 6.0, Forward: false, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: -pedestrianPreferredOffsetM},
	}
	for step := 0; step < 30; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, nil, nil, stopping)
	}
	// Should stop at path-x = DistOnPath + buffer = 10 + 2 = 12, i.e., Distance = 20 - 12 = 8.
	expectedDistance := float32(20.0) - (10 + pedestrianCrossingStopBufferM)
	if pedestrians[0].Distance > expectedDistance+1.0 {
		t.Fatalf("expected backward pedestrian to stop at Distance≈%.3f, got %.3f", expectedDistance, pedestrians[0].Distance)
	}
	if pedestrians[0].Distance < expectedDistance-0.1 {
		t.Fatalf("expected backward pedestrian to reach the stop line (≈%.3f), got %.3f", expectedDistance, pedestrians[0].Distance)
	}
}

func TestGreenPedestrianLightAllowsPassage(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 10, PedestrianForward: true, WorldPos: NewVec2(10, 3)},
	}
	cycles := greenPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	if len(stopping) != 0 {
		t.Fatalf("expected green light to produce no stopping entries, got %v", stopping)
	}
	pedestrians := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 6.0, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 12; step++ {
		pedestrians, _ = updatePedestrians(pedestrians, paths, nil, nil, 0.25, nil, nil, stopping)
	}
	if pedestrians[0].Distance <= 10 {
		t.Fatalf("expected pedestrian to cross a green light, got distance %.3f", pedestrians[0].Distance)
	}
}

func TestPedestrianWaitingAtRedDoesNotBlockCrossingAhead(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	// Light stops forward pedestrians at DistOnPath=8; crossing sits at 10.
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 8, PedestrianForward: true, WorldPos: NewVec2(8, 3)},
	}
	cycles := redPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	// Simulate the crossing via a spline at x=10 (matches earlier helper).
	spline := buildCrossingTestSpline(1)
	crossings := computePedestrianCrossings(paths, []Spline{spline})
	// Pedestrian has been pushed back to the red-light stop line at ~6.
	peds := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 6.0, Forward: true, Speed: 0, BaseSpeed: 1.4, Radius: pedestrianRadiusM},
	}
	blocked := buildPedestrianBlockedSplineDists(crossings, paths, peds, stopping)
	if len(blocked[1]) != 0 {
		t.Fatalf("pedestrian waiting at a red light should not block the crossing past it, got %v", blocked)
	}
	// With no red light, the same pedestrian would block (sanity check).
	noLights := map[int][]TrafficLight{}
	blocked2 := buildPedestrianBlockedSplineDists(crossings, paths, peds, noLights)
	if len(blocked2[1]) == 0 {
		t.Fatal("sanity check: without the light, the pedestrian should still block the crossing")
	}
}

func TestPedestrianQueueSpacingAtStopLine(t *testing.T) {
	paths := []PedestrianPath{
		{P0: NewVec2(0, 0), P1: NewVec2(20, 0)},
	}
	lights := []TrafficLight{
		{ID: 100, SplineID: 0, CycleID: 1, PedestrianPathIndex: 0, DistOnPath: 10, PedestrianForward: true, WorldPos: NewVec2(10, 3)},
	}
	cycles := redPedestrianLightCycle(1, 100)
	stopping := buildStoppingPedestrianLightsByPath(lights, cycles)
	// Three forward peds walking together toward the light.
	peds := []Pedestrian{
		{ID: 1, PathIndex: 0, Distance: 5.5, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
		{ID: 2, PathIndex: 0, Distance: 4.5, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
		{ID: 3, PathIndex: 0, Distance: 3.5, Forward: true, Speed: 1.4, BaseSpeed: 1.4, Radius: pedestrianRadiusM, LateralOffset: pedestrianPreferredOffsetM},
	}
	for step := 0; step < 40; step++ {
		peds, _ = updatePedestrians(peds, paths, nil, nil, 0.25, nil, nil, stopping)
	}
	if len(peds) != 3 {
		t.Fatalf("expected all 3 peds to survive, got %d", len(peds))
	}
	// Leader should stop at the usual stop line (≈ 10 - buffer = 8).
	stopLine := float32(10.0) - pedestrianCrossingStopBufferM
	// Sort copy by Distance descending for predictability.
	lead, mid, tail := peds[0], peds[1], peds[2]
	if lead.Distance > stopLine+0.1 || lead.Distance < stopLine-0.5 {
		t.Fatalf("leader expected near %.2f, got %.3f", stopLine, lead.Distance)
	}
	// Each subsequent ped should be at least one spacing unit behind the
	// previous, with some slack for the exponential approach.
	minGap := float32(0.7)
	if lead.Distance-mid.Distance < minGap {
		t.Fatalf("expected gap between leader (%.3f) and middle (%.3f) >= %.2f", lead.Distance, mid.Distance, minGap)
	}
	if mid.Distance-tail.Distance < minGap {
		t.Fatalf("expected gap between middle (%.3f) and tail (%.3f) >= %.2f", mid.Distance, tail.Distance, minGap)
	}
}

func TestSplineLightNotConfusedWithPedestrianLight(t *testing.T) {
	lights := []TrafficLight{
		{ID: 1, SplineID: 5, DistOnSpline: 10, CycleID: 7},
		{ID: 2, SplineID: 0, CycleID: 7, PedestrianPathIndex: 3, DistOnPath: 4, PedestrianForward: true},
	}
	cycles := redPedestrianLightCycle(7, 0)
	// Adapt cycle to cover both lights being red.
	cycles[0].LightIDs = []int{1, 2}
	cycles[0].Phases = []TrafficPhase{{DurationSecs: 5}, {DurationSecs: 5, GreenLightIDs: []int{1, 2}}}

	bySpline := buildStoppingTrafficLightsBySpline(lights, cycles)
	if len(bySpline[5]) != 1 {
		t.Fatalf("expected spline 5 to have 1 stopping light, got %d", len(bySpline[5]))
	}
	if len(bySpline[0]) != 0 {
		t.Fatalf("did not expect pedestrian light to leak into spline map, got %d", len(bySpline[0]))
	}

	byPath := buildStoppingPedestrianLightsByPath(lights, cycles)
	if len(byPath[3]) != 1 {
		t.Fatalf("expected path 3 to have 1 stopping light, got %d", len(byPath[3]))
	}
	if len(byPath[5]) != 0 {
		t.Fatalf("did not expect spline light to leak into path map")
	}
}
