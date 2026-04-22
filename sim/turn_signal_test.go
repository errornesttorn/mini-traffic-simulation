package sim

import "testing"

func turnSignalStraightSpline(id int, x0, y0, x3, y3 float32) Spline {
	return NewSpline(id,
		NewVec2(x0, y0),
		NewVec2(x0+(x3-x0)*0.33, y0+(y3-y0)*0.33),
		NewVec2(x0+(x3-x0)*0.66, y0+(y3-y0)*0.66),
		NewVec2(x3, y3),
	)
}

func turnSignalGraph(splines []Spline) *RoadGraph {
	return NewRoadGraph(splines, nil)
}

// Car's right-hand side (per the simulator's rightNormal convention) for an
// east-heading spline is the −Y direction, so a sibling lane at lower Y is a
// right-turn, at higher Y is a left-turn. The tests pin down both directions
// plus the neutral "no desired lane" case.

func TestTurnSignalOffWithoutLaneChange(t *testing.T) {
	splines := []Spline{turnSignalStraightSpline(1, 0, 0, 30, 0)}
	graph := turnSignalGraph(splines)
	car := Car{CurrentSplineID: 1, DistanceOnSpline: 5, DesiredLaneSplineID: -1}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalNone {
		t.Fatalf("expected TurnSignalNone, got %d", got)
	}
}

func TestTurnSignalDesiredLaneRight(t *testing.T) {
	splines := []Spline{
		turnSignalStraightSpline(1, 0, 0, 30, 0),
		turnSignalStraightSpline(2, 0, -4, 30, -4),
	}
	graph := turnSignalGraph(splines)
	car := Car{
		CurrentSplineID:     1,
		DistanceOnSpline:    5,
		DesiredLaneSplineID: 2,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalRight {
		t.Fatalf("expected TurnSignalRight, got %d", got)
	}
}

func TestTurnSignalDesiredLaneLeft(t *testing.T) {
	splines := []Spline{
		turnSignalStraightSpline(1, 0, 0, 30, 0),
		turnSignalStraightSpline(2, 0, 4, 30, 4),
	}
	graph := turnSignalGraph(splines)
	car := Car{
		CurrentSplineID:     1,
		DistanceOnSpline:    5,
		DesiredLaneSplineID: 2,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalLeft {
		t.Fatalf("expected TurnSignalLeft, got %d", got)
	}
}

func TestTurnSignalActiveLaneChangeRight(t *testing.T) {
	bridge := NewSpline(10,
		NewVec2(0, 0),
		NewVec2(10, 0),
		NewVec2(20, -4),
		NewVec2(30, -4),
	)
	splines := []Spline{bridge}
	graph := turnSignalGraph(splines)
	car := Car{
		LaneChanging:       true,
		CurrentSplineID:    10,
		LaneChangeSplineID: 10,
		AfterSplineID:      2,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalRight {
		t.Fatalf("expected TurnSignalRight during rightward bridge, got %d", got)
	}
}

func TestTurnSignalActiveLaneChangeLeft(t *testing.T) {
	bridge := NewSpline(10,
		NewVec2(0, 0),
		NewVec2(10, 0),
		NewVec2(20, 4),
		NewVec2(30, 4),
	)
	splines := []Spline{bridge}
	graph := turnSignalGraph(splines)
	car := Car{
		LaneChanging:       true,
		CurrentSplineID:    10,
		LaneChangeSplineID: 10,
		AfterSplineID:      2,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalLeft {
		t.Fatalf("expected TurnSignalLeft during leftward bridge, got %d", got)
	}
}

func TestTurnSignalFromMapLinkRight(t *testing.T) {
	// Two connected splines: s1 ends where s2 starts. s1 marks s2 as a
	// right-turn successor.
	s1 := NewSpline(1,
		NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0),
	)
	s2 := NewSpline(2,
		NewVec2(15, 0), NewVec2(18, 0), NewVec2(21, 2), NewVec2(24, 6),
	)
	s1.RightTurnLinkIDs = []int{2}
	graph := NewRoadGraph([]Spline{s1, s2}, nil)
	car := Car{
		CurrentSplineID:     1,
		DistanceOnSpline:    10,
		DestinationSplineID: 2,
		DesiredLaneSplineID: -1,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalRight {
		t.Fatalf("expected TurnSignalRight from right turn link, got %d", got)
	}
}

func TestTurnSignalFromMapLinkLeft(t *testing.T) {
	s1 := NewSpline(1,
		NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0),
	)
	s2 := NewSpline(2,
		NewVec2(15, 0), NewVec2(18, 0), NewVec2(21, -2), NewVec2(24, -6),
	)
	s1.LeftTurnLinkIDs = []int{2}
	graph := NewRoadGraph([]Spline{s1, s2}, nil)
	car := Car{
		CurrentSplineID:     1,
		DistanceOnSpline:    10,
		DestinationSplineID: 2,
		DesiredLaneSplineID: -1,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalLeft {
		t.Fatalf("expected TurnSignalLeft from left turn link, got %d", got)
	}
}

func TestTurnSignalMapLinkIgnoredIfNextSplineDiffers(t *testing.T) {
	// s1 has a right-turn link to s3, but the car's destination routes
	// through s2. No signal should fire.
	s1 := NewSpline(1,
		NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0),
	)
	s2 := NewSpline(2,
		NewVec2(15, 0), NewVec2(18, 0), NewVec2(22, 0), NewVec2(25, 0),
	)
	s3 := NewSpline(3,
		NewVec2(15, 0), NewVec2(17, 2), NewVec2(20, 5), NewVec2(23, 8),
	)
	s1.RightTurnLinkIDs = []int{3}
	graph := NewRoadGraph([]Spline{s1, s2, s3}, nil)
	car := Car{
		CurrentSplineID:     1,
		DistanceOnSpline:    10,
		DestinationSplineID: 2,
		DesiredLaneSplineID: -1,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalNone {
		t.Fatalf("turn link to a spline not on route should not fire; got %d", got)
	}
}

func TestTurnSignalLaneChangeBeatsMapLink(t *testing.T) {
	// Active lane change to the right should win even if a map link on the
	// source spline says turn left.
	bridge := NewSpline(10,
		NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, -3), NewVec2(15, -3),
	)
	s1 := NewSpline(1,
		NewVec2(0, 0), NewVec2(5, 0), NewVec2(10, 0), NewVec2(15, 0),
	)
	s1.LeftTurnLinkIDs = []int{2}
	graph := NewRoadGraph([]Spline{s1, bridge}, nil)
	car := Car{
		CurrentSplineID:     10,
		LaneChanging:        true,
		LaneChangeSplineID:  10,
		AfterSplineID:       2,
		DestinationSplineID: 2,
		DesiredLaneSplineID: -1,
	}
	if got := computeCarTurnSignal(car, graph); got != TurnSignalRight {
		t.Fatalf("lane change (right) should win over map link (left); got %d", got)
	}
}

func TestAssignCarTurnSignalsSkipsExternal(t *testing.T) {
	splines := []Spline{
		turnSignalStraightSpline(1, 0, 0, 30, 0),
		turnSignalStraightSpline(2, 0, 4, 30, 4),
	}
	graph := turnSignalGraph(splines)
	cars := []Car{
		{CurrentSplineID: 1, DistanceOnSpline: 5, DesiredLaneSplineID: 2},
		{CurrentSplineID: 1, DistanceOnSpline: 5, DesiredLaneSplineID: 2, ControlMode: CarControlExternal, TurnSignal: TurnSignalNone},
	}
	assignCarTurnSignals(cars, graph)
	if cars[0].TurnSignal != TurnSignalLeft {
		t.Fatalf("AI car should get TurnSignalLeft, got %d", cars[0].TurnSignal)
	}
	if cars[1].TurnSignal != TurnSignalNone {
		t.Fatalf("external car should be left untouched, got %d", cars[1].TurnSignal)
	}
}
