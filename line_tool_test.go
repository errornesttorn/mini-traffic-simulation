package main

import (
	"testing"

	simpkg "github.com/errornesttorn/mini-traffic-simulation-core"
)

func TestLineControlPointDefaultsToMidpoint(t *testing.T) {
	draft := LineDraft{P0: NewVec2(0, 0)}
	got := lineControlPoint(draft, NewVec2(10, 10), EndHit{}, nil)
	assertVecNear(t, got, NewVec2(5, 5))
}

func TestLineControlPointAlignsToPreviousSplineAxis(t *testing.T) {
	prev := simpkg.NewSpline(1, NewVec2(-10, 0), NewVec2(-5, 0), NewVec2(-2, 0), NewVec2(0, 0))
	draft := LineDraft{
		P0:           prev.P3,
		FromPrevAxis: true,
		PrevAxisDir:  endpointAxisDir(prev, EndFinish),
	}

	got := lineControlPoint(draft, NewVec2(10, 10), EndHit{}, []Spline{prev})
	assertVecNear(t, got, NewVec2(5, 0))
}

func TestLineControlPointAlignsToNextSplineAxis(t *testing.T) {
	next := simpkg.NewSpline(2, NewVec2(10, 10), NewVec2(10, 13), NewVec2(14, 18), NewVec2(18, 20))
	draft := LineDraft{P0: NewVec2(0, 0)}
	hovered := EndHit{SplineIndex: 0, Kind: EndStart}

	got := lineControlPoint(draft, next.P0, hovered, []Spline{next})
	assertVecNear(t, got, NewVec2(10, 5))
}

func TestLineControlPointUsesEndpointAxisIntersectionWhenBothEndsConnect(t *testing.T) {
	prev := simpkg.NewSpline(1, NewVec2(-10, 0), NewVec2(-5, 0), NewVec2(-2, 0), NewVec2(0, 0))
	next := simpkg.NewSpline(2, NewVec2(10, 10), NewVec2(10, 13), NewVec2(14, 18), NewVec2(18, 20))
	draft := LineDraft{
		P0:           prev.P3,
		FromPrevAxis: true,
		PrevAxisDir:  endpointAxisDir(prev, EndFinish),
	}
	hovered := EndHit{SplineIndex: 0, Kind: EndStart}

	got := lineControlPoint(draft, next.P0, hovered, []Spline{next})
	assertVecNear(t, got, NewVec2(10, 0))
}
