//go:build !darwin

package main

import (
	"os"
	"testing"
)

func TestParseAscHeader(t *testing.T) {
	p := "/home/eryk/workspace/warsaw-commute-game/example-map/83232_1744593_N-34-138-B-b-3-2.asc"
	if _, err := os.Stat(p); err != nil {
		t.Skip("example asc not available")
	}
	h, err := parseAscHeader(p)
	if err != nil {
		t.Fatal(err)
	}
	if h.NCols != 4385 || h.NRows != 4745 {
		t.Fatalf("unexpected dims: %+v", h)
	}
	if h.CellSize != 0.5 {
		t.Fatalf("unexpected cellsize: %v", h.CellSize)
	}
	if h.LLIsCorner {
		t.Fatalf("expected center not corner")
	}
}
