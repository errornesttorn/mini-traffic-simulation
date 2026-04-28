//go:build !darwin

package main

import (
	"bufio"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	rl "github.com/gen2brain/raylib-go/raylib"
)

// MapTile is an immutable background tile loaded from a .tif/.asc pair.
// Position and size are expressed in simulation (raylib) coordinates.
type MapTile struct {
	DemPath   string
	OrthoPath string
	Texture   rl.Texture2D
	LeftX     float32
	TopY      float32
	WidthM    float32
	HeightM   float32
}

type ascHeader struct {
	NCols      int
	NRows      int
	XLLCenter  float64
	YLLCenter  float64
	CellSize   float64
	LLIsCorner bool
}

func parseAscHeader(path string) (*ascHeader, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	sc := bufio.NewScanner(f)
	sc.Buffer(make([]byte, 1024*1024), 64*1024*1024)
	h := &ascHeader{}
	seen := 0
	for seen < 6 && sc.Scan() {
		line := strings.TrimSpace(sc.Text())
		if line == "" {
			continue
		}
		parts := strings.Fields(line)
		if len(parts) < 2 {
			break
		}
		key := strings.ToLower(parts[0])
		val := parts[1]
		switch key {
		case "ncols":
			h.NCols, _ = strconv.Atoi(val)
		case "nrows":
			h.NRows, _ = strconv.Atoi(val)
		case "xllcenter":
			h.XLLCenter, _ = strconv.ParseFloat(val, 64)
		case "xllcorner":
			h.XLLCenter, _ = strconv.ParseFloat(val, 64)
			h.LLIsCorner = true
		case "yllcenter":
			h.YLLCenter, _ = strconv.ParseFloat(val, 64)
		case "yllcorner":
			h.YLLCenter, _ = strconv.ParseFloat(val, 64)
			h.LLIsCorner = true
		case "cellsize":
			h.CellSize, _ = strconv.ParseFloat(val, 64)
		case "nodata_value":
			// ignored
		default:
			// header ended
			return h, nil
		}
		seen++
	}
	return h, nil
}

// convertTiffToPngFast uses gdal_translate to extract a downsampled PNG of the orthophoto.
// We use gdal_translate because it's much faster than ImageMagick's convert and uses TIFF overviews automatically.
func convertTiffToPngFast(tifPath string, outPath string) error {
	if _, err := exec.LookPath("gdal_translate"); err != nil {
		return errors.New("gdal_translate is required to downsample the GeoTIFF orthophoto")
	}
	cmd := exec.Command("gdal_translate", "-q", "-outsize", "2048", "0", "-of", "PNG", tifPath, outPath)
	cmd.Stderr = os.Stderr
	return cmd.Run()
}

type mapJson struct {
	Version      int    `json:"version"`
	Name         string `json:"name"`
	Simulation   string `json:"simulation,omitempty"`
	RaylibCenter struct {
		X float64 `json:"x"`
		Y float64 `json:"y"`
	} `json:"raylib_center"`
	Tiles []struct {
		Dem   string `json:"dem"`
		Ortho string `json:"ortho"`
	} `json:"tiles"`
	Orthos []string `json:"orthos,omitempty"`
}

// loadMapFormat reads a map.json and its referenced tiles. Returns the loaded
// tiles, the raylib centre (in EPSG metric coords), the name of the
// referenced simulation file (empty if none), and any error.
func loadMapFormat(mapJsonPath string) (tiles []MapTile, centerX, centerY float64, simName string, mapName string, err error) {
	data, err := os.ReadFile(mapJsonPath)
	if err != nil {
		return nil, 0, 0, "", "", err
	}
	var m mapJson
	if err := json.Unmarshal(data, &m); err != nil {
		return nil, 0, 0, "", "", err
	}
	dir := filepath.Dir(mapJsonPath)
	centerX = m.RaylibCenter.X
	centerY = m.RaylibCenter.Y
	simName = m.Simulation
	mapName = m.Name

	for _, t := range m.Tiles {
		ascPath := filepath.Join(dir, t.Dem)
		tifPath := filepath.Join(dir, t.Ortho)
		h, aerr := parseAscHeader(ascPath)
		if aerr != nil {
			err = fmt.Errorf("parse %s: %w", t.Dem, aerr)
			return
		}
		cachePath := filepath.Join(dir, ".cache_"+strings.TrimSuffix(filepath.Base(t.Ortho), filepath.Ext(t.Ortho))+".png")
		if _, statErr := os.Stat(cachePath); statErr != nil {
			if cerr := convertTiffToPngFast(tifPath, cachePath); cerr != nil {
				err = fmt.Errorf("convert %s: %w", t.Ortho, cerr)
				return
			}
		}
		tex := rl.LoadTexture(cachePath)
		if tex.ID == 0 {
			err = fmt.Errorf("failed to load cached texture %s", cachePath)
			return
		}

		widthM := float64(h.NCols) * h.CellSize
		heightM := float64(h.NRows) * h.CellSize
		var leftEpsg, topEpsg float64
		if h.LLIsCorner {
			leftEpsg = h.XLLCenter
			topEpsg = h.YLLCenter + heightM
		} else {
			leftEpsg = h.XLLCenter - h.CellSize/2
			topEpsg = h.YLLCenter - h.CellSize/2 + heightM
		}
		leftR := float32(leftEpsg - centerX)
		topR := float32(-(topEpsg - centerY))

		tiles = append(tiles, MapTile{
			DemPath:   ascPath,
			OrthoPath: tifPath,
			Texture:   tex,
			LeftX:     leftR,
			TopY:      topR,
			WidthM:    float32(widthM),
			HeightM:   float32(heightM),
		})
	}

	if len(m.Orthos) > 0 {
		orthoPaths, rerr := resolveMapFilePatterns(dir, m.Orthos)
		if rerr != nil {
			err = fmt.Errorf("resolve ortho files: %w", rerr)
			return
		}

		for _, orthoPath := range orthoPaths {
			west, east, south, north, berr := readOrthoBounds(orthoPath)
			if berr != nil {
				err = fmt.Errorf("read bounds for %s: %w", orthoPath, berr)
				return
			}

			cachePath := filepath.Join(dir, ".cache_"+strings.TrimSuffix(filepath.Base(orthoPath), filepath.Ext(orthoPath))+".png")
			if _, statErr := os.Stat(cachePath); statErr != nil {
				if cerr := convertTiffToPngFast(orthoPath, cachePath); cerr != nil {
					err = fmt.Errorf("convert %s: %w", orthoPath, cerr)
					return
				}
			}
			tex := rl.LoadTexture(cachePath)
			if tex.ID == 0 {
				err = fmt.Errorf("failed to load cached texture %s", cachePath)
				return
			}

			leftR := float32(west - centerX)
			topR := float32(-(north - centerY))
			widthM := float32(east - west)
			heightM := float32(north - south)

			tiles = append(tiles, MapTile{
				OrthoPath: orthoPath,
				Texture:   tex,
				LeftX:     leftR,
				TopY:      topR,
				WidthM:    widthM,
				HeightM:   heightM,
			})
		}
	}

	return
}

// updateMapJsonSimulationField rewrites map.json preserving all fields but
// setting the "simulation" field to simName.
func updateMapJsonSimulationField(mapJsonPath, simName string) error {
	data, err := os.ReadFile(mapJsonPath)
	if err != nil {
		return err
	}
	var raw map[string]interface{}
	if err := json.Unmarshal(data, &raw); err != nil {
		return err
	}
	raw["simulation"] = simName
	out, err := json.MarshalIndent(raw, "", "  ")
	if err != nil {
		return err
	}
	return os.WriteFile(mapJsonPath, out, 0644)
}

func drawMapTiles(tiles []MapTile) {
	for _, t := range tiles {
		if t.Texture.ID == 0 {
			continue
		}
		srcRec := rl.NewRectangle(0, 0, float32(t.Texture.Width), float32(t.Texture.Height))
		dstRec := rl.NewRectangle(t.LeftX, t.TopY, t.WidthM, t.HeightM)
		rl.DrawTexturePro(t.Texture, srcRec, dstRec, rl.NewVector2(0, 0), 0, rl.White)
	}
}

func resolveMapFilePatterns(baseDir string, entries []string) ([]string, error) {
	var paths []string
	seen := map[string]bool{}

	for _, entry := range entries {
		if strings.TrimSpace(entry) == "" {
			continue
		}

		pattern := entry
		if !filepath.IsAbs(pattern) {
			pattern = filepath.Join(baseDir, pattern)
		}
		pattern = filepath.Clean(pattern)

		var matches []string
		if hasGlobMeta(pattern) {
			var err error
			matches, err = filepath.Glob(pattern)
			if err != nil {
				return nil, fmt.Errorf("%s: %w", entry, err)
			}
			if len(matches) == 0 {
				return nil, fmt.Errorf("%s: no matches", entry)
			}
		} else {
			matches = []string{pattern}
		}

		sort.Strings(matches)
		for _, match := range matches {
			match = filepath.Clean(match)
			if seen[match] {
				continue
			}
			seen[match] = true
			paths = append(paths, match)
		}
	}

	return paths, nil
}

func hasGlobMeta(path string) bool {
	return strings.ContainsAny(path, "*?[")
}

func readOrthoBounds(path string) (west, east, south, north float64, err error) {
	if _, lookErr := exec.LookPath("gdalinfo"); lookErr != nil {
		return 0, 0, 0, 0, errors.New("gdalinfo is required to read orthophoto bounds")
	}
	out, runErr := exec.Command("gdalinfo", "-json", path).Output()
	if runErr != nil {
		return 0, 0, 0, 0, fmt.Errorf("gdalinfo failed: %w", runErr)
	}
	var info struct {
		CornerCoordinates struct {
			UpperLeft  [2]float64 `json:"upperLeft"`
			LowerRight [2]float64 `json:"lowerRight"`
		} `json:"cornerCoordinates"`
	}
	if err := json.Unmarshal(out, &info); err != nil {
		return 0, 0, 0, 0, fmt.Errorf("parse gdalinfo output: %w", err)
	}
	west = info.CornerCoordinates.UpperLeft[0]
	north = info.CornerCoordinates.UpperLeft[1]
	east = info.CornerCoordinates.LowerRight[0]
	south = info.CornerCoordinates.LowerRight[1]
	if east <= west || north <= south {
		return 0, 0, 0, 0, fmt.Errorf("invalid georeference (corners %.3f,%.3f .. %.3f,%.3f)", west, north, east, south)
	}
	return west, east, south, north, nil
}
