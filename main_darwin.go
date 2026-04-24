//go:build darwin

package main

import (
	"fmt"
	"os"
)

func main() {
	fmt.Fprintln(os.Stderr, "mini-traffic-simulation does not support macOS")
	os.Exit(1)
}
