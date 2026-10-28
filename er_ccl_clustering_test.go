package obstaclespointcloud

import (
	"context"
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	pc "go.viam.com/rdk/pointcloud"
)

// A cloud with no extent in the grid axes used to make gridResolution return 0, which is then a
// divisor: every index became int(math.Ceil(0/0)) = int(NaN). On amd64 that is MinInt64 and
// pcProjection panics with "index out of range [-9223372036854775808]"; on arm64 it saturates to
// 0 and nothing looks wrong. These assert the divisor directly so they mean the same thing on
// either architecture.
func TestGridResolutionIsNeverZero(t *testing.T) {
	for _, tc := range []struct {
		name string
		md   pc.MetaData
	}{
		{"a single point, so no extent at all", pc.MetaData{
			MinX: 200, MaxX: 200, MinY: 200, MaxY: 200, MinZ: 50, MaxZ: 50,
		}},
		{"several points sharing x and y", pc.MetaData{
			MinX: 200, MaxX: 200, MinY: 200, MaxY: 200, MinZ: 50, MaxZ: 60,
		}},
		{"an empty cloud, whose bounds are +/-MaxFloat", pc.NewBasicEmpty().MetaData()},
	} {
		t.Run(tc.name, func(t *testing.T) {
			for _, heightIsY := range []bool{false, true} {
				got := gridResolution(tc.md, heightIsY)
				test.That(t, math.IsNaN(got), test.ShouldBeFalse)
				test.That(t, got, test.ShouldBeGreaterThanOrEqualTo, 1)
			}
		})
	}
}

func TestGridResolutionKeepsItsOldAnswerWhenThereIsExtent(t *testing.T) {
	md := pc.MetaData{MinX: 0, MaxX: 1000, MinY: 0, MaxY: 600, MinZ: 0, MaxZ: 400}
	test.That(t, gridResolution(md, false), test.ShouldEqual, 4) // ceil((ceil(3)+ceil(5))/2)
	test.That(t, gridResolution(md, true), test.ShouldEqual, 4)  // ceil((ceil(2)+ceil(5))/2)
}

// The whole path, from the scene that triggered this in the field: a flat table with one point
// standing above it. Before the fix this panicked on amd64 and took the module down with it,
// which a caller sees only as "UNAVAILABLE ... error reading from server: EOF".
func TestOneObjectOverAFlatPlaneDoesNotPanic(t *testing.T) {
	cloud := pc.NewBasicEmpty()
	for i := 0; i < 40; i++ {
		for j := 0; j < 40; j++ {
			test.That(t, cloud.Set(r3.Vector{X: float64(i) * 10, Y: float64(j) * 10, Z: 0}, nil),
				test.ShouldBeNil)
		}
	}
	test.That(t, cloud.Set(r3.Vector{X: 200, Y: 200, Z: 50}, nil), test.ShouldBeNil)

	cfg := &ErCCLConfig{MinPtsInPlane: 100, MaxDistFromPlane: 10, MinPtsInSegment: 1,
		ClusteringRadius: 10, ClusteringStrictness: 3}
	cfg.SetDefaultValues()
	objs, err := ApplyERCCLToPointCloud(context.Background(), cloud, cfg)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(objs), test.ShouldEqual, 1)
}
