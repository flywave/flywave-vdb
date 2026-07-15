package vdb

import (
	"testing"

	vec2d "github.com/flywave/go3d/float64/vec2"
)

func TestDefaultOptions(t *testing.T) {
	o := DefaultOptions()
	if o.Level != 18 {
		t.Errorf("Level = %d, want 18", o.Level)
	}
	if o.Precision != 2.0 {
		t.Errorf("Precision = %v, want 2.0", o.Precision)
	}
	if o.MaxCacheSize != DEFAULT_MAX_CACHE_SIZE {
		t.Errorf("MaxCacheSize = %v, want %v", o.MaxCacheSize, DEFAULT_MAX_CACHE_SIZE)
	}
	if o.GridClass != GC_LEVEL_SET {
		t.Errorf("GridClass = %v, want GC_LEVEL_SET", o.GridClass)
	}
	// Default bounding box spans the whole space.
	if o.Boundbox.Min != vec2d.MinVal || o.Boundbox.Max != vec2d.MaxVal {
		t.Errorf("Boundbox not full extent: %+v", o.Boundbox)
	}
	// Quality knobs must be in a sane range.
	if o.TextureQuality <= 0 || o.TextureQuality > 1 {
		t.Errorf("TextureQuality out of (0,1]: %v", o.TextureQuality)
	}
	if o.IsoValue <= 0 {
		t.Errorf("IsoValue not positive: %v", o.IsoValue)
	}
}

func TestTypeConstants(t *testing.T) {
	// GridClass values are passed through to the C API; ensure they are stable.
	if GC_LEVEL_SET != 0 || GC_SURFACE != 1 {
		t.Errorf("GridClass constants shifted: LS=%d SURF=%d", GC_LEVEL_SET, GC_SURFACE)
	}
	if CT_UNION != 0 || CT_INTERSECTION != 1 || CT_DIFFERENCE != 2 {
		t.Errorf("CompositeType constants shifted")
	}
	if ST_POINT != 0 || ST_BOX != 1 || ST_QUADRATIC != 2 {
		t.Errorf("SamplerType constants shifted")
	}
}
