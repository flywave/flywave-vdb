package vdb

const (
	DEFAULT_MAX_CACHE_SIZE = int64(1024 * 1024 * 1024)
)

type Options struct {
	Boundbox             BBox2d
	Level                uint16
	Precision            float64
	TextureQuality       float64
	IsoValue             float64
	AdapterValue         float64
	FaceQuality          float64
	PackTrianglePixelPad float64
	MaxCacheSize         int64
}

func DefaultOptions() Options {
	return Options{Boundbox: *NewBBox2d(), Level: 18, Precision: 2.0, TextureQuality: 0.8, IsoValue: 0.01, AdapterValue: 0.01, FaceQuality: 1.0, PackTrianglePixelPad: 2.0, MaxCacheSize: DEFAULT_MAX_CACHE_SIZE}
}
