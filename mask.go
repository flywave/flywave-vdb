package vdb

type Mask struct {
	Volume *FloatGrid
	Min    float64
	Max    float64
	Invert bool
}

func NewMask() *Mask {
	return &Mask{
		Volume: NewFloatGrid(),
		Min:    0,
		Max:    0,
		Invert: false,
	}
}

func NewMaskFromGrid(g *FloatGrid) *Mask {
	return &Mask{
		Volume: g,
		Min:    0,
		Max:    0,
		Invert: false,
	}
}
