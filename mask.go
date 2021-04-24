package vdb

type Mask struct {
	Volume *Grid
	Min    float64
	Max    float64
	Invert bool
}

func NewMask() *Mask {
	return &Mask{
		Volume: NewGrid(),
		Min:    0,
		Max:    0,
		Invert: false,
	}
}

func NewMaskFromGrid(g *Grid) *Mask {
	return &Mask{
		Volume: g,
		Min:    0,
		Max:    0,
		Invert: false,
	}
}
