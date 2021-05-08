package vdb

import "errors"

type Ray struct {
	origin []float64
	dir    []float64
}

func NewRay(eye []float64, dir []float64) (error, *Ray) {
	if len(eye) != 3 || len(dir) != 3 {
		return errors.New("bbox size must 6"), nil
	}
	return nil, &Ray{origin: eye, dir: dir}
}

func (b *Ray) SetOrigin(ori []float64) {
	b.origin = ori
}

func (b *Ray) SetDir(d []float64) {
	b.dir = d
}

func (b *Ray) GetOrigin() []float64 {
	return b.origin
}

func (b *Ray) GetDir() []float64 {
	return b.dir
}
