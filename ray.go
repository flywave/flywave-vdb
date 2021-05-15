package vdb

import (
	"errors"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

type Ray struct {
	origin vec3d.T
	dir    vec3d.T
}

func NewRay(eye, dir vec3d.T) (error, *Ray) {
	if len(eye) != 3 || len(dir) != 3 {
		return errors.New("bbox size must 6"), nil
	}
	return nil, &Ray{origin: eye, dir: dir}
}

func (b *Ray) SetOrigin(ori vec3d.T) {
	b.origin = ori
}

func (b *Ray) SetDir(d vec3d.T) {
	b.dir = d
}

func (b *Ray) GetOrigin() *vec3d.T {
	return &b.origin
}

func (b *Ray) GetDir() *vec3d.T {
	return &b.dir
}
