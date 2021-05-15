package vdb

import (
	"errors"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

type Ray struct {
	Origin vec3d.T
	Dir    vec3d.T
}

func NewRay(eye, dir vec3d.T) (error, *Ray) {
	if len(eye) != 3 || len(dir) != 3 {
		return errors.New("bbox size must 6"), nil
	}
	return nil, &Ray{Origin: eye, Dir: dir}
}
