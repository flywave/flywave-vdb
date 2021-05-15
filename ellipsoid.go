package vdb

import (
	"math"

	"github.com/flywave/go3d/float64/vec3"

	mat4d "github.com/flywave/go3d/float64/mat4"
	quatd "github.com/flywave/go3d/float64/quaternion"
	vec3d "github.com/flywave/go3d/float64/vec3"
)

const (
	_e                     = 0.0066943799901413165
	EARTH_RADIUS           = float64(6378137)
	EARTH_DIAMETER         = EARTH_RADIUS * 2.0
	EARTH_CIRCUMFERENCE    = EARTH_DIAMETER * math.Pi
	INVERSE_FLATTEN_FACTOR = float64(1.00336409)
	MAXEXTENT              = EARTH_CIRCUMFERENCE / 2.0
	M_PI_by2               = float64(math.Pi / 2)
	D2R                    = float64(math.Pi / 180)
	R2D                    = float64(180 / math.Pi)
	M_PIby360              = float64(math.Pi / 360)
	MAXEXTENTby180         = float64(MAXEXTENT / 180)
)

type Ellipsoid struct {
	Space
}

func (e *Ellipsoid) ToGridWord(xyz vec3d.T) vec3d.T {
	xyz[0] = xyz[0] / EARTH_RADIUS
	xyz[1] = xyz[1] / EARTH_RADIUS
	xyz[2] = xyz[2] / EARTH_RADIUS

	pxyz := e.proj(e.xyzToLonlat(xyz))

	pxyz[0] = pxyz[0] * EARTH_RADIUS
	pxyz[1] = pxyz[1] * EARTH_RADIUS
	pxyz[2] = pxyz[2] * EARTH_RADIUS
	return pxyz
}

func (e *Ellipsoid) ToSpaceWord(vxyz vec3d.T) vec3d.T {
	vxyz[0] = vxyz[0] / EARTH_RADIUS
	vxyz[1] = vxyz[1] / EARTH_RADIUS
	vxyz[2] = vxyz[2] / EARTH_RADIUS

	pxyz := e.proj(e.lonlatToXyz(vxyz))

	pxyz[0] = pxyz[0] * EARTH_RADIUS
	pxyz[1] = pxyz[1] * EARTH_RADIUS
	pxyz[2] = pxyz[2] * EARTH_RADIUS
	return pxyz
}

func (e *Ellipsoid) TileToSpace(xyz vec3d.T) mat4d.T {
	xyz[0] = xyz[0] / EARTH_RADIUS
	xyz[1] = xyz[1] / EARTH_RADIUS
	xyz[2] = xyz[2] / EARTH_RADIUS

	coord := e.xyzToLonlat(xyz)

	qz := quatd.FromAxisAngle(&vec3.UnitZ, (-coord[0]*180/math.Pi)*D2R)
	qy := quatd.FromAxisAngle(&vec3.UnitY, (coord[1]*180/math.Pi)*D2R)

	q := quatd.Mul(&qy, &qz)

	mt := mat4d.Ident
	mt.AssignQuaternion(&q)

	mt.SetTranslation(&xyz)

	return mt
}

func (e *Ellipsoid) MakeTileRay(xyz vec3d.T, max vec3d.T) (error, *Ray) {
	return NewRay(vec3d.T{math.Ceil(max[0]), xyz[1], xyz[2]}, vec3d.T{-1, 0, 0})
}

func (e *Ellipsoid) ComputeDistanceFromSurface(vxyz vec3d.T) float64 {
	vxyz[0] = vxyz[0] / EARTH_RADIUS
	vxyz[1] = vxyz[1] / EARTH_RADIUS
	vxyz[2] = vxyz[2] / EARTH_RADIUS

	coord := e.xyzToLonlat(vxyz)
	coord[2] = 0
	g := e.lonlatToXyz(coord)

	return (vxyz.Length() - g.Length()) * EARTH_RADIUS
}

func (e *Ellipsoid) ComputePointFromElevation(vxyz vec3d.T, elevation float64) vec3d.T {
	vxyz[0] = vxyz[0] / EARTH_RADIUS
	vxyz[1] = vxyz[1] / EARTH_RADIUS
	vxyz[2] = vxyz[2] / EARTH_RADIUS

	coord := e.xyzToLonlat(vxyz)
	coord[2] = elevation / EARTH_RADIUS
	g := e.lonlatToXyz(coord)

	g[0] = g[0] * EARTH_RADIUS
	g[1] = g[1] * EARTH_RADIUS
	g[2] = g[2] * EARTH_RADIUS
	return g
}

func (e *Ellipsoid) TileSize(min vec3d.T, max vec3d.T) float64 {
	min[0] = min[0] / EARTH_RADIUS
	min[1] = min[1] / EARTH_RADIUS
	min[2] = min[2] / EARTH_RADIUS
	max[0] = max[0] / EARTH_RADIUS
	max[1] = max[1] / EARTH_RADIUS
	max[2] = max[2] / EARTH_RADIUS

	min = e.xyzToLonlat(min)
	max = e.xyzToLonlat(max)

	l := (max[0] - min[0]) / 2.0

	return math.Sin(l) * EARTH_RADIUS * 2.0
}

func (e *Ellipsoid) rayCastToEllipsoid(ret vec3d.T, eye vec3d.T) float64 {
	radio := float64(1)
	dir := eye.Invert()

	u := eye[2] * INVERSE_FLATTEN_FACTOR
	n := dir[2] * INVERSE_FLATTEN_FACTOR

	w := dir[0]*dir[0] + dir[1]*dir[1] + n*n
	t := 2 * (dir[0]*eye[0] + dir[1]*eye[1] + n*u)

	s := eye[0]*eye[0] + eye[1]*eye[1] + u*u - radio*radio

	if s > 0 {
		r := t*t - 4*w*s
		if r > 0 {
			r = (-t - math.Sqrt(r)) / (2 * w)
			ret[0] = eye[0] + dir[0]*r
			ret[1] = eye[1] + dir[1]*r
			ret[2] = eye[2] + dir[2]*r
			return r
		}
	}
	return -1
}

func (e *Ellipsoid) xyzToLonlat(xyz vec3d.T) vec3d.T {
	x := xyz[0]
	y := xyz[1]
	z := xyz[2]
	C := float64(1)
	var w vec3d.T
	r := math.Sqrt(y*y + x*x)
	t := math.Atan(z / (r * math.Sqrt(1.0-_e)))
	q := math.Atan2(y, x)
	_G := math.Sin(t)
	B := math.Cos(t)
	A := math.Atan((z + _G*_G*_G*C*_e/math.Sqrt(1.0-_e)) /
		(r - C*_e*B*B*B))
	G := math.Sin(A)
	v := C / math.Sqrt(1.0-_e*G*G)
	u := r/math.Cos(A) - v
	w[0] = q
	w[1] = A
	w[2] = u
	return w
}

func (e *Ellipsoid) lonlatToXyz(pxyz vec3d.T) vec3d.T {
	lon := pxyz[0]
	lat := pxyz[1]
	h := pxyz[2]

	var xyz vec3d.T

	r := lon
	y := lat
	z := math.Sin(y)
	v := math.Cos(y)
	u := 1.0 / math.Sqrt(1.0-_e*z*z)

	xyz[0] = (u + h) * v * math.Cos(r)
	xyz[1] = (u + h) * v * math.Sin(r)
	xyz[2] = (u*(1-_e) + h) * z

	return xyz
}

func (e *Ellipsoid) proj(coord vec3d.T) vec3d.T {
	return vec3d.T{coord[0], math.Log(math.Tan(math.Pi/4.0 + coord[1]/2.0)), coord[2]}
}

func (e *Ellipsoid) unproj(coord vec3d.T) vec3d.T {
	return vec3d.T{coord[0], (2.0*math.Atan(math.Exp(coord[1])) - M_PI_by2), coord[2]}
}
