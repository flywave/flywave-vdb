package vdb

// #include <stdlib.h>
// #include "multiview_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"

type TransferFunction struct {
	m *C.vdb_transfer_function_t
}

func NewTransferFunction() *TransferFunction {
	return &TransferFunction{m: C.vdb_transfer_function_create()}
}

func (tf *TransferFunction) Free() {
	C.vdb_transfer_function_free(tf.m)
	tf.m = nil
}

func (tf *TransferFunction) AddPoint(scalar, r, g, b, a float32) {
	C.vdb_transfer_function_add_point(tf.m, C.float(scalar), C.float(r), C.float(g), C.float(b), C.float(a))
}

func (tf *TransferFunction) Build() {
	C.vdb_transfer_function_build(tf.m)
}

func (tf *TransferFunction) Map(scalar float32) (r, g, b, a float32) {
	var cr, cg, cb, ca C.float
	C.vdb_transfer_function_map(tf.m, C.float(scalar), &cr, &cg, &cb, &ca)
	return float32(cr), float32(cg), float32(cb), float32(ca)
}

type Camera struct {
	Eye    [3]float64
	Target [3]float64
	Up     [3]float64
	Fov    float64
}

func CameraLookAt(eye, target, up [3]float64, fov float64) Camera {
	return Camera{Eye: eye, Target: target, Up: up, Fov: fov}
}

func CameraOrbit(center [3]float64, radius, azimuthDeg, elevationDeg, fov float64) Camera {
	var c C.vdb_camera_t
	C.vdb_camera_orbit(&c,
		C.double(center[0]), C.double(center[1]), C.double(center[2]),
		C.double(radius), C.double(azimuthDeg), C.double(elevationDeg),
		C.double(fov))
	return Camera{
		Eye:    [3]float64{float64(c.eye[0]), float64(c.eye[1]), float64(c.eye[2])},
		Target: [3]float64{float64(c.target[0]), float64(c.target[1]), float64(c.target[2])},
		Up:     [3]float64{float64(c.up[0]), float64(c.up[1]), float64(c.up[2])},
		Fov:    float64(c.fov),
	}
}

func CameraOrbitPath(center [3]float64, radius, fov float64, n int) []Camera {
	if n <= 0 {
		return nil
	}
	ccameras := make([]C.vdb_camera_t, n)
	C.vdb_camera_orbit_path(&ccameras[0], C.int(n),
		C.double(center[0]), C.double(center[1]), C.double(center[2]),
		C.double(radius), C.double(fov))
	cams := make([]Camera, n)
	for i := 0; i < n; i++ {
		cams[i] = Camera{
			Eye:    [3]float64{float64(ccameras[i].eye[0]), float64(ccameras[i].eye[1]), float64(ccameras[i].eye[2])},
			Target: [3]float64{float64(ccameras[i].target[0]), float64(ccameras[i].target[1]), float64(ccameras[i].target[2])},
			Up:     [3]float64{float64(ccameras[i].up[0]), float64(ccameras[i].up[1]), float64(ccameras[i].up[2])},
			Fov:    float64(ccameras[i].fov),
		}
	}
	return cams
}

func (g *FloatGrid) RayMarch(ox, oy, oz, dx, dy, dz float64, tfn *TransferFunction, stepSize, maxDist float64) (bool, float32, float32, float32, float32, float64) {
	var r, g_, b, a C.float
	var depth C.double
	ok := bool(C.vdb_grid_ray_march(g.m,
		C.double(ox), C.double(oy), C.double(oz),
		C.double(dx), C.double(dy), C.double(dz),
		tfn.m, C.double(stepSize), C.double(maxDist),
		&r, &g_, &b, &a, &depth))
	return ok, float32(r), float32(g_), float32(b), float32(a), float64(depth)
}
