package vdb

import (
	"testing"
)

func TestTransferFunctionCreateFree(t *testing.T) {
	tf := NewTransferFunction()
	if tf == nil {
		t.Fatal("NewTransferFunction returned nil")
	}
	tf.Free()
}

func TestTransferFunctionAddAndMap(t *testing.T) {
	tf := NewTransferFunction()
	defer tf.Free()
	tf.AddPoint(0.0, 0, 0, 0, 0)
	tf.AddPoint(0.5, 0.5, 0.5, 0.5, 0.5)
	tf.AddPoint(1.0, 1, 1, 1, 1)
	tf.Build()

	r, g, b, a := tf.Map(0.0)
	if r != 0 || g != 0 || b != 0 || a != 0 {
		t.Errorf("at 0.0: got (%f,%f,%f,%f), want (0,0,0,0)", r, g, b, a)
	}
	r, g, b, a = tf.Map(1.0)
	if r != 1 || g != 1 || b != 1 || a != 1 {
		t.Errorf("at 1.0: got (%f,%f,%f,%f), want (1,1,1,1)", r, g, b, a)
	}
	r, g, b, a = tf.Map(0.5)
	if r != 0.5 || g != 0.5 || b != 0.5 || a != 0.5 {
		t.Errorf("at 0.5: got (%f,%f,%f,%f), want (0.5,0.5,0.5,0.5)", r, g, b, a)
	}
}

func TestTransferFunctionClamp(t *testing.T) {
	tf := NewTransferFunction()
	defer tf.Free()
	tf.AddPoint(0.2, 1, 0, 0, 1)
	r, g, b, a := tf.Map(0.0)
	if r != 1 || a != 1 {
		t.Errorf("below min should clamp: got (%f,%f,%f,%f)", r, g, b, a)
	}
	r, g, b, a = tf.Map(1.0)
	if r != 1 || a != 1 {
		t.Errorf("above max should clamp: got (%f,%f,%f,%f)", r, g, b, a)
	}
}

func TestCameraOrbit(t *testing.T) {
	cam := CameraOrbit([3]float64{0, 0, 0}, 10, 0, 0, 45)
	if cam.Eye[0] == 0 && cam.Eye[1] == 0 && cam.Eye[2] == 0 {
		t.Errorf("orbit camera eye should be on sphere, got %v", cam.Eye)
	}
	if cam.Fov != 45 {
		t.Errorf("Fov = %f, want 45", cam.Fov)
	}
	if cam.Target != [3]float64{0, 0, 0} {
		t.Errorf("Target should be center")
	}
}

func TestCameraOrbitPath(t *testing.T) {
	cams := CameraOrbitPath([3]float64{0, 0, 0}, 10, 60, 12)
	if len(cams) != 12 {
		t.Errorf("got %d cameras, want 12", len(cams))
	}
	if cams[0].Fov != 60 {
		t.Errorf("Fov not set")
	}
	// Ensure all cameras are at unique positions
	positions := map[[3]float64]int{}
	for i, c := range cams {
		positions[c.Eye]++
		if positions[c.Eye] > 1 {
			t.Errorf("duplicate camera position at index %d: %v", i, c.Eye)
		}
	}
}

func TestFloatGridRayMarch(t *testing.T) {
	points := []float64{0, 0, 0}
	radius := []float64{2.0}
	g := NewFloatGridFromPoints(points, radius, 0.2, 2)
	defer g.Free()

	tf := NewTransferFunction()
	defer tf.Free()
	tf.AddPoint(0.0, 0, 0, 0, 0)
	tf.AddPoint(0.5, 0, 1, 0, 0.5)
	tf.AddPoint(1.0, 0, 0, 1, 1)
	tf.Build()

	// Fire ray from above the sphere, looking down
	ok, r, g_, b, a, depth := g.RayMarch(0, 5, 0, 0, -1, 0, tf, 0.1, 15)
	if !ok {
		t.Errorf("ray should hit the sphere")
	}
	if a < 0.01 {
		t.Errorf("accumulated opacity too low: %f", a)
	}
	if depth < 0 {
		t.Errorf("depth negative: %f", depth)
	}
	t.Logf("ray result: ok=%v rgba=(%f,%f,%f,%f) depth=%f", ok, r, g_, b, a, depth)
}
