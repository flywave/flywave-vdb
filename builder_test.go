package vdb

import (
	"testing"
)

func TestMeshBuilder(t *testing.T) {
	builder := NewVoxelMeshBuilder()
	defer builder.Free()

	builder.SetName("test")

	if builder.GetName() != "test" {
		t.FailNow()
	}

	mesh := &MeshModel{}
	mesh.buildBox(2, 2)

	if len(mesh.Vertices) != 12*6 {
		t.FailNow()
	}

	builder.AddMesh(mesh)
	vmesh := builder.BuildMesh()
	defer vmesh.Free()

	if vmesh.Empty() {
		t.FailNow()
	}

	clip := &NoneClipBoxCreateor{}
	mat := []float64{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}
	precision := float32(10)

	vpixel := vmesh.SampleVoxelPixel(nil, 1, precision, clip, ST_LEVEL_SET, mat)

	if vpixel.Empty() {
		t.FailNow()
	}
}
