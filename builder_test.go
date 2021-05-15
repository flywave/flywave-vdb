package vdb

import (
	"strconv"
	"testing"

	"github.com/flywave/flywave-vdb/coord"
	_ "github.com/flywave/go-obj"
	mat4d "github.com/flywave/go3d/float64/mat4"
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
	mat := mat4d.Ident
	precision := float32(10)

	vpixel := vmesh.SampleVoxelPixel(nil, 1, precision, clip, GC_LEVEL_SET, mat)

	if vpixel.Empty() {
		t.FailNow()
	}

	grid := vpixel.GetVoxelGrid()

	str := grid.PrintInfo()

	print(str)

	grid.VisitOn(func(c coord.T, v float32) bool {
		print(strconv.Itoa(int(c[0])) + "_" + strconv.Itoa(int(c[1])) + "_" + strconv.Itoa(int(c[2])) + "_")
		return true
	})
}
