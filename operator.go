package vdb

import "errors"

type OperatorType uint32

const (
	OP_VOXELIZE = OperatorType(0)
	OP_MERGE    = OperatorType(1)
	OP_MASK     = OperatorType(2)
	OP_SURFACE  = OperatorType(3)
)

type Operator interface {
	Apply() error
}

type VoxelizeOperator struct {
	Operator
	mesh         *VoxelMesh
	base         *VoxelPixel
	localFeature uint16
	precision    float32
	creator      ClipBoxCreateor
	tp           SamplerType
	matrix       []float64
}

func (f *VoxelizeOperator) GetVoxelPixel() *VoxelPixel {
	return f.base
}

func (f *VoxelizeOperator) Apply() error {
	if f.mesh == nil || f.base == nil {
		return errors.New("op not inited")
	}

	f.base = f.mesh.SampleVoxelPixel(nil, f.localFeature, f.precision, f.creator, f.tp, f.matrix)
	return nil
}

func NewVoxelizeOperator() *VoxelizeOperator {
	return nil
}

type CSGOperator struct {
	Operator
	mesh         *VoxelMesh
	base         *VoxelPixel
	target       *VoxelPixel
	ctp          CompositeType
	localFeature uint16
	precision    float32
	creator      ClipBoxCreateor
	tp           SamplerType
	matrix       []float64
}

func (f *CSGOperator) Apply() error {
	if f.mesh == nil || f.target == nil {
		return errors.New("op not inited")
	}
	mtls := f.base.GetMaterials()
	defer mtls.Free()
	f.target = f.mesh.SampleVoxelPixel(mtls, f.localFeature, f.precision, f.creator, f.tp, f.matrix)
	defer f.target.Free()
	f.base.SetMaterials(mtls)
	f.composite()
	return nil
}

func (f *CSGOperator) composite() error {
	if f.base == nil || f.target == nil {
		return errors.New("csg not init!")
	}
	f.base.Composite(f.target, f.ctp)
	return nil
}

type MergeOperator struct {
	Operator
}

func NewMergeOperator() *MergeOperator {
	return nil
}

type MaskOperator struct {
	Operator
}

func NewMaskOperator() *MergeOperator {
	return nil
}

type SurfaceOperator struct {
	Operator
	mesh         *VoxelMesh
	base         *VoxelPixel
	target       *VoxelPixel
	ctp          CompositeType
	localFeature uint16
	precision    float32
	creator      ClipBoxCreateor
	tp           SamplerType
	matrix       []float64
}

func (f *SurfaceOperator) Apply() error {
	if f.mesh == nil || f.target == nil {
		return errors.New("op not inited")
	}
	mtls := f.base.GetMaterials()
	defer mtls.Free()
	f.target = f.mesh.SampleVoxelPixel(mtls, f.localFeature, f.precision, f.creator, f.tp, f.matrix)
	defer f.target.Free()
	f.base.SetMaterials(mtls)
	color := f.base.ExtractColor(f.target)
	defer color.Free()
	f.base.FillColor(f.target, color)
	return nil
}

func NewSurfaceOperator() *SurfaceOperator {
	return nil
}
