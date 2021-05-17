package vdb

import (
	"errors"

	mat4d "github.com/flywave/go3d/float64/mat4"
)

type OperatorType uint32

const (
	OP_VOXELIZE = OperatorType(0)
	OP_COMBI    = OperatorType(1)
	OP_MASK     = OperatorType(2)
	OP_SURFACE  = OperatorType(3)
)

type Operator interface {
	Apply() error
	GetVoxelPixel() *VoxelPixel
}

type VoxelizeOperator struct {
	Operator
	mesh         *VoxelMesh
	base         *VoxelPixel
	localFeature LocalFeatureID
	precision    float32
	creator      ClipBoxCreateor
	class        GridClass
	matrix       mat4d.T
}

func (f *VoxelizeOperator) GetVoxelPixel() *VoxelPixel {
	return f.base
}

func (f *VoxelizeOperator) Apply() error {
	if f.mesh == nil || f.base == nil {
		return errors.New("op not inited")
	}

	f.base = f.mesh.SampleVoxelPixel(nil, f.localFeature, f.precision, f.creator, f.class, f.matrix)
	return nil
}

type CSGOperator struct {
	VoxelizeOperator
	target *VoxelPixel
	ctp    CompositeType
}

func (f *CSGOperator) GetVoxelPixel() *VoxelPixel {
	return f.target
}

func (f *CSGOperator) Apply() error {
	if f.mesh == nil || f.target == nil {
		return errors.New("op not inited")
	}
	mtls := f.base.GetMaterials()
	defer mtls.Free()
	f.target = f.mesh.SampleVoxelPixel(mtls, f.localFeature, f.precision, f.creator, f.class, f.matrix)
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

type CombiOperator struct {
	CSGOperator
}

type MaskOperator struct {
	CSGOperator
}

type SurfaceOperator struct {
	CSGOperator
}

func (f *SurfaceOperator) Apply() error {
	if f.mesh == nil || f.target == nil {
		return errors.New("op not inited")
	}
	mtls := f.base.GetMaterials()
	defer mtls.Free()
	f.target = f.mesh.SampleVoxelPixel(mtls, f.localFeature, f.precision, f.creator, f.class, f.matrix)
	defer f.target.Free()
	f.base.SetMaterials(mtls)
	color := f.base.ExtractColor(f.target)
	defer color.Free()
	f.base.FillColor(f.target, color)
	return nil
}

func newCSGOperator(ctp CompositeType, base *VoxelPixel, mesh *VoxelMesh, creator ClipBoxCreateor, precision float32, localFeature LocalFeatureID, class GridClass, matrix mat4d.T) *CSGOperator {
	return &CSGOperator{ctp: ctp, VoxelizeOperator: VoxelizeOperator{mesh: mesh, base: base, localFeature: localFeature, precision: precision, creator: creator, class: class, matrix: matrix}}
}

func NewOperator(tp OperatorType, base *VoxelPixel, mesh *VoxelMesh, creator ClipBoxCreateor, precision float32, localFeature LocalFeatureID, class GridClass, matrix mat4d.T) Operator {
	switch tp {
	case OP_VOXELIZE:
		return &VoxelizeOperator{mesh: mesh, base: base, localFeature: localFeature, precision: precision, creator: creator, class: class, matrix: matrix}
	case OP_COMBI:
		return &CombiOperator{CSGOperator: *newCSGOperator(CT_UNION, base, mesh, creator, precision, localFeature, class, matrix)}
	case OP_MASK:
		return &MaskOperator{CSGOperator: *newCSGOperator(CT_DIFFERENCE, base, mesh, creator, precision, localFeature, class, matrix)}
	case OP_SURFACE:
		return &SurfaceOperator{CSGOperator: *newCSGOperator(CT_UNION, base, mesh, creator, precision, localFeature, class, matrix)}
	}
	return nil
}
