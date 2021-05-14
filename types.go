package vdb

// #include <stdlib.h>
import "C"

type CompositeType C.uint

const (
	CT_UNION        = CompositeType(0)
	CT_INTERSECTION = CompositeType(1)
	CT_DIFFERENCE   = CompositeType(2)
)

type GridClass int32

const (
	GC_LEVEL_SET = GridClass(0)
	GC_SURFACE   = GridClass(1)
)

type SamplerType int32

const (
	ST_POINT = SamplerType(0)
	ST_BOX   = SamplerType(1)
	ST_QUADRATIC   = SamplerType(2)
)
