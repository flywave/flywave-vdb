package vdb

// #include <stdlib.h>
import "C"

type CompositeType C.uint

const (
	CT_UNION        = CompositeType(0)
	CT_INTERSECTION = CompositeType(1)
	CT_DIFFERENCE   = CompositeType(2)
)
