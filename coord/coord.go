package coord

// #include <stdlib.h>
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
import "C"
import (
	"fmt"
	"math"
	"unsafe"

	vec3d "github.com/flywave/go3d/float64/vec3"
)

var (
	// Zero holds a zero vector.
	Zero = T{0, 0, 0}

	// Min holds a min vector.
	MinVal = T{-math.MaxInt32, -math.MaxInt32, -math.MaxInt32}

	// Max holds a max vector.
	MaxVal = T{+math.MaxInt32, +math.MaxInt32, +math.MaxInt32}
)

type T [3]int32

func RoundFrom(xyz vec3d.T) *T {
	return &T{int32(math.Round(xyz[0])), int32(math.Round(xyz[1])), int32(math.Round(xyz[2]))}
}

func FloorFrom(xyz vec3d.T) *T {
	return &T{int32(math.Floor(xyz[0])), int32(math.Floor(xyz[1])), int32(math.Floor(xyz[2]))}
}

func CeilFrom(xyz vec3d.T) *T {
	return &T{int32(math.Ceil(xyz[0])), int32(math.Ceil(xyz[1])), int32(math.Ceil(xyz[2]))}
}

func Parse(s string) (r T, err error) {
	_, err = fmt.Sscan(s, &r[0], &r[1], &r[2])
	return r, err
}

// String formats T as string. See also Parse().
func (vec *T) String() string {
	return fmt.Sprint(vec[0], vec[1], vec[2])
}

// Rows returns the number of rows of the vector.
func (vec *T) Rows() int {
	return 3
}

// Cols returns the number of columns of the vector.
func (vec *T) Cols() int {
	return 1
}

// Size returns the number elements of the vector.
func (vec *T) Size() int {
	return 3
}

// Slice returns the elements of the vector as slice.
func (vec *T) Slice() []int32 {
	return vec[:]
}

// Slice returns the elements of the vector as slice.
func (vec *T) CSlice() *C.int {
	return (*C.int)(unsafe.Pointer(&vec[0]))
}

// Get returns one element of the vector.
func (vec *T) Get(col, row int) int32 {
	return vec[row]
}

// IsZero checks if all elements of the vector are zero.
func (vec *T) IsZero() bool {
	return vec[0] == 0 && vec[1] == 0 && vec[2] == 0
}

// Length returns the length of the vector.
// See also LengthSqr and Normalize.
func (vec *T) Length() int32 {
	return int32(math.Sqrt(float64(vec.LengthSqr())))
}

// LengthSqr returns the squared length of the vector.
// See also Length and Normalize.
func (vec *T) LengthSqr() int32 {
	return vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]
}

// Scale multiplies all element of the vector by f and returns vec.
func (vec *T) Scale(f float32) *T {
	vec[0] = int32(math.Floor(float64(vec[0]) * float64(f)))
	vec[1] = int32(math.Floor(float64(vec[1]) * float64(f)))
	vec[2] = int32(math.Floor(float64(vec[2]) * float64(f)))
	return vec
}

// Scaled returns a copy of vec with all elements multiplies by f.
func (vec *T) Scaled(f float32) T {
	return T{int32(math.Floor(float64(vec[0]) * float64(f))), int32(math.Floor(float64(vec[1]) * float64(f))), int32(math.Floor(float64(vec[2]) * float64(f)))}
}

// Invert inverts the vector.
func (vec *T) Invert() *T {
	vec[0] = -vec[0]
	vec[1] = -vec[1]
	vec[2] = -vec[2]
	return vec
}

// Inverted returns an inverted copy of the vector.
func (vec *T) Inverted() T {
	return T{-vec[0], -vec[1], -vec[2]}
}

func Abs(n int32) int32 {
	if n < 0 {
		return -n
	}
	return n
}

// Abs sets every component of the vector to its absolute value.
func (vec *T) Abs() *T {
	vec[0] = Abs(vec[0])
	vec[1] = Abs(vec[1])
	vec[2] = Abs(vec[2])
	return vec
}

// Absed returns a copy of the vector containing the absolute values.
func (vec *T) Absed() T {
	return T{Abs(vec[0]), Abs(vec[1]), Abs(vec[2])}
}

// Add adds another vector to vec.
func (vec *T) Add(v *T) *T {
	vec[0] += v[0]
	vec[1] += v[1]
	vec[2] += v[2]
	return vec
}

// Sub subtracts another vector from vec.
func (vec *T) Sub(v *T) *T {
	vec[0] -= v[0]
	vec[1] -= v[1]
	vec[2] -= v[2]
	return vec
}

// Mul multiplies the components of the vector with the respective components of v.
func (vec *T) Mul(v *T) *T {
	vec[0] *= v[0]
	vec[1] *= v[1]
	vec[2] *= v[2]
	return vec
}

// Add returns the sum of two vectors.
func Add(a, b *T) T {
	return T{a[0] + b[0], a[1] + b[1], a[2] + b[2]}
}

// Sub returns the difference of two vectors.
func Sub(a, b *T) T {
	return T{a[0] - b[0], a[1] - b[1], a[2] - b[2]}
}

// Mul returns the component wise product of two vectors.
func Mul(a, b *T) T {
	return T{a[0] * b[0], a[1] * b[1], a[2] * b[2]}
}

// Min returns the component wise minimum of two vectors.
func Min(a, b *T) T {
	min := *a
	if b[0] < min[0] {
		min[0] = b[0]
	}
	if b[1] < min[1] {
		min[1] = b[1]
	}
	if b[2] < min[2] {
		min[2] = b[2]
	}
	return min
}

// Max returns the component wise maximum of two vectors.
func Max(a, b *T) T {
	max := *a
	if b[0] > max[0] {
		max[0] = b[0]
	}
	if b[1] > max[1] {
		max[1] = b[1]
	}
	if b[2] > max[2] {
		max[2] = b[2]
	}
	return max
}

// Clamp clamps the vector's components to be in the range of min to max.
func (vec *T) Clamp(min, max *T) *T {
	for i := range vec {
		if vec[i] < min[i] {
			vec[i] = min[i]
		} else if vec[i] > max[i] {
			vec[i] = max[i]
		}
	}
	return vec
}

// Clamped returns a copy of the vector with the components clamped to be in the range of min to max.
func (vec *T) Clamped(min, max *T) T {
	result := *vec
	result.Clamp(min, max)
	return result
}
