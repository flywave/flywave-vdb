package vdb

import "testing"

func blistToRowCol(level uint32, blist []byte) (row, col uint32) {
	rowbits := []uint32{0x00, 0x00, 0x01, 0x01}
	colbits := []uint32{0x00, 0x01, 0x01, 0x00}

	row = 0
	col = 0

	for j := 0; j < int(level); j++ {
		row = (row << 1) | (rowbits[blist[j]])
		col = (col << 1) | (colbits[blist[j]])
	}
	return
}

var (
	TEST_BLIST = [QT_DEFAULT_MAX_LEVEL]byte{1, 2, 0, 0, 3, 1, 1, 3, 2, 1, 0, 3,
		3, 3, 0, 1, 1, 2, 2, 2, 3, 0, 0, 1}
	TEST_ASCII_BLIST = [QT_DEFAULT_MAX_LEVEL]byte{'1', '2', '0', '0', '3', '1', '1', '3', '2', '1', '0', '3',
		'3', '3', '0', '1', '1', '2', '2', '2', '3', '0', '0', '1'}
)

const (
	EXPECTED_BINARY = uint64(0x60D793F16AC10018)
	MASK48          = uint64(0xFFFFFFFFFFFF0000)
)

func TestTileIndexConstructors(t *testing.T) {

	if NewTileIndex().path != 0 {
		t.FailNow()
	}

	for level := uint32(0); level <= QT_DEFAULT_MAX_LEVEL; level++ {
		from_blist := NewTileIndexFromLevelAndBitList(level, TEST_BLIST[:])
		mask := uint64(MASK48 << (48 - level*2))
		expected := uint64((EXPECTED_BINARY & mask) | uint64(level))
		if from_blist.path != expected {
			t.FailNow()
		}

		from_ascii := NewTileIndexFromLevelAndBitList(level, TEST_ASCII_BLIST[:])
		if !from_ascii.Equal(from_blist) {
			t.FailNow()
		}
	}

	for level := uint32(0); level <= QT_DEFAULT_MAX_LEVEL; level++ {
		var row, col uint32
		var branchlist [32]byte
		if level > 0 {
			copy(branchlist[:], TEST_BLIST[0:level])
		}
		row, col = blistToRowCol(level, branchlist[:])
		from_lrc := NewTileIndexFromLevelAndRowCol(level, row, col)
		mask := uint64(MASK48 << (48 - level*2))
		expected := (EXPECTED_BINARY & mask) | uint64(level)
		if from_lrc.path != expected {
			t.FailNow()
		}

		qlevel, qrow, qcol := from_lrc.GetLevelRowCol()
		if qlevel != level || qrow != row || qcol != col {
			t.FailNow()
		}
	}

}

func TestTileIndexParentChild(t *testing.T) {
	for level := uint32(0); level < QT_DEFAULT_MAX_LEVEL; level++ {
		test_path := NewTileIndexFromLevelAndBitList(level, TEST_BLIST[:])
		for i := uint32(0); i < 4; i++ {
			child := test_path.Child(i)
			if child.GetLevel() != (level+uint32(1)) || !child.Parent().Equal(test_path) {
				t.FailNow()
			}
		}
	}
}
