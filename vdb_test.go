package vdb

import "testing"

func TestRegex(t *testing.T) {
	zxystr := _parsr_file_reg.FindStringSubmatch("1_2_3.vtile")
	print(len(zxystr))
}
