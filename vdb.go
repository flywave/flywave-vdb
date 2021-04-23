package vdb

// #include <stdlib.h>
// #include "vdb_api.h"
// #cgo CFLAGS: -I ./lib
// #cgo CXXFLAGS: -I ./lib
// #cgo linux LDFLAGS:  -L ./lib -L /usr/lib/x86_64-linux-gnu -Wl,--start-group  -lstdc++ -lm -pthread -ldl -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd -Wl,--end-group
// #cgo windows LDFLAGS: -L ./lib　-lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd
// #cgo darwin LDFLAGS: -L　./lib -lamd -lamd_l -lbin_packer -lblosc -lboost_chrono_internal -lboost_date_time_internal -lboost_filesystem_internal -lboost_iostreams_internal -lboost_log_internal -lboost_program_options_internal -lboost_regex_internal -lboost_serialization_internal -lboost_system_internal -lboost_test_internal -lboost_thread_internal -lboost_timer_internal -lcamd -lcamd_l -lccolamd -lccolamd_l -lceres -lcholmod -lcholmod_l -lcolamd -lcolamd_l -lembree -lexpat -lfmt -lfontconfig -lfreetype -lGKlib -liconv -licudata -licudt -licui18n -licutools -licuuc -lIlmImf -ljasper -ljpeg -llerc -llexers -llz4 -llzma -lmath -lmetis -lOpenImageIO -lOpenImageIO_Util -lopenvdb -lpng -lsimd -lskylight -lsnappy -lspqr -lSuiteSparse_config -lsys -ltasking -ltbb -ltbbmalloc -ltbbmalloc_proxy -ltiff -lvdb -lwebp -lxatlas -lxml2 -lzlib -lzstd
import "C"

type Grid struct {
	m *C.struct__vdb_grid_t
}

func NewGrid() *Grid {
	return &Grid{
		m: C.vdb_create(),
	}
}

func (m *Grid) Free() {
	C.vdb_free(m.m)
	m.m = nil
}
