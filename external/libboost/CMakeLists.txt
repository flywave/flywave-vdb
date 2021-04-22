add_definitions(-Wno-unused-variable)
add_definitions(-Wno-deprecated-declarations)

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}/boost_1_67_0")

add_library(boost_chrono_internal STATIC
boost_1_67_0/libs/chrono/src/chrono.cpp
boost_1_67_0/libs/chrono/src/process_cpu_clocks.cpp
boost_1_67_0/libs/chrono/src/thread_clock.cpp
)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_chrono_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_date_time_internal STATIC
boost_1_67_0/libs/date_time/src/gregorian/date_generators.cpp
boost_1_67_0/libs/date_time/src/gregorian/greg_month.cpp
boost_1_67_0/libs/date_time/src/gregorian/greg_weekday.cpp
)

add_library(boost_serialization_internal STATIC
boost_1_67_0/libs/serialization/src/basic_iarchive.cpp
boost_1_67_0/libs/serialization/src/basic_oarchive.cpp
boost_1_67_0/libs/serialization/src/basic_pointer_iserializer.cpp
boost_1_67_0/libs/serialization/src/basic_iarchive.cpp
boost_1_67_0/libs/serialization/src/basic_pointer_oserializer.cpp
boost_1_67_0/libs/serialization/src/binary_oarchive.cpp
boost_1_67_0/libs/serialization/src/binary_iarchive.cpp
boost_1_67_0/libs/serialization/src/text_iarchive.cpp
boost_1_67_0/libs/serialization/src/text_oarchive.cpp
boost_1_67_0/libs/serialization/src/xml_oarchive.cpp
boost_1_67_0/libs/serialization/src/xml_iarchive.cpp
boost_1_67_0/libs/serialization/src/extended_type_info.cpp
boost_1_67_0/libs/serialization/src/extended_type_info_typeid.cpp
boost_1_67_0/libs/serialization/src/basic_oserializer.cpp
boost_1_67_0/libs/serialization/src/basic_iserializer.cpp
boost_1_67_0/libs/serialization/src/void_cast.cpp
boost_1_67_0/libs/serialization/src/archive_exception.cpp
boost_1_67_0/libs/serialization/src/basic_text_iprimitive.cpp
boost_1_67_0/libs/serialization/src/basic_archive.cpp
boost_1_67_0/libs/serialization/src/basic_text_oprimitive.cpp
boost_1_67_0/libs/serialization/src/basic_serializer_map.cpp
boost_1_67_0/libs/serialization/src/xml_archive_exception.cpp
boost_1_67_0/libs/serialization/src/utf8_codecvt_facet.cpp
boost_1_67_0/libs/serialization/src/basic_xml_archive.cpp
boost_1_67_0/libs/serialization/src/xml_wgrammar.cpp
boost_1_67_0/libs/serialization/src/xml_grammar.cpp 
)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_serialization_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_definitions(-DBOOST_CHRONO_HEADER_ONLY)

add_library(boost_timer_internal STATIC
boost_1_67_0/libs/timer/src/cpu_timer.cpp
)


IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_timer_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_date_time_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_program_options_internal STATIC
boost_1_67_0/libs/program_options/src/cmdline.cpp
boost_1_67_0/libs/program_options/src/config_file.cpp
boost_1_67_0/libs/program_options/src/convert.cpp
boost_1_67_0/libs/program_options/src/options_description.cpp
boost_1_67_0/libs/program_options/src/parsers.cpp
boost_1_67_0/libs/program_options/src/positional_options.cpp
boost_1_67_0/libs/program_options/src/split.cpp
boost_1_67_0/libs/program_options/src/utf8_codecvt_facet.cpp
boost_1_67_0/libs/program_options/src/value_semantic.cpp
boost_1_67_0/libs/program_options/src/variables_map.cpp
boost_1_67_0/libs/program_options/src/winmain.cpp)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_program_options_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_filesystem_internal STATIC
boost_1_67_0/libs/filesystem/src/codecvt_error_category.cpp
boost_1_67_0/libs/filesystem/src/operations.cpp
boost_1_67_0/libs/filesystem/src/path.cpp
boost_1_67_0/libs/filesystem/src/path_traits.cpp
boost_1_67_0/libs/filesystem/src/portability.cpp
boost_1_67_0/libs/filesystem/src/unique_path.cpp
boost_1_67_0/libs/filesystem/src/utf8_codecvt_facet.cpp
boost_1_67_0/libs/filesystem/src/windows_file_codecvt.cpp)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_filesystem_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_regex_internal STATIC
  boost_1_67_0/libs/regex/src/c_regex_traits.cpp
  boost_1_67_0/libs/regex/src/cpp_regex_traits.cpp
  boost_1_67_0/libs/regex/src/cregex.cpp
  boost_1_67_0/libs/regex/src/fileiter.cpp
  boost_1_67_0/libs/regex/src/icu.cpp
  boost_1_67_0/libs/regex/src/instances.cpp
  boost_1_67_0/libs/regex/src/posix_api.cpp
  boost_1_67_0/libs/regex/src/regex.cpp
  boost_1_67_0/libs/regex/src/regex_debug.cpp
  boost_1_67_0/libs/regex/src/regex_raw_buffer.cpp
  boost_1_67_0/libs/regex/src/regex_traits_defaults.cpp
  boost_1_67_0/libs/regex/src/static_mutex.cpp
  boost_1_67_0/libs/regex/src/usinstances.cpp
  boost_1_67_0/libs/regex/src/w32_regex_traits.cpp
  boost_1_67_0/libs/regex/src/wc_regex_traits.cpp
  boost_1_67_0/libs/regex/src/wide_posix_api.cpp
  boost_1_67_0/libs/regex/src/winstances.cpp
)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_regex_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_system_internal STATIC
boost_1_67_0/libs/system/src/error_code.cpp)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_system_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_test_internal STATIC
boost_1_67_0/libs/test/src/compiler_log_formatter.cpp
boost_1_67_0/libs/test/src/cpp_main.cpp
boost_1_67_0/libs/test/src/debug.cpp
boost_1_67_0/libs/test/src/decorator.cpp
boost_1_67_0/libs/test/src/execution_monitor.cpp
boost_1_67_0/libs/test/src/framework.cpp
boost_1_67_0/libs/test/src/junit_log_formatter.cpp
boost_1_67_0/libs/test/src/plain_report_formatter.cpp
boost_1_67_0/libs/test/src/progress_monitor.cpp
boost_1_67_0/libs/test/src/results_collector.cpp
boost_1_67_0/libs/test/src/results_reporter.cpp
boost_1_67_0/libs/test/src/test_main.cpp
boost_1_67_0/libs/test/src/test_tools.cpp
boost_1_67_0/libs/test/src/test_tree.cpp
boost_1_67_0/libs/test/src/unit_test_log.cpp
boost_1_67_0/libs/test/src/unit_test_main.cpp
boost_1_67_0/libs/test/src/unit_test_monitor.cpp
boost_1_67_0/libs/test/src/unit_test_parameters.cpp
boost_1_67_0/libs/test/src/xml_log_formatter.cpp
boost_1_67_0/libs/test/src/xml_report_formatter.cpp)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_test_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

IF(UNIX)
add_library(boost_thread_internal STATIC
boost_1_67_0/libs/thread/src/pthread/once_atomic.cpp
boost_1_67_0/libs/thread/src/pthread/thread.cpp
boost_1_67_0/libs/thread/src/future.cpp
boost_1_67_0/libs/thread/src/tss_null.cpp
)
ELSE()
add_library(boost_thread_internal STATIC
boost_1_67_0/libs/thread/src/win32/thread_primitives.cpp
boost_1_67_0/libs/thread/src/win32/thread.cpp
boost_1_67_0/libs/thread/src/win32/tss_dll.cpp
boost_1_67_0/libs/thread/src/win32/tss_pe.cpp
boost_1_67_0/libs/thread/src/future.cpp
boost_1_67_0/libs/thread/src/tss_null.cpp
)
ENDIF()

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_thread_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../bzip2/")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../lzma/api/")

add_library(boost_iostreams_internal STATIC
boost_1_67_0/libs/iostreams/src/bzip2.cpp
boost_1_67_0/libs/iostreams/src/file_descriptor.cpp
boost_1_67_0/libs/iostreams/src/gzip.cpp
boost_1_67_0/libs/iostreams/src/mapped_file.cpp
boost_1_67_0/libs/iostreams/src/zlib.cpp
boost_1_67_0/libs/iostreams/src/lzma.cpp
)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_iostreams_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)

add_library(boost_log_internal STATIC
    boost_1_67_0/libs/log/attribute_name.cpp
		boost_1_67_0/libs/log/attribute_set.cpp
		boost_1_67_0/libs/log/attribute_value_set.cpp
		boost_1_67_0/libs/log/code_conversion.cpp
		boost_1_67_0/libs/log/core.cpp
		boost_1_67_0/libs/log/date_time_format_parser.cpp
		boost_1_67_0/libs/log/default_attribute_names.cpp
		boost_1_67_0/libs/log/default_sink.cpp
		boost_1_67_0/libs/log/dump.cpp
    boost_1_67_0/libs/log/event.cpp
    boost_1_67_0/libs/log/exceptions.cpp
    boost_1_67_0/libs/log/format_parser.cpp
    boost_1_67_0/libs/log/global_logger_storage.cpp
    boost_1_67_0/libs/log/named_scope_format_parser.cpp
    boost_1_67_0/libs/log/named_scope.cpp
    boost_1_67_0/libs/log/once_block.cpp
    boost_1_67_0/libs/log/permissions.cpp
    boost_1_67_0/libs/log/process_id.cpp
    boost_1_67_0/libs/log/process_name.cpp
    boost_1_67_0/libs/log/record_ostream.cpp
    boost_1_67_0/libs/log/severity_level.cpp
    boost_1_67_0/libs/log/spirit_encoding.cpp
    boost_1_67_0/libs/log/syslog_backend.cpp
    boost_1_67_0/libs/log/text_file_backend.cpp
    boost_1_67_0/libs/log/text_multifile_backend.cpp
    boost_1_67_0/libs/log/text_ostream_backend.cpp
    boost_1_67_0/libs/log/thread_id.cpp
    boost_1_67_0/libs/log/thread_specific.cpp
    boost_1_67_0/libs/log/threadsafe_queue.cpp
    boost_1_67_0/libs/log/timer.cpp
    boost_1_67_0/libs/log/timestamp.cpp
    boost_1_67_0/libs/log/trivial.cpp
    boost_1_67_0/libs/log/unhandled_exception_count.cpp
)

IF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
  SET_TARGET_PROPERTIES(boost_log_internal PROPERTIES FOLDER external/boost)
ENDIF(FLYWAVE_ENABLE_SOLUTION_FOLDERS)
