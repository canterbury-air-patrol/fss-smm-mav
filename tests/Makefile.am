AUTOMAKE_OPTIONS = subdir-objects

AM_CXXFLAGS = -pthread -Wno-address-of-packed-member
AM_CXXFLAGS += -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++

check_PROGRAMS = fmu_test
TESTS = $(check_PROGRAMS)

fmu_test_SOURCES = fmu_test.cpp $(top_srcdir)/src/fmu.cpp
fmu_test_CPPFLAGS = -I$(top_srcdir)/src $(CATCH2_CFLAGS)
fmu_test_CXXFLAGS = $(AM_CXXFLAGS)
fmu_test_LDADD = $(CATCH2_LIBS)
