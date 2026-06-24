AUTOMAKE_OPTIONS = subdir-objects

AM_CXXFLAGS = -pthread -Wno-address-of-packed-member
AM_CXXFLAGS += -Wall -Wextra -Werror -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
AM_CXXFLAGS += $(SANITIZER_FLAGS)

AM_LDFLAGS = $(SANITIZER_FLAGS)

check_PROGRAMS = fmu_test
TESTS = $(check_PROGRAMS)

fmu_test_SOURCES = fmu_test.cpp $(top_srcdir)/src/fmu.cpp $(top_srcdir)/src/fmu-config.cpp $(top_srcdir)/src/mav/mav-sys.cpp
fmu_test_CPPFLAGS = -I$(top_srcdir)/src -isystem $(top_srcdir)/mavlink $(CATCH2_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
fmu_test_CXXFLAGS = $(AM_CXXFLAGS)
fmu_test_LDADD = $(CATCH2_LIBS) $(JSONCPP_LIBS)
