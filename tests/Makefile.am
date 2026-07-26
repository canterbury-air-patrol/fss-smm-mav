AUTOMAKE_OPTIONS = subdir-objects

AM_CXXFLAGS = -pthread -Wno-address-of-packed-member
AM_CXXFLAGS += -Wall -Wextra -Werror -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
AM_CXXFLAGS += $(SANITIZER_FLAGS)

AM_LDFLAGS =
AM_LDFLAGS += $(SANITIZER_FLAGS)

check_PROGRAMS = fmu_test mav_io_test fss_test
TESTS = $(check_PROGRAMS)

fmu_test_SOURCES = fmu_test.cpp $(top_srcdir)/src/fmu.cpp $(top_srcdir)/src/fmu-config.cpp $(top_srcdir)/src/logger.cpp $(top_srcdir)/src/event-dispatcher.cpp $(top_srcdir)/src/mav/mav-sys.cpp
fmu_test_CPPFLAGS = -I$(top_srcdir)/src -isystem $(top_srcdir)/mavlink $(CATCH2_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
fmu_test_CXXFLAGS = $(AM_CXXFLAGS)
fmu_test_LDADD = $(CATCH2_LIBS) $(JSONCPP_LIBS)

# Integration tests that drive the real mav_connection over a loopback socket.
# This links the actual MAVLink/SMM networking objects (and libsmmasset), unlike
# fmu_test which stays a fast, pure unit suite over the decision logic.
mav_io_test_SOURCES = mav_io_test.cpp $(top_srcdir)/src/mav/mavlink.cpp $(top_srcdir)/src/mav/mav.cpp $(top_srcdir)/src/mav/mav-sys.cpp $(top_srcdir)/src/smm/smm.cpp
mav_io_test_CPPFLAGS = -I$(top_srcdir)/src -isystem $(top_srcdir)/mavlink $(CATCH2_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
mav_io_test_CXXFLAGS = $(AM_CXXFLAGS)
mav_io_test_LDADD = $(CATCH2_LIBS) $(SMM_ASSET_LIBS) $(FSS_CLIENT_SSL_LIBS) $(JSONCPP_LIBS)

# Tests over the real FSS send worker (todo/98): its task queue, the
# position-report coalescing in FSS::enqueue, and workerLoop's dispatch. Links
# the real FSS/fss_client_ssl objects, so it needs libfss-client-ssl -- but no
# peer and no TLS: the FSS under test is built from a nonexistent config file,
# which leaves a client with no servers, and the send seams are overridden.
fss_test_SOURCES = fss_test.cpp $(top_srcdir)/src/fss/fss.cpp $(top_srcdir)/src/fss/fss-client-ssl.cpp
fss_test_CPPFLAGS = -I$(top_srcdir)/src -isystem $(top_srcdir)/mavlink $(CATCH2_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
fss_test_CXXFLAGS = $(AM_CXXFLAGS)
fss_test_LDADD = $(CATCH2_LIBS) $(FSS_CLIENT_SSL_LIBS) $(JSONCPP_LIBS)
