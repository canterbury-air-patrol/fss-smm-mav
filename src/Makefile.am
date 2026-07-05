AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = $(ACLOCAL_FLAGS)

bin_PROGRAMS = cap-fmu

AM_CXXFLAGS=
AM_CXXFLAGS+= -pthread -Wno-address-of-packed-member
AM_CXXFLAGS+= -Wall -Wextra -Werror -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
# Use $(top_srcdir), not a build-dir-relative path: the vendored MAVLink headers
# live in the source tree, so ../mavlink only resolves in an in-tree build and
# breaks a VPATH build (e.g. make distcheck). tests/Makefile.am already does this.
AM_CXXFLAGS+= -isystem $(top_srcdir)/mavlink
AM_CXXFLAGS+= $(SANITIZER_FLAGS)

AM_LDFLAGS=
AM_LDFLAGS+= $(SANITIZER_FLAGS)

cap_fmu_SOURCES = main.cpp fmu.cpp fmu.hpp fmu-types.hpp fmu-core-types.hpp fmu-state-types.hpp ilogger.hpp event.hpp event-dispatcher.cpp event-dispatcher.hpp aircraft.hpp altitude-units.hpp util.hpp logger.cpp logger.hpp fmu-config.cpp fmu-config.hpp
cap_fmu_SOURCES += fss/fss.cpp fss/fmu-fss.hpp fss/fmu-fss-types.hpp fss/ifss.hpp fss/fss-client-ssl.cpp fss/internal.hpp fss/command-ack.hpp fss/command-ack-group.hpp
cap_fmu_SOURCES += mav/internal.hpp mav/mav.cpp mav/mav.hpp mav/mav-sys.cpp mav/mavlink.cpp mav/imav.hpp mav/mav-comms.hpp mav/mav-params.hpp mav/mission-plan.hpp mav/mode-resolve.hpp mav/battery-voltage.hpp mav/velocity.hpp mav/latlon-encoding.hpp
cap_fmu_SOURCES += smm/smm.cpp smm/smm.hpp smm/smm-types.hpp smm/ismm.hpp smm/smm-command.hpp smm/search-acquire.hpp smm/search-altitude.hpp smm/connection-state.hpp

cap_fmu_CXXFLAGS = $(AM_CXXFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
cap_fmu_CFLAGS = $(AM_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
cap_fmu_LDADD = $(JSONCPP_LIBS) $(FSS_CLIENT_SSL_LIBS) $(FSS_TRANSPORT_LIBS) $(SMM_ASSET_LIBS)
