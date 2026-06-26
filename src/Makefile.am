AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = $(ACLOCAL_FLAGS)

bin_PROGRAMS = cap-fmu

AM_CXXFLAGS=
AM_CXXFLAGS+= -pthread -Wno-address-of-packed-member
AM_CXXFLAGS+= -Wall -Wextra -Werror -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
AM_CXXFLAGS+= -isystem ../mavlink
AM_CXXFLAGS+= $(SANITIZER_FLAGS)

AM_LDFLAGS=
AM_LDFLAGS+= $(SANITIZER_FLAGS)

cap_fmu_SOURCES = main.cpp fmu.cpp fmu.hpp fmu-types.hpp event.hpp aircraft.hpp logger.cpp logger.hpp fmu-config.cpp fmu-config.hpp
cap_fmu_SOURCES += fss/fss.cpp fss/fmu-fss.hpp fss/fmu-fss-types.hpp fss/fss-client-ssl.cpp fss/internal.hpp
cap_fmu_SOURCES += mav/internal.hpp mav/mav.cpp mav/mav.hpp mav/mav-sys.cpp mav/mavlink.cpp mav/mav-comms.hpp mav/mission-plan.hpp mav/mode-resolve.hpp mav/battery-voltage.hpp
cap_fmu_SOURCES += smm/smm.cpp smm/smm.hpp smm/smm-types.hpp smm/search-altitude.hpp smm/connection-state.hpp

cap_fmu_CXXFLAGS = $(AM_CXXFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
cap_fmu_CFLAGS = $(AM_CFLAGS) $(SMM_ASSET_CFLAGS) $(FSS_CLIENT_SSL_CFLAGS) $(JSONCPP_CFLAGS)
cap_fmu_LDADD = $(JSONCPP_LIBS) $(FSS_CLIENT_SSL_LIBS) $(FSS_TRANSPORT_LIBS) $(SMM_ASSET_LIBS)
