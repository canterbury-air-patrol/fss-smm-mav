AUTOMAKE_OPTIONS = subdir-objects
ACLOCAL_AMFLAGS = $(ACLOCAL_FLAGS)

bin_PROGRAMS = cap-fmu

AM_CXXFLAGS=
AM_CXXFLAGS+= -pthread -std=c++11 -Wno-address-of-packed-member
AM_CXXFLAGS+= -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
AM_CXXFLAGS+= -I ../mavlink

cap_fmu_SOURCES = main.cpp fmu.cpp fmu.hpp fmu-types.hpp
cap_fmu_SOURCES += fss/fss.cpp fss/fmu-fss.hpp fss/fmu-fss-types.hpp fss/fss-client.cpp fss/internal.hpp
cap_fmu_SOURCES += mav/internal.hpp mav/mav.cpp mav/mav.hpp mav/mav-sys.cpp mav/mavlink.cpp
cap_fmu_SOURCES += smm/smm.cpp smm/smm.hpp smm/smm-types.hpp

cap_fmu_CXXFLAGS = $(AM_CXXFLAGS) $(FSS_CLIENT_CFLAGS) $(SMM_ASSET_CFLAGS)
cap_fmu_CFLAGS = $(AM_CFLAGS) $(FSS_CLIENT_CFLAGS) $(SMM_ASSET_CFLAGS)
cap_fmu_LDADD = $(JSONCPP_LIBS) $(FSS_CLIENT_LIBS) $(FSS_TRANSPORT_LIBS) $(SMM_ASSET_LIBS)
