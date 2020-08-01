GCC?=gcc
GXX?=g++
CFLAGS?=
CXXFLAGS?= -std=c++20
LDFLAGS?=
CXXFLAGS+= -Wall -Wextra -Wshadow -Wnon-virtual-dtor -Wduplicated-cond -Wduplicated-branches -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++
# Required to handle mavlink
#CXXFLAGS+= -pedantic -Wconversion -Wsign-conversion 
CXXFLAGS+= -Wno-address-of-packed-member
# Ends
CXXFLAGS+= -Imavlink
CXXFLAGS+= -pthread
LDFLAGS+= -pthread

CXXFLAGS+= `pkg-config --cflags fss`
LDFLAGS+= `pkg-config --libs fss-client fss-transport`

CXXFLAGS+= -ggdb

CPP_CODE=main.cpp fmu.cpp \
		$(addprefix fss/, fss.cpp fss-client.cpp) \
		$(addprefix smm/, smm.cpp) \
		$(addprefix mav/, mav.cpp mavlink.cpp mav-sys.cpp)
C_CODE=
OBJS=
OBJS+=$(C_CODE:.c=.o)
OBJS+=$(CPP_CODE:.cpp=.o)

all: fmu


%.o: %.c
	$(GCC) -c -o $(@) $(<) $(CFLAGS)

%.o: %.cpp
	$(GXX) -c -o $(@) $(<) $(CXXFLAGS)


fmu: $(OBJS)
	$(GXX) -o $(@) $(OBJS) $(LDFLAGS)
