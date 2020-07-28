GCC?=gcc
GPP?=g++
CFLAGS?=
CXXFLAGS?=

CPP_CODE=main.cpp
C_CODE=
OBJS=
OBJS+=$(C_CODE:.c=.o)
OBJS+=$(CPP_CODE:.cpp=.o)

all: fmu


%.o: %.c
	$(GCC) -c -o $(@) $(<) $(CFLAGS)

%.o: %.cpp
	$(GPP) -c -o $(@) $(<) $(CXXFLAGS)


fmu: $(OBJS)
	$(GPP) -o $(@) $(OBJS)
