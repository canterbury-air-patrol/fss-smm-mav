GCC?=gcc
GPP?=g++
CFLAGS?=
CXXFLAGS?= -Wall -Wextra -Wshadow -Wnon-virtual-dtor -pedantic -Wsign-conversion -Wduplicated-cond -Wduplicated-branches -Wconversion -Wmisleading-indentation -Wlogical-op -Wformat=2 -Weffc++

CPP_CODE=main.cpp fmu.cpp $(addprefix fss/, fss.cpp) $(addprefix smm/, smm.cpp) $(addprefix mav/, mav.cpp)
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
