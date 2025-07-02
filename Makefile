PROGS:=serial
CPP_FILES:=$(wildcard *.cpp)
OBJ_FILES:=$(patsubst %.cpp,%.o,$(CPP_FILES))
DEP_FILES:=deps.d
CPPFLAGS:=-O3 -MMD -MF $(DEP_FILES)

all: $(PROGS)

-include $(DEP_FILES)

$(PROGS): $(OBJ_FILES)
	$(CXX) -o $(PROGS) $(OBJ_FILES) $(CPPFLAGS)

%.o: %.cpp
	$(CXX) -c $< $(CPPFLAGS)

clean:
	$(RM) $(DEP_FILES) $(OBJ_FILES) $(PROGS)
