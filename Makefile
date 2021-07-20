PROG:=serial
CPP_FILES:=$(wildcard *.cpp)
OBJ_FILES:=$(patsubst %.cpp,%.o,$(CPP_FILES))
CPPFLAGS:=-O3

$(PROG): $(OBJ_FILES)
	$(CXX) -o $(PROG) $(OBJ_FILES)

%.o: %.cpp
	$(CXX) -c $< $(CPPFLAGS)

clean:
	$(RM) *.o $(PROG)
