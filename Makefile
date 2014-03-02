CXXFLAGS += -Wall -pedantic -std=c++11 -fpic -Iinclude -Os

SRC:=Printer.cpp

INC:=$(SRC:%.cpp=include/%.h)
SRC:=$(SRC:%=src/%)
OBJ:=$(SRC:%.cpp=%.o)

all: bin/librepetier.so bin/librepetier.a

doc:
	doxygen

bin/librepetier.so: $(OBJ)
	$(CXX) $(CXXFLAGS) -shared -o $@ $<
bin/librepetier.a: $(OBJ)
	ar rcs $@ $<

$(OBJ): src/%.o: $(INC)

clean:
	rm -f $(OBJ)
	rm -rf doc/html doc/latex

.PHONY: clean doc
