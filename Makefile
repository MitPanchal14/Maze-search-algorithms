CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra
TARGET   := maze_solver
SRC      := maze_solver.cpp

.PHONY: all run clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $<

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)
