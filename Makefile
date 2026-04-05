CXX          := g++
CXXFLAGS     := -std=c++17 -O2 -Wall -Wextra
TARGET       := maze_solver
SRC          := maze_solver.cpp
TEST_TARGET  := maze_solver_tests
TEST_SRC     := maze_solver_tests.cpp

.PHONY: all run test clean

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $<

run: $(TARGET)
	./$(TARGET)

# The test binary #includes maze_solver.cpp directly, so it depends on both files.
$(TEST_TARGET): $(TEST_SRC) $(SRC)
	$(CXX) $(CXXFLAGS) -o $@ $<

test: $(TEST_TARGET)
	./$(TEST_TARGET)

clean:
	rm -f $(TARGET) $(TEST_TARGET)
