CXX ?= c++
CXXFLAGS ?= -std=c++17 -O3 -Wall -Wextra -I. -I./third_party
TARGET = arithmetic_planner
SRCS = src/main.cpp

all: $(TARGET)

$(TARGET): $(SRCS) include/types.hpp include/embedding.hpp include/planner.hpp include/engine.hpp
	$(CXX) $(CXXFLAGS) $(SRCS) -o $(TARGET)

run: $(TARGET)
	./$(TARGET) arithmetic_xy_reach_target.json --vectors --compatibility --compose

clean:
	rm -f $(TARGET) transition_execution.txt visualizer_manifest.json

.PHONY: all run clean
