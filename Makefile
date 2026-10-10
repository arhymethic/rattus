CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O3 -DSIMULATION -Iinclude -MMD -MP
SRCS = src/main.cpp src/API.cpp src/Sensors.cpp src/Motors.cpp src/FloodFill.cpp
OBJS = $(SRCS:.cpp=.o)
DEPS = $(OBJS:.o=.d)
TARGET = sim_mouse

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $(OBJS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEPS)

test: $(TARGET)
	@echo "=== Running Championship Test Suite ==="
	python3 test/run_all_tests.py

clean:
	rm -f $(OBJS) $(DEPS) $(TARGET) sim_mouse_*

.PHONY: all clean test
