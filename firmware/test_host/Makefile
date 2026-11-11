# Host build of the AgroBot control core: unit tests + shared library for the simulator.
CORE := ../lib/agro_core
SRCS := $(CORE)/kinematics.cpp $(CORE)/drive.cpp $(CORE)/arm_controller.cpp $(CORE)/mission.cpp \
        $(CORE)/robot.cpp $(CORE)/protocol.cpp
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion -Wno-float-conversion -I$(CORE)
BUILD := build

.PHONY: all test lib clean sketch-check
all: test lib

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/test_core: test_core.cpp $(SRCS) $(wildcard $(CORE)/*.h) | $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp $(SRCS) -lm

test: $(BUILD)/test_core
	./$(BUILD)/test_core

# shared library loaded by simulation/agro_fw.py (firmware-in-the-loop)
lib: $(BUILD)/libagrocore.so
$(BUILD)/libagrocore.so: $(SRCS) $(CORE)/c_api.cpp $(wildcard $(CORE)/*.h) | $(BUILD)
	$(CXX) $(CXXFLAGS) -fPIC -shared -o $@ $(SRCS) $(CORE)/c_api.cpp -lm

# compile the Arduino sketch against stub headers: catches syntax and type errors without the ESP32 toolchain
sketch-check: | $(BUILD)
	$(CXX) -std=c++17 -fsyntax-only -Wall -Wextra -Imocks -I$(CORE) -I../src -DAGRO_HOST_CHECK \
	    -x c++ ../src/main.cpp ../src/hal_esp32.cpp ../src/web_server.cpp

clean:
	rm -rf $(BUILD)
