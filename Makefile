# Top-level Makefile: configure and build using CMake (Unix Makefiles) so `make` works on macOS.

# You can override these on the make command line, e.g. `make BUILD_DIR=out release`
CMAKE ?= cmake
GENERATOR ?= "Unix Makefiles"
BUILD_DIR ?= build
CPU_JOBS ?= $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)

.PHONY: all debug release clean rebuild check-screen

all: release

debug:
	$(CMAKE) -B $(BUILD_DIR)/debug -G $(GENERATOR) -DCMAKE_BUILD_TYPE=Debug
	$(MAKE) -C $(BUILD_DIR)/debug -j$(CPU_JOBS)

release:
	$(CMAKE) -B $(BUILD_DIR)/release -G $(GENERATOR) -DCMAKE_BUILD_TYPE=Release
	$(MAKE) -C $(BUILD_DIR)/release -j$(CPU_JOBS)

clean:
	@echo "Removing $(BUILD_DIR) directory"
	rm -rf $(BUILD_DIR)

rebuild: clean release

# Quick checks for leftover display references
check-screen:
	@echo "Checking for Screen and HAL_LCD references..."
	@grep -R --line-number "Screen.h" source || true
	@grep -R --line-number "Screen_" source || true
	@grep -R --line-number "HAL_LCD" . || true
	@echo "Done."
