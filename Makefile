SHELL := /bin/bash

# Override on the command line: `make flash PORT=/dev/cu.usbserial-1234`
# When unset, idf.py auto-detects a serial port.
PORT ?=
PORT_ARG := $(if $(PORT),-p $(PORT),)

# Resolve how to invoke idf.py.
#   - If idf.py is already on PATH (export.sh sourced in shell), use it directly.
#   - Else, if IDF_PATH is set, source export.sh per-command so each recipe works standalone.
#   - Else, fall through and let the missing command produce its own error.
IDF := idf.py
ifeq ($(shell command -v idf.py 2>/dev/null),)
  ifneq ($(IDF_PATH),)
    IDF := source "$(IDF_PATH)/export.sh" >/dev/null && idf.py
  endif
endif

.DEFAULT_GOAL := help

TEST_BUILD := tests/build

.PHONY: help build flash monitor fm clean fullclean erase menuconfig size test test-clean lint set-esp32 set-esp32s3

help: ## Show available targets
	@echo "siri-bridge — ESP-IDF wrapper"
	@echo
	@echo "Prereq: ESP-IDF installed. Either source \$$IDF_PATH/export.sh in your shell,"
	@echo "or set IDF_PATH in your env and each target will source it for you."
	@echo
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

build: ## Compile firmware
	$(IDF) build

flash: ## Flash firmware (override port with PORT=/dev/cu.usbserial-XXXX)
	$(IDF) $(PORT_ARG) flash

monitor: ## Open serial monitor (Ctrl+] to exit)
	$(IDF) $(PORT_ARG) monitor

fm: ## Flash then monitor — typical dev loop
	$(IDF) $(PORT_ARG) flash monitor

clean: ## Remove build artefacts
	$(IDF) clean

fullclean: ## Wipe build/ entirely (forces full rebuild)
	$(IDF) fullclean

erase: ## Erase flash — wipes NVS bonds; use after bad pairing state
	$(IDF) $(PORT_ARG) erase-flash

menuconfig: ## Interactive sdkconfig editor
	$(IDF) menuconfig

size: ## Show firmware size breakdown
	$(IDF) size

test: ## Build and run host-side unit tests (no ESP-IDF, no hardware)
	@cmake -S tests/host -B $(TEST_BUILD) -DCMAKE_BUILD_TYPE=Debug > /dev/null
	@cmake --build $(TEST_BUILD) > /dev/null
	@ctest --test-dir $(TEST_BUILD) --output-on-failure

test-clean: ## Remove host test build artefacts
	@rm -rf $(TEST_BUILD)

lint: ## Check C/H formatting against .clang-format
	@command -v clang-format >/dev/null 2>&1 || { \
	    echo "clang-format not found. Install with: brew install clang-format"; \
	    exit 1; }
	@# peer.c + esp_central.h come from the upstream NimBLE blecent example
	@# and use a different brace style; keep them out of the lint set so we
	@# don't drift away from the upstream source we periodically diff against.
	@find components/siri_ble components/siri_audio components/button_pulse \
	    components/report_decoder components/mqtt_entity main tests/host \
	    \( -name '*.c' -o -name '*.h' \) -not -name 'peer.c' -not -name 'esp_central.h' | \
	    xargs clang-format --dry-run -Werror

set-esp32: ## Target WROOM-32 / original ESP32
	$(IDF) set-target esp32

set-esp32s3: ## Target ESP32-S3
	$(IDF) set-target esp32s3
