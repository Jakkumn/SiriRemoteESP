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

.PHONY: help build flash monitor fm clean fullclean erase menuconfig size set-esp32 set-esp32s3

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

set-esp32: ## Target WROOM-32 / original ESP32
	$(IDF) set-target esp32

set-esp32s3: ## Target ESP32-S3
	$(IDF) set-target esp32s3
