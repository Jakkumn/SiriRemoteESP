SHELL := /bin/bash

# Override on the command line: `make flash PORT=/dev/cu.usbserial-1234`
PORT ?=
PORT_ARG := $(if $(PORT),-p $(PORT),)

IDF := idf.py
ifeq ($(shell command -v idf.py 2>/dev/null),)
  ifneq ($(IDF_PATH),)
    IDF := source "$(IDF_PATH)/export.sh" >/dev/null && idf.py
  endif
endif

.DEFAULT_GOAL := help

.PHONY: help build flash monitor fm clean fullclean erase menuconfig size

help: ## Show available targets
	@echo "wake_minimal — diagnostic BLE-only build"
	@echo
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ {printf "  \033[36m%-14s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)

build: ## Compile firmware
	$(IDF) build

flash: ## Flash firmware (override with PORT=/dev/cu.usbserial-XXXX)
	$(IDF) $(PORT_ARG) flash

monitor: ## Open serial monitor (Ctrl+] to exit)
	$(IDF) $(PORT_ARG) monitor

fm: ## Flash then monitor
	$(IDF) $(PORT_ARG) flash monitor

clean: ## Remove build artefacts
	$(IDF) clean

fullclean: ## Wipe build/ entirely
	$(IDF) fullclean

erase: ## Erase flash (wipes NVS bond — forces re-pairing on next boot)
	$(IDF) $(PORT_ARG) erase-flash

menuconfig: ## Interactive sdkconfig editor
	$(IDF) menuconfig

size: ## Show firmware size breakdown
	$(IDF) size
