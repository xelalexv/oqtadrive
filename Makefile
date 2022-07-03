#
#   OqtaDrive - Sinclair Microdrive emulator
#   Copyright (c) 2021, Alexander Vollschwitz
#
#   This file is part of OqtaDrive.
#
#   OqtaDrive is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   OqtaDrive is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with OqtaDrive. If not, see <http://www.gnu.org/licenses/>.
#

.DEFAULT_GOAL := help
SHELL = /bin/bash

REPO = oqtadrive
OQTADRIVE_RELEASE = 0.3.1
OQTADRIVE_VERSION := $(shell git describe --always --tag --dirty)

ROOT = $(shell pwd)
SKETCH_DIR := $(ROOT)/arduino
SKETCH := $(SKETCH_DIR)/oqtadrive.ino
BUILD_OUTPUT = _build
BINARIES = $(BUILD_OUTPUT)/bin
ISOLATED_PKG = $(BUILD_OUTPUT)/pkg
ISOLATED_CACHE = $(BUILD_OUTPUT)/cache
UI_BASE = $(ROOT)/ui/web

ARDUINO_CLI_IMAGE = xelalex/oqtadrive-arduino-cli
GO_IMAGE = golang:1.18.1-buster@sha256:f1e97d64a50f4c2b4fa61211f5206e636a54f992a047d192d6d068fbcd1946c3
JSMINIFY_IMAGE = tdewolff/minify@sha256:946e1146b79c3299626893f163e247ccd2bb4f1646e4537f1b0623bcd2023c33

## env
# You can set the following environment variables when calling make:
#
#	${ITL}VERBOSE=y${NRM}	get detailed output
#
#	${ITL}ISOLATED=y${NRM}	When using this with the build target, the build will be isolated in the
#			sense that local caches such as ${DIM}\${GOPATH}/pkg${NRM} and ${DIM}~/.cache${NRM} will not be
#			mounted into the container. Instead, according folders underneath the
#			configured build folder are used. These folders are removed when running
#			${DIM}make clean${NRM}. That way you can force a clean build/test, where all
#			dependencies are retrieved & built inside the container.
#
#	${ITL}CROSS=y${NRM}		When using this with the build target, ${ITL}MacOS${NRM} & ${ITL}Windows${NRM} binaries
#			are also built.
#

VERBOSE ?=
ifeq ($(VERBOSE),y)
    MAKEFLAGS += --trace
else
    MAKEFLAGS += -s
endif

ISOLATED ?=
ifeq ($(ISOLATED),y)
    CACHE_VOLS = -v $(shell pwd)/$(ISOLATED_PKG):/go/pkg -v $(shell pwd)/$(ISOLATED_CACHE):/.cache
else
    CACHE_VOLS = -v $(GOPATH)/pkg:/go/pkg -v /home/$(USER)/.cache:/.cache
endif

PORT ?=
ifeq ($(PORT),)
	ARDUINO_CLI_ARGS = --clean --verify
	TTY_VOL =
else
	ARDUINO_CLI_ARGS = --clean --upload --port $(PORT)
	TTY_VOL = -v $(PORT):$(PORT)
endif

FQBN ?= arduino:avr:nano

UNATT_ROOT := /home/alex/tmp
UNATT_SRC := hack/unattended/raspberrypi

export

#
#
#

.PHONY: help
help:
#	show this help
#
	$(call utils, synopsis) | more


.PHONY: run
run:
#	run the daemon with Go on host; set ${DIM}DEVICE${NRM} to serial device
#
	go run cmd/oqtad/main.go serve --device=$(DEVICE)


.PHONY: imgarduino
imgarduino:
#	build the ${ITL}Arduino CLI${NRM} image required for compiling adapter firmware
#
ifeq ($(shell docker images --digests --quiet $(ARDUINO_CLI_IMAGE)),)
	echo "building Arduino CLI image..."
	docker build --build-arg BRANCH=$(BRANCH) -t $(ARDUINO_CLI_IMAGE) \
		-f ./hack/arduino-cli.Dockerfile .
	echo "image build done"
endif


.PHONY: imgsetup_raspberrypi
imgsetup_raspberrypi:
#	build the unattended setup image for standalone with ${ITL}RaspberryPi${NRM}
#
	# TODO getting partition start: fdisk -l -o Start --bytes
	#
	# FIXME file ownership
	# boot
	touch $(UNATT_BOOT)/ssh
	cp $(UNATT_SRC)/wpa_supplicant.conf $(UNATT_BOOT)/
	echo "pi:$(shell echo 'oqtadrive' | openssl passwd -6 -stdin)" \
		> $(UNATT_BOOT)/userconf.txt
	grep --silent "enable_uart=1" $(UNATT_BOOT)/config.txt \
		|| { \
			sed -i '/^[all]$$/d' $(UNATT_BOOT)/config.txt; \
			echo -e "[all]\nenable_uart=1" >> $(UNATT_BOOT)/config.txt; \
		}
	sed -i 's/console=serial0,115200 //g' $(UNATT_BOOT)/cmdline.txt
	# root
	grep --silent "# OqtaDrive bootstrapper" $(UNATT_ROOT)/etc/rc.local \
		|| { \
			sed -i '/^exit 0$$/d' $(UNATT_ROOT)/etc/rc.local; \
			cat $(UNATT_SRC)/rc.local >> $(UNATT_ROOT)/etc/rc.local; \
		}
	cp hack/Makefile $(UNATT_ROOT)/home/pi/
	mkdir $(UNATT_ROOT)/home/pi/repo
	mkdir $(UNATT_ROOT)/home/pi/oqtadrive
	cp $(UNATT_SRC)/config.h $(UNATT_ROOT)/home/pi/oqtadrive/


.PHONY: firmware
firmware: imgarduino
#	compile the adapter firmware; set ${DIM}FQBN${NRM} to select board type (defaults to ${DIM}arduino:avr:nano${NRM});
#	set ${DIM}PORT${NRM} to port of connected adapter to actually upload instead of just compiling, e.g.:
# 
#	    ${DIM}FQBN=arduino:avr:pro PORT=/dev/ttyUSB0 make firmware${NRM}
#
	docker run --rm -ti --privileged -v "$(SKETCH_DIR):/oqtadrive/oqtadrive" \
		$(TTY_VOL) $(ARDUINO_CLI_IMAGE) \
		./arduino/arduino-cli compile $(ARDUINO_CLI_ARGS) \
			--fqbn $(FQBN) /oqtadrive/oqtadrive


.PHONY: build
build: prep ui
#	build the ${DIM}oqtactl${NRM} binary and pack UI artifacts
#
	rm -f $(BINARIES)/oqtactl
	$(call utils, build_binary oqtactl linux amd64 keep)
ifneq ($(CROSS),)
	$(call utils, build_binary oqtactl linux 386)
	$(call utils, build_binary oqtactl linux arm)
	$(call utils, build_binary oqtactl linux arm64)
	$(call utils, build_binary oqtactl darwin amd64)
	$(call utils, build_binary oqtactl darwin arm64)
	$(call utils, build_binary oqtactl windows amd64)
endif
	cd $(BINARIES); sha256sum oqtactl*_*.zip ui.zip > checksums.txt

	[[ -L $(BINARIES)/oqtactl ]] || \
		( cd $(BINARIES); ln -s oqtactl_$(OQTADRIVE_RELEASE)_linux_amd64 oqtactl )


.PHONY: ui
ui: prep
#	pack the UI artifacts
#
	$(call utils, minify_js drives.js files.js repo.js config.js main.js)
	zip -r $(BINARIES)/ui.zip ui -x 'ui/web/js/oqta/*'


.PHONY: prep
prep: #
	mkdir -p $(BINARIES) $(ISOLATED_PKG) $(ISOLATED_CACHE)


.PHONY: clean
clean:
#	clean up
#
	[[ ! -d $(BUILD_OUTPUT) ]] || chmod -R u+w $(BUILD_OUTPUT)
	rm -rf $(BUILD_OUTPUT)/*


#
# helper functions
#
utils = ./hack/devenvutil.sh $(1)
