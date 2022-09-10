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
ROOT = $(shell pwd)

## makerc
# This Makefile should help simplify installing & upgrading an ${ITL}OqtaDrive${NRM} daemon
# on Linux systems (MacOS may work, but has not been tested at all), and flashing
# the firmware onto the adapter. For an initial install, place this Makefile into
# the folder where ${ITL}OqtaDrive${NRM} should be installed and run:
#
#	${DIM}make install
#	make flash${NRM}
#
# The install part may take several minutes, in particular on slower systems
# such as a ${ITL}RaspberryPi${NRM}. If you want to enable the daemon as a ${DIM}systemd${NRM} service
# (Linux only), run:
#
#	${DIM}make service_on${NRM}
#
# To upgrade ${DIM}oqtactl${NRM} and the adapter to the latest release, run:
#
#	${DIM}make upgrade${NRM}
#
# To install a specific version, prefix the command with ${DIM}VERSION=x.y.z${NRM}. See below
# for other environment settings you can make. If you want to configure the adapter
# firmware, place a ${DIM}config.h${NRM} file into the ${DIM}oqtadrive${NRM} folder. Your settings will
# be picked up from there and not be overwritten by an upgrade. For the settings
# format, see the config section in ${DIM}oqtadrive.ino${NRM}.
#

## env
# You can set the following environment variables when calling ${DIM}make${NRM}:
#
#	${ITL}VERSION${NRM}		${ITL}OqtaDrive${NRM} version to install; defaults to latest release
#
#	${ITL}PORT${NRM}		serial port device to use when uploading; defaults to
#			${DIM}/dev/ttyUSB0${NRM}
#
#	${ITL}BAUD_RATE${NRM}	baud rate for serial port, in most cases there's no need
#			to set this, currently only values accepted are ${DIM}500000${NRM}
#			and ${DIM}1000000${NRM} (default); note that if you use this setting
#			you need to set the same baud rate in the config header
#			of the adapter firmware
#
#	${ITL}RESET_PIN${NRM}	reset pin number to use when adapter is connected via
#			GPIOs rather than USB; note that the pin numbering is
#			the one used by ${DIM}gpioset${NRM} and friends from the ${ITL}gpiod${NRM}
#			package, on a ${ITL}RaspberryPi${NRM} that's identical to the
#			number in the pin name, but may differ on other boards,
#			use ${DIM}gpioinfo${NRM} to find out; defaults to ${DIM}18${NRM}
#
#	${ITL}OLD_NANO=y${NRM}	set this when using a ${ITL}Nano${NRM} board with old boot loader
#			(sets ${ITL}FQBN${NRM} to ${DIM}arduino:avr:nano:cpu=atmega328old${NRM})
#
#	${ITL}SKIP_UI=y${NRM}	set this to skip installing the web UI
#
#	${ITL}REPO${NRM}		set to repo folder during install to activate repo
#
#	${ITL}FQBN${NRM}		board identifier to use when uploading; defaults to
#			${DIM}arduino:avr:nano${NRM} for the ${ITL}Nano${NRM}; for the ${ITL}Pro Mini${NRM},
#			use ${DIM}arduino:avr:pro${NRM}
#
#	${ITL}BUILD_URL${NRM}	set base URL of an alternative download site to not
#			download from ${ITL}GitHub${NRM} during install, e.g. for using
#			development builds
#
#	${ITL}VERBOSE=y${NRM}	get detailed output
#
# You can set these as environment variables in your ${DIM}~/.bashrc${NRM} to avoid typing
# them each time, or alternatively create a short shell script. Another option is
# to set them in file ${DIM}oqtadrive.makerc${NRM} residing in the same directory as this
# Makefile.
#
-include oqtadrive.makerc

##
#
# Happy Hacking!
#

VERBOSE ?=
ifeq ($(VERBOSE),y)
    $(warning ***** starting Makefile for goal(s) "$(MAKECMDGOALS)")
    $(warning ***** $(shell date))
    MAKEFLAGS += --trace
else
    MAKEFLAGS += -s
endif

VERSION ?=
PORT ?= /dev/ttyUSB0
BAUD_RATE ?= 1000000
RESET_PIN ?= 18
SKIP_UI ?= n
REPO ?=

OQTACTL := $(ROOT)/oqtactl
SKETCH_DIR := $(ROOT)/oqtadrive
SKETCH := $(SKETCH_DIR)/oqtadrive.ino
ARDUINO_DIR := $(ROOT)/arduino
ARDUINO_CLI := $(ARDUINO_DIR)/arduino-cli
OQTADRIVE_SERVICE := oqtadrive.service

FQBN ?= arduino:avr:nano
ifeq ($(OLD_NANO),y)
	FQBN = arduino:avr:nano:cpu=atmega328old
endif

BASE_URL := https://codeberg.org/xelalexv/oqtadrive/raw
BRANCH ?= master

define unattended_settings
# OqtaDrive settings
export PORT=$(PORT)
export BAUD_RATE=$(BAUD_RATE)
export OLD_NANO=$(OLD_NANO)
export RESET_PIN=$(RESET_PIN)
export FQBN=$(FQBN)
export REPO=$(REPO)

endef

export

#
#
#

.PHONY: help
help: sanity
#	show this help
#
	$(call utils, synopsis) | more


.PHONY: unattended_setup
unattended_setup: connected time_synced unattended_packages install patch_avrdude flash service_on #
	sed -Ei '/# OqtaDrive settings/,/^$$/d' ~/.bashrc
	echo "$${unattended_settings}" >> ~/.bashrc
	touch .unattended_setup_completed


.PHONY: unattended_packages
unattended_packages: #
	sudo apt-get install -y --no-install-recommends --fix-missing \
		-o DPkg::Lock::Timeout=300 \
		-o Acquire::http::Timeout=300 \
		-o Acquire::https::Timeout=300 \
		-o Acquire::ftp::Timeout=300 \
		curl jq gawk strace gpiod


.PHONY: install
install: sanity get_oqtactl get_firmware get_ui get_arduinocli
#	run full installation
#


.PHONY: remove
remove: sanity service_off
#	completely remove the installation
#
	echo -e "uninstalling components"
	rm -rf "$(SKETCH)" "$(SKETCH).org" "$(ARDUINO_DIR)" \
		./oqtactl ./hack ./ui ~/.arduino15


.PHONY: service_on
service_on: sanity
#	enable the daemon as a ${DIM}systemd${NRM} service, Linux only!; use ${DIM}PORT${NRM} to set port
#
	echo "adding daemon service"
	$(call utils, manage_service install enable start)


.PHONY: service_off
service_off: sanity
#	disable the daemon ${DIM}systemd${NRM} service, Linux only!
#
	echo "removing daemon service"
	$(call utils, manage_service stop disable remove)


.PHONY: can_upgrade
can_upgrade: #
	true


.PHONY: upgrade
upgrade: current get_oqtactl get_ui get_firmware flash
#	get ${DIM}oqtactl${NRM} binary and firmware & upload to adapter
#


.PHONY: flash
flash: sanity
#	compile & upload the firmware to the adapter
#
	echo
	$(call utils, manage_service stop)
	echo -e "\ncompiling & uploading to adapter"
	rm -f "$(SKETCH)"
	[[ ! -f "$(SKETCH_DIR)/config.h" ]] \
		|| echo -e '#include "config.h"\n' > "$(SKETCH)"
	cat "$(SKETCH).org" >> "$(SKETCH)"
	echo "port: $(PORT), FQBN: $(FQBN)"
	# FIXME: what to do if flashing of new firmware fails? need to roll back
	"$(ARDUINO_CLI)" compile --clean --verify --upload --port $(PORT) \
		--fqbn $(FQBN) "$(SKETCH)"
	$(call utils, manage_service start)
	echo


.PHONY: get_arduinocli
get_arduinocli: sanity
#	download & setup ${ITL}Arduino CLI${NRM}
#
	echo -e "\ndownloading Arduino CLI"
	mkdir -p "$(ARDUINO_DIR)"
	curl -fsSL \
		https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh \
			| PATH="$(PATH):$(ARDUINO_DIR)" BINDIR="$(ARDUINO_DIR)" sh -s 0.18.3
	echo -e "\nconfiguring Arduino CLI"
	"$(ARDUINO_CLI)" config init --overwrite
	echo -e "\nupdating index"
	"$(ARDUINO_CLI)" core update-index
	echo -e "\ninstalling core"
	"$(ARDUINO_CLI)" core install arduino:avr@1.8.3
	echo


.PHONY: patch_avrdude
patch_avrdude: sanity
#	patch ${DIM}avrdude${NRM} for serial via GPIOs; this is needed when the serial
#	connection between adapter and daemon host is done via GPIOs rather
#	than USB, e.g. in Tom Dalby's stand-alone setup (${ITL}Arduino Nano${NRM} +
#	${ITL}RaspberryPi Zero${NRM})
#
	$(call utils, patch_avrdude)


.PHONY: unpatch_avrdude
unpatch_avrdude: sanity
#	revert the changes made by ${DIM}patch_avrdude${NRM}
#
	$(call utils, unpatch_avrdude)


.PHONY: get_firmware
get_firmware: sanity
#	download adapter firmware
#
	echo -e "\ndownloading firmware"
	mkdir -p "$(SKETCH_DIR)"
	$(call utils, download_firmware)
	echo


.PHONY: get_oqtactl
get_oqtactl: sanity
#	download ${DIM}oqtactl${NRM} binary
#
	echo -e "\ndownloading oqtactl binary"
	$(call utils, download_oqtactl)
	"${OQTACTL}" version


.PHONY: get_ui
get_ui: sanity
#	download web UI assets
#
ifneq ($(SKIP_UI),y)
	echo -e "\ndownloading web UI assets"
	$(call utils, download_ui)
else
	echo -e "\nskipping UI installation, as requested"
endif


.PHONY: sanity
sanity: prereqs #
	$(call utils, sanity)


.PHONY: prereqs
prereqs: hack/devenvutil.sh #
ifeq ($(BUILD_ONLY),y)
	$(call utils, prereqs jq gawk strace) # no gpiod in build container
else
	$(call utils, prereqs jq gawk strace gpioinfo)
endif


.PHONY: current
current: hack/devenvutil.sh
#	make sure this Makefile and its tooling is at latest version
#
	echo -e "\nensuring devenvutils are up to date"
	$(call utils, assure_current hack/devenvutil.sh copy)
	[[ ! -f hack/devenvutil.sh.current ]] \
		|| mv -f hack/devenvutil.sh.current hack/devenvutil.sh


.PHONY: Makefile
Makefile: hack/devenvutil.sh #
# prevents running the check for every goal; add goals for which Makefile
# should be checked to filter
ifneq ($(filter upgrade current,$(MAKECMDGOALS)),)
	echo -e "\nensuring Makefile is up to date"
	cp -af Makefile hack/Makefile
	$(call utils, assure_current hack/Makefile)
	if [[ hack/Makefile -nt Makefile ]]; then \
		mv -f hack/Makefile Makefile; \
	else \
		rm -f hack/Makefile; \
	fi
endif


hack/devenvutil.sh: connected time_synced
	mkdir -p hack
	curl -fsSL -o hack/devenvutil.sh "$(BASE_URL)/$(BRANCH)/hack/devenvutil.sh"
	chmod +x hack/devenvutil.sh


.PHONY: connected
connected: #
	# ensure network, for unattended setup
	while ! ping -c 1 codeberg.org > /dev/null 2>&1; do \
		echo "waiting for network"; sleep 5; \
	done


.PHONY: time_synced
time_synced: #
	# ensure time synced, for unattended setup
	while ! timedatectl | grep --quiet "clock synchronized: yes"; do \
		echo "waiting for time sync"; sleep 5; \
	done


#
# helper functions
#
utils = ./hack/devenvutil.sh $(1)
