.DEFAULT_GOAL := build

.PHONY: build test

.PHONY: no_targets list

SHELL = /usr/bin/env bash

#############
# Variables # 
#############
# Git
CI_BUILD_CONCURRECY ?= 40
CI_COMMIT_SHORT_SHA ?= no_sha
CI_COMMIT_REF_NAME ?= master
CI_PROJECT_DIR ?= $(shell pwd)
GIT_CLONE_COMMAND ?= git clone --recurse-submodules -j$(CI_BUILD_CONCURRECY)

# Athena athena_drivers_st build variables
CYBERDOG_PATH ?= /opt/ros2/cyberdog
CROSS_ROOT_PATH ?= /mnt/sdcard
CROSS_HOME_PATH ?= $(CROSS_ROOT_PATH)/home
CROSS_SETUP_BASH_FILE ?= $(CROSS_ROOT_PATH)/opt/ros2/foxy/local_setup.bash
CROSS_CYBERDOG_PATH ?= $(CROSS_ROOT_PATH)$(CYBERDOG_PATH)
CROSS_MAKE_PATH ?= $(CI_PROJECT_DIR)/DailyBuild

REPO_NAME := $(lastword $(subst /, ,$(CI_PROJECT_DIR)))
PACKAGE_SRC_PATH ?= src
ATHENA_DRIVERS_DEBS_SUFFIX ?= *.deb
PACKAGE_DEB_DIR ?= $(CI_PROJECT_DIR)/athena_deb_repos
PACKAGE_DRIVERS_NAME ?= athena_drivers_st_deb
PACKAGE_DRIVERS_DEBS_PATH ?= $(PACKAGE_DRIVERS_NAME)/src
PACKAGE_DRIVERS_DEBS_VERSION_PATH ?= $(PACKAGE_DRIVERS_DEBS_PATH)/DEBIAN/control
PACKAGE_OTA_NAME ?= athena_ota_server_deb
PACKAGE_FACTORY_NAME ?= athena_factory_deb
BUILD_DRIVERS_BIN_DIR ?= $(CI_PROJECT_DIR)/$(PACKAGE_DRIVERS_NAME)/$(PACKAGE_SRC_PATH)/usr/bin
BUILD_OTA_BIN_DIR ?= $(CI_PROJECT_DIR)/$(PACKAGE_OTA_NAME)/$(PACKAGE_SRC_PATH)/usr/bin
BUILD_FACTORY_BIN_DIR ?= $(CI_PROJECT_DIR)/$(PACKAGE_FACTORY_NAME)/$(PACKAGE_SRC_PATH)/usr/bin

BUILD_DRIVERS_FILE_LIST ?= k91* stm32_version_info
BUILD_OTA_FILE_LIST ?= can-ota
BUILD_FACTORY_FILE_LIST ?= can-test-nusensors
BUILD_DEFAULT_VERDION_PREFIX ?= 1.

BUILD_FILE_SRC ?= $(PACKAGE_DRIVERS_NAME)_src-$(CI_COMMIT_SHORT_SHA).tgz
BUILD_FILE_NAME ?= $(PACKAGE_DRIVERS_NAME)-$(CI_COMMIT_SHORT_SHA).tgz

# Athena athena_drivers_st FDS variables
FDS_ATHENA_DRIVERS_DEB_URL ?= https://cnbj2m-fds.api.xiaomi.net/mirp-public/cyberdog-tools/ota_debs/athena_sensor_deb.tgz

#################
# BUILD TARGET  #
#################
build: touch-files build-files package-files build-deb

touch-files: clean
	@curl -s $(FDS_ATHENA_DRIVERS_DEB_URL) | tar zx && \
		mv $(firstword $(subst .tgz, ,$(lastword $(subst /, ,$(FDS_ATHENA_DRIVERS_DEB_URL))))) $(PACKAGE_DRIVERS_NAME) && \
		mkdir -p \
		$(BUILD_DRIVERS_BIN_DIR) \
		$(BUILD_OTA_BIN_DIR) \
		$(BUILD_FACTORY_BIN_DIR) \
		$(PACKAGE_DEB_DIR)

build-files:
	@cd $(CROSS_MAKE_PATH) && \
		make -j$(CI_BUILD_CONCURRECY)

package-files: package-copy-files

build-deb:
	@if [[ $(CI_COMMIT_REF_NAME) =~ ^[0-9].* ]]; then \
    	sed -i 's/Version: .*/Version: $(CI_COMMIT_REF_NAME)/' $(PACKAGE_DRIVERS_DEBS_VERSION_PATH); \
	else \
		sed -i 's/Version: .*/Version: $(BUILD_DEFAULT_VERDION_PREFIX)$(CI_COMMIT_REF_NAME)/' $(PACKAGE_DRIVERS_DEBS_VERSION_PATH); \
	fi && \
	dpkg -b $(PACKAGE_DRIVERS_DEBS_PATH) .

upload-files:
	@tar czf $(BUILD_FILE_NAME) $(ATHENA_DRIVERS_DEBS_SUFFIX) && \
		$(FDS_COMMAND_UPLOAD) $(BUILD_FILE_NAME) $(FDS_URL_PREFIX)/$(REPO_NAME)/$(CI_COMMIT_REF_NAME)/$(BUILD_FILE_NAME) && \
		tar czf $(BUILD_FILE_SRC) $(PACKAGE_DRIVERS_NAME) $(PACKAGE_OTA_NAME) $(PACKAGE_FACTORY_NAME) && \
		$(FDS_COMMAND_UPLOAD) $(BUILD_FILE_SRC) $(FDS_URL_PREFIX)/$(REPO_NAME)/$(CI_COMMIT_REF_NAME)/$(BUILD_FILE_SRC) 

#################                                                      
# TEST TARGET   #
#################
test:


################
# Minor Targets#
################
no_targets:
list:
	@sh -c "$(MAKE) -p no_targets | awk -F':' '/^[a-zA-Z0-9][^\$$#\/\\t=]*:([^=]|$$)/ {split(\$$1,A,/ /);for(i in A)print A[i]}' | grep -v '__\$$' \
		| grep -v 'make'| grep -v 'list'| grep -v 'no_targets' |grep -v 'Makefile' | sort | uniq"

dpkg-drivers-file:
	@cd $(CI_PROJECT_DIR)/$(PACKAGE_DRIVERS_NAME) && \
		chmod -R o-w $(PACKAGE_SRC_PATH) && \
		dpkg -b $(PACKAGE_SRC_PATH) . && \
		cp -arp $(PACKAGE_DEB_LIST) $(PACKAGE_DEB_DIR)

dpkg-ota-file:
	@cd $(CI_PROJECT_DIR)/$(PACKAGE_OTA_NAME) && \
		chmod -R o-w $(PACKAGE_SRC_PATH) && \
		dpkg -b $(PACKAGE_SRC_PATH) . && \
		cp -arp $(PACKAGE_DEB_LIST) $(PACKAGE_DEB_DIR)

dpkg-factory-file:
	@cd $(CI_PROJECT_DIR)/$(PACKAGE_FACTORY_NAME) && \
		chmod -R o-w $(PACKAGE_SRC_PATH) && \
		dpkg -b $(PACKAGE_SRC_PATH) . && \
		cp -arp $(PACKAGE_DEB_LIST) $(PACKAGE_DEB_DIR)

package-copy-files:
	@cd $(CROSS_MAKE_PATH) && \
		cp -arp $(BUILD_DRIVERS_FILE_LIST) $(BUILD_DRIVERS_BIN_DIR) && \
		cp -arp $(BUILD_OTA_FILE_LIST) $(BUILD_OTA_BIN_DIR) && \
		cp -arp $(BUILD_FACTORY_FILE_LIST) $(BUILD_FACTORY_BIN_DIR)

package-dpkg-files: dpkg-drivers-file dpkg-ota-file dpkg-factory-file

clean:
	@rm -rf \
		$(CI_PROJECT_DIR)/$(PACKAGE_DRIVERS_NAME) \
		$(CI_PROJECT_DIR)/$(PACKAGE_OTA_NAME) \
		$(CI_PROJECT_DIR)/$(PACKAGE_FACTORY_NAME) \
		$(PACKAGE_DEB_DIR)