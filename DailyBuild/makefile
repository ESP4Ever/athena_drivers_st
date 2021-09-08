#this file will call sub folders makefile to run seperately

all:
	cd ../k91_bootloader/DailyBuild  && make -j8 clean && make -j8 all && cd ../../DailyBuild && \
	cd ../k91_main_source.0x08040000/DailyBuild  && make -j8 clean && make -j8 all && cd ../../DailyBuild && \
	cd ../k91_main_source.0x080A0000/DailyBuild  && make -j8 clean && make -j8 all && cd ../../DailyBuild && \
	cd ../can-tool && make -j8 clean && make -j8 all && cd ../DailyBuild && \
	cd ../touch-tool && make && cd ../DailyBuild && \
	cp ../k91_main_source.0x08040000/DailyBuild/k91_main_source.0x08040000.bin . && \
	cp ../k91_main_source.0x080A0000/DailyBuild/k91_main_source.0x080A0000.bin . && \
	cp ../can-tool/can-ota . && \
	cp ../can-tool/can-test-nusensors . && \
	cp ../can-tool/stm32_version_info . && \
	cp ../touch-tool/touch_test . && \
	mv k91_main_source.0x08040000.bin k91_main_source.0x08040000.12leds.bin && \
	mv k91_main_source.0x080A0000.bin k91_main_source.0x080A0000.12leds.bin

.PHONY: all
