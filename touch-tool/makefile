TARGETS = touch_test

CFLAGS += -Wall

all:$(TARGETS)

$(TARGETS):touch_test.cpp InputEventReader.cpp TouchSensorHandler.cpp
	$(CC_PREFIX)g++ $(CFLAGS) $^ -o $@ -lpthread

clean:
	rm -rf *.o
