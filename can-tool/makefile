
TARGETS1 = can-ota

OBJS1  = can-ota.o

TARGETS2 = can-test-nusensors

OBJS2  = can-test-nusensors.o

CFLAGS += -Wall

all: $(TARGETS1) $(TARGETS2)

$(TARGETS1): $(OBJS1)
	$(CC) $(CFLAGS) $(LDFLAGS) $< $(LDLIBS) -o $@ -lpthread

$(TARGETS2): $(OBJS2)
	$(CC) $(CFLAGS) $(LDFLAGS) $< $(LDLIBS) -o $@ -lpthread

%.o:%.c
	$(CC) -c $(CFLAGS) $< -o $@ -lpthread
clean:
	rm -rf $(TARGETS1) $(TARGETS2) *.o
