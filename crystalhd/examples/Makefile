CPP := g++
LFLAGS += -lcrystalhd -lpthread

% : %.cpp
	$(CPP) $(LFLAGS) -o $@ $<

all: hellobcm mpeg2test

clean:
	rm hellobcm mpeg2test

