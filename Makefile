EXECUTABLE=intrinsic_calibration

CC=g++

CFLAGS=`pkg-config opencv --cflags` `pkg-config opencv --libs`

CFLAGS+=-I./
SRC+=calibration.cpp \
	main.cpp

all:$(EXECUTABLE)

$(EXECUTABLE):$(SRC)
	$(CC) $(SRC) $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
