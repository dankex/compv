LIBSERIAL_SRC := \
	libserial/PosixSignalDispatcher.cpp \
	libserial/SerialPort.cpp \
	libserial/SerialStreamBuf.cc \
	libserial/SerialStream.cc \

SRC :=  $(LIBSERIAL_SRC) \
	Hercules.cpp \
	Keyboard.cpp \
	main.cpp \

LDFLAGS := -lncurses

all: sample-hercules-remote

debug: debug-sample-hercules-remote

sample-hercules-remote: $(SRC)
	g++ -o $@ $(SRC) $(LDFLAGS)

debug-sample-hercules-remote: $(SRC)
	g++ -o $@ $(SRC) $(LDFLAGS) -g

clean:
	rm -rf *~ sample-hercules-remote debug-sample-hercules-remote
