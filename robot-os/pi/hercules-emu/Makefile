LIBSERIAL_SRC := \
	libserial/PosixSignalDispatcher.cpp \
	libserial/SerialPort.cpp \
	libserial/SerialStreamBuf.cc \
	libserial/SerialStream.cc \

SRC :=  $(LIBSERIAL_SRC) \
	Hercules.cpp \
	main.cpp \

LDFLAGS := -lncurses

all: hercules-emu

debug: debug-hercules-emu

hercules-emu: $(SRC)
	g++ -o $@ $(SRC) $(LDFLAGS)

debug-hercules-emu: $(SRC)
	g++ -o $@ $(SRC) $(LDFLAGS) -g

clean:
	rm -rf *~ hercules-emu debug-hercules-emu
