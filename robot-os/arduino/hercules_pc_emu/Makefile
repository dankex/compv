SRC :=  hercules_pc_emu.cpp

all: hercules_pc_emu

LDFLAGS := -std=c++11

hercules_pc_emu: $(SRC)
	g++ -o $@ $(SRC) $(LDFLAGS)

clean:
	rm -rf *~ *.o hercules_pc_emu
