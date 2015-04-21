CC=gcc
CFLAGS=-Wall -g
LDFLAGS=-lm -lgsl -lgslcblas

demo: pso.c demo.c
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

clean:
	rm -rf *.o demo
