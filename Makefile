CC=g++
CXXFLAGS=-std=c++14 -O2

all: cnc

cnc: cnc.o
	$(CC) $(CXXFLAGS) cnc.o -o cnc

cnc.o: cnc.cpp
	$(CC) $(CXXFLAGS) -c cnc.cpp

clean:
	rm -rf *.o cnc
