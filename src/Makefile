CC=clang++
CCOPT=-O3 -m64 -std=c++11

CLIBS= -lboost_system -lboost_filesystem -lboost_program_options # Boost
CLIBS+=`pkg-config --libs opencv` # OpenCV
CLIBS+= -I../lib/vlfeat # vlfeat

all: main

main: main.cpp
	$(CC) $(CCOPT) $(CLIBS) $(CVLIB) $< -o $@

clean:
	rm -f main

