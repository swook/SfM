CC=clang++
CCOPT=-O3 -m64 -std=c++11

CLIBS= -lboost_system -lboost_filesystem -lboost_program_options # Boost
CLIBS+=`pkg-config --libs opencv` # OpenCV

### START VLFEAT
COBJS= -I../lib/vlfeat  # 3rd party libs
UNAME_S=$(shell uname -s)
ifeq ($(UNAME_S),Linux)
	CLIBS+=-L../lib/vlfeat/bin/glnxa64
endif
ifeq ($(UNAME_S),Darwin)
	CLIBS+=-L../lib/vlfeat/bin/maci64
endif
CLIBS+=-lvl
### END VLFEAT


all: main

main: main.cpp
		$(CC) $(CCOPT) $(COBJS) $< $(CLIBS) -o $@

clean:
	rm -f main

