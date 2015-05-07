CC := g++
UNAME := $(shell uname)
ifeq ($(UNAME), Linux)
	CFLAGS := -o poke -O3 -std=c++11 -lGL -lglut
else
	CFLAGS := -o poke -O3 -std=c++11 -framework GLUT -framework OpenGL -framework Cocoa
endif

all:
	$(CC) main.cpp $(CFLAGS)

clean:
	rm -rf poke* *.gch
