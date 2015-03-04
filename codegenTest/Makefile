CC=g++
CFLAGS=-c -Wall
LDFLAGS=
SOURCES=*cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=planner

all: $(SOURCES) $(EXECUTABLE)
    
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm planner
