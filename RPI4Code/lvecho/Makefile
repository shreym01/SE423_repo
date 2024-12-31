CXX=gcc

targets = LVecho

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c

LVecho.o: LVecho.c
	$(CXX) -c LVecho.c	 

LVecho: netapi.o LVecho.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 
clean:
	rm -rf *.o $(targets)
  
