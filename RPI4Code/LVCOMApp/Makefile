CXX=gcc

targets = LVCOMApp

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c

LVCOMApp.o: LVCOMApp.c
	$(CXX) -c LVCOMApp.c	 

LVCOMApp: netapi.o LVCOMApp.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ -lrt -lpthread
clean:
	rm -rf *.o $(targets)
  
