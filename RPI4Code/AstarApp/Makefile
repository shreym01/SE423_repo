CXX = gcc

targets = AstarApp 

all: $(targets)

pQueue.o: pQueue.c
	$(CXX) -c pQueue.c

AstarApp.o: AstarApp.c
	$(CXX) -c AstarApp.c

AstarApp: AstarApp.o pQueue.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ -lrt -lpthread -lm

clean:
	rm -rf *.o $(targets)
  
