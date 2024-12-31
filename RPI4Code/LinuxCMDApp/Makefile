CXX=gcc

targets = LinuxCMDApp

all: $(targets)

LinuxCMDApp.o: LinuxCMDApp.c
	$(CXX) -c LinuxCMDApp.c	 

LinuxCMDApp: LinuxCMDApp.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ -lrt -lpthread
clean:
	rm -rf *.o $(targets)
  
