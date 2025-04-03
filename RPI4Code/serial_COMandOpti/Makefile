CXX = gcc

targets = serial_COMandOpti

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
serial_dev.o: serial_dev.c
	$(CXX) -c serial_dev.c	

serial_COMandOpti.o: serial_COMandOpti.c
	$(CXX) -c serial_COMandOpti.c

serial_COMandOpti: serial_COMandOpti.o serial_dev.o netapi.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ -lrt -lpthread

clean:
	rm -rf *.o $(targets)
shm:
	dos2unix shmdelete.sh
	chmod +x shmdelete.sh
