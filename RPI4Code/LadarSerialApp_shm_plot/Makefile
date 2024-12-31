CXXFLAGS=
CXX=gcc $(CXXFLAGS)

targets = ladar_server shmclient 

all: $(targets)

ladar_server.o: ladar_server.c
	$(CXX) -c ladar_server.c -lrt -lpthread

serial_dev.o: serial_dev.c
	$(CXX) -c $^

ladar_server: ladar_server.o serial_dev.o
	$(CXX) -o ladar_server  $^ -lrt -lpthread

shmclient: shmclient.c
	gcc shmclient.c -o shmclient -lrt -lpthread

clean:
	rm -rf *.o shmclient ladar_server
