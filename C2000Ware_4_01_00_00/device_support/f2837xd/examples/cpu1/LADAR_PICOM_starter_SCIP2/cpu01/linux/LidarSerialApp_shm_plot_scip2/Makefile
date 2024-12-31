CXXFLAGS=
CXX=gcc $(CXXFLAGS)

targets = lidar_server shmclient 

all: $(targets)

lidar_server.o: lidar_server.c
	$(CXX) -c lidar_server.c -lrt -lpthread

serial_dev.o: serial_dev.c
	$(CXX) -c $^

lidar_server: lidar_server.o serial_dev.o
	$(CXX) -o lidar_server  $^ -lrt -lpthread #problrm with static flag

shmclient: shmclient.c
	gcc shmclient.c -o shmclient -lrt -lpthread

clean:
	rm -rf *.o shmclient lidar_server
