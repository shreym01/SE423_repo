CXX = gcc

CXXFLAGS = -lws2_32 -lwsock32

targets = TCPClientToESP8266

all: $(targets)		

TCPClientToESP8266.o: TCPClientToESP8266.c
	$(CXX) $(CXXFLAGS) -c TCPClientToESP8266.c	 

TCPClientToESP8266: TCPClientToESP8266.o
	$(CXX) -o $@.exe $^ $(CXXFLAGS) $(INCLUDES)  

clean:
	rm -rf *.o $(targets)
  
