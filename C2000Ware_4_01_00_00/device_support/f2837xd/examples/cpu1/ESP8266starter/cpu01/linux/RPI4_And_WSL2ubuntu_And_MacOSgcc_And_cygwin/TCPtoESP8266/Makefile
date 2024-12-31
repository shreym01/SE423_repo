CXX = gcc

targets = TCPClientToESP8266

all: $(targets)

DieWithMessage.o: DieWithMessage.c 
	$(CXX) -c DieWithMessage.c	

TCPClientUtility.o: TCPClientUtility.c 
	$(CXX) -c TCPClientUtility.c	

TCPClientToESP8266.o: TCPClientToESP8266.c
	$(CXX) -c TCPClientToESP8266.c	 

TCPClientToESP8266: TCPClientUtility.o DieWithMessage.o TCPClientToESP8266.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 

clean:
	rm -rf *.o $(targets)
  
