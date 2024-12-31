CXX = gcc

targets = DSPUARTcomm blob_new

all: $(targets)

netapi.o: netapi.c 
	$(CXX) -c netapi.c
	
serial_dev.o: serial_dev.c
	$(CXX) -c serial_dev.c	

DSPUARTcomm.o: DSPUARTcomm.c
	$(CXX) -c DSPUARTcomm.c	 

DSPUARTcomm: netapi.o serial_dev.o DSPUARTcomm.o
	$(CXX) -o $@ $(CXXFLAGS) $(INCLUDES) $^ 

blob_new.o:blob_new.cpp
	g++ `pkg-config --cflags --libs opencv4` -c blob_new.cpp

blob_new: blob_new.o serial_dev.o
	g++ -o $@ $(CXXFLAGS) $(INCLUDES) $^ `pkg-config --cflags --libs opencv4`

clean:
	rm -rf *.o $(targets)
  
