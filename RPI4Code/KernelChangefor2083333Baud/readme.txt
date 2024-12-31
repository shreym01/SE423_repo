This file is located in drivers\tty\serial  in the kernel source files.  
search for 2083333 to find the modifications. 
This is needed because this is the baud rate that the F28379D can communicate at.  It cannot be set to 2000000 