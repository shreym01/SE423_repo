run `cc -fPIC -shared sem_for_py.c -lrt -lpthread -o plot_sem.so` to genrate plot_sem.so <br>
run code on the F28 to send command to get Lidar reading <br>
`make` <br>
run  `./lidar_server` <br>
run `python3 plot_shm.py` <br>
