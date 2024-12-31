run `cc -fPIC -shared sem_for_py.c -lrt -lpthread -o plot_sem.so` to genrate plot_sem.so <br>
run code on the F28 to send command to get ladar reading <br>
`make` <br>
run  `./ladar_server` <br>
run `python3 plot_shm.py` <br>
