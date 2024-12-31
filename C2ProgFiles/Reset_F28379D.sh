#GPIO2 of  PI4 connected to F28379D GPIO72  SCI boot pin when low and then reset F28379D in SCI boot
gpio -g mode 2 out
gpio -g write 2 1
#GPIO3 of PI4 connected to F28379D RESET
gpio -g mode 3 out
gpio -g write 3 0
sleep 1
gpio -g write 3 1
sleep 1
gpio -g mode 2 in
gpio -g mode 3 in
raspi-gpio get 0-15
