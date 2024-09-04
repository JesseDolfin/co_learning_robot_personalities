On machine A we run roscore (this is the machine advertised as 192.180.1.5 so that the arm properly connects)
On machine A we export the variable ROS_IP (this variable tells roscore on which IP to advertise)
```sh
export ROS_MASTER_URI=http://<Machine_A_IP>:11311
export ROS_IP=<Machine_A_IP>
```

On machine B we need to let ros know where to look for a rosmaster to do this we set the ROS_MASTER_URI variable:
```sh
export ROS_MASTER_URI=http://<Machine_A_IP>:11311
export ROS_IP=<Machine_B_IP>
```

After exporting the variables start roscore (alone or via roslaunch) on machine A in the same terminal you exported the variables in.  
Then run your applications on machine B and they should recognise the rosmaster on machine A.  
To test if they correctly recognise rosmaster A you can type rostopic list on machine B. If topics show up you know the connection is OK.  

NOTE: Make sure that machine B ipv4 address is somewhere in the range 192.180.1.x!, I am unsure if the subnet mask needs to be set but to be safe set it to: 255.255.255.0  

**Practical use case example:**  
step 1: set the IP of machine A (the desktop) to: 192.180.1.5, netmask: 255.255.255.0, gateway: 192.190.1.1 (KUKA preset on the desktop in the lab)  
step 2: set the IP om machine B (the laptop)  to: 192.180.1.10, netmask: 255.255.255.0.  
step 3: run the commands.  
Machine A:
```sh
export ROS_MASTER_URI=http://192.180.1.5:11311
export ROS_IP=192.180.1.5
```
Machine B:
```sh
export ROS_MASTER_URI=http://192.180.1.5:11311
export ROS_IP=192.180.1.10
```
