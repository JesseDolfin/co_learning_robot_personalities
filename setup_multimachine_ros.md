On machine A we run roscore (this is the machine advertised as 192.180.1.5 so that the arm properly connects)
On machine A we export the variable ROS_IP (this variable tells roscore on which IP to advertise)
```sh
export ROS_IP=192.180.1.5
```

On machine B we need to let ros know where to look for a rosmaster to do this we set the ROS_MASTER_URI variable:
```sh
export ROS_MASTER_URI=http://192.180.1.5:11311
```

After exporting the variables start roscore (alone or via roslaunch) on machine A in the same terminal you exported the variables in
Then run your applications on machine B and they should recognise the rosmaster on machine A
To test if they correctly recognise rosmaster A you can type rostopic list on machine B. If topics show up you know the connection is OK

NOTE: Make sure that machine B ipv4 address is somewhere in the range 192.180.1.x!, I am unsure if the subnet mask needs to be set but to be safe set it to: 255.255.255.0

Practical use case: The worker PC is machine A, which runs roscore, the real-sense software, and the detection package. The laptop is machine B, which runs the rest of the packages. 
