<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->

<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <img src="https://www.kuka.com/-/media/kuka-corporate/images/products/robots/cta-images/lbr-iiwa.png?rev=-1&w=767&hash=78E7DD844A27074AFD67AEF17C72078A" alt="KUKA arm" width="200"/>

<h3 align="center">Co-learning with robot personalities for KUKA iiwa7</h3>

  <p align="center">
    <a href="https://github.com/JesseDolfin/Co-Learning-KUKA-RL/README.md"><strong>About the project</strong></a>
    <br />
    <a href="https://github.com/JesseDolfin/Co-Learning-KUKA-RL"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/JesseDolfin/Co-Learning-KUKA-RL">View Demo</a>
    ·
    <a href="https://github.com/JesseDolfin/Co-Learning-KUKA-RL/issues">Report Bug</a>
    ·
    <a href="https://github.com/JesseDolfin/Co-Learning-KUKA-RL/issues">Request Feature</a>
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-project">About The Project</a></li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About The Project
[![Product Name Screen Shot][product-screenshot]](https://preview.free3d.com/img/2015/06/2205987029685109856/qyz27g5f.jpg)

<!-- GETTING STARTED -->
## Getting Started
This package is tested with Ubuntu 20.2 and ROS noetic; it uses Python version 3.8. Other configurations have not been tested, and compatibility may vary.

### Prerequisites
1. Install the combined robot hw package:
   ```sh
   sudo apt install ros-noetic-combined-robot-hw
   ```
2. Install the RealSense SDK 2.0, to do this follow their [installation instructions](https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide)

### Installation
1. Create a workspace folder and go into it, then create a src folder
   ```sh
   mkdir <WORKSPACE_NAME> && cd <WORKSPACE_NAME>
   mkdir src
   ```
2. Install the [iiwa_ros](https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/iiwa_ros) package from the KUKA iiwa CoR lab's repository, following their installation instructions **except for the compilation instructions**. This means that all the cloned repos go into ```<WORKSPACE_NAME>```.
3. Go into the source folder and clone the iiwa_ros repo:
  ```sh
  cd src
  git clone git@gitlab.tudelft.nl:kuka-iiwa-7-cor-lab/iiwa_ros.git
  ```
4. Clone the impedance controller and checkout a specific branch that allows compatibility with the cor_tud iiwa_ros package:
   ```sh
   git clone git@gitlab.tudelft.nl:nickymol/iiwa_impedance_control.git
   cd iiwa_impedance_control
   git checkout cor_tud_compatibility
   cd ..
   ``
5. Clone the qb_hand repositories:
   ```sh
   git clone --recurse-submodules https://bitbucket.org/qbrobotics/qbdevice-ros.git
   git clone https://bitbucket.org/qbrobotics/qbhand-ros.git
   ```
6. Lastly, clone this repository and build the workspace:
  ```sh
  git clone https://github.com/JesseDolfin/co_learning_robot_personalities.git
  cd ..
  catkin_make
  ```
7. Install additional dependencies:
   ```sh pip install numpy==1.21 python-dateutil==2.8.2 mediapipe pyrealsense2 ultralytics gymnasium pygame pyserial  ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage
After installing and sourcing the software the simulation may be started using the following roslaunch command:
```sh
roslaunch co_learning_controllers bringup.launch allow_nodes:=false
```
This will start the simulation without any of the nodes.
If you want to start this on the real robot, run the command with the prefix `simulation:=false`:
```sh
roslaunch co_learning_controllers co_learning_test_setup.launch simulation:=false
```
It is possible to selectively turn off nodes , make sure that you set ```allow_nodes:=true``` or omit the option entirely as the default value is true. The full set of node control parameters are:
```xml
<arg name="allow_nodes" default="true"/>
<arg name="control_node" default="true"/>
<arg name="secondary_task" default="true"/>
<arg name="rviz" default="false"/>
<arg name="detection" default="true"/>
<arg name="qb_hand" default="true"/>
```
All the nodes are stand-alone except for the control_node, which needs at least the detection node to run. If you pass the parameter ``fake=true``` to the control node, you can run it without the qb_hand, as it does not depend on any of the other nodes.

### The full setup is as follows
Follow the [Setup multimachine ROS guide](setup_multimachine_ros.md) 
Then on machine A source the workspace in a terminal and run the following:
```sh
roslaunch co_learning_controllers bringup.launch simulation:=false control_node:=false secondary_task:=false
```

In another terminal source the workspace and run the following:
```sh
rosrun co_learning_controllers control_node.py
```

On machine B connect the ethernet cable from the switch to the machine, source the workspace and run the following:
```sh
rosrun co_learning_secondary_task secondary_task.py 
```

After all of this, start the FRIOverlay app in automatic mode on the tablet.  
The robot arm should now be moving, to start the experiment simply start the simulation on machine B.

_For more examples, please refer to the [Documentation](https://google.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
## Contributing
Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion to improve this, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement."
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License
Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTACT -->
## Contact
Jesse Dolfin - jesse.dolfin@gmail.com

Project Link: [https://github.com/JesseDolfin/Co-Learning-KUKA-RL](https://github.com/JesseDolfin/Co-Learning-KUKA-RL)

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/JesseDolfin/co_learning_robot_personalities.svg?style=for-the-badge
[contributors-url]: https://github.com/JesseDolfin/co_learning_robot_personalities/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JesseDolfin/co_learning_robot_personalities.svg?style=for-the-badge
[forks-url]: https://github.com/JesseDolfin/co_learning_robot_personalities/network/members
[stars-shield]: https://img.shields.io/github/stars/JesseDolfin/co_learning_robot_personalities.svg?style=for-the-badge
[stars-url]: https://github.com/JesseDolfin/co_learning_robot_personalities/stargazers
[issues-shield]: https://img.shields.io/github/issues/JesseDolfin/co_learning_robot_personalities.svg?style=for-the-badge
[issues-url]: https://github.com/JesseDolfin/co_learning_robot_personalities/issues
[license-shield]: https://img.shields.io/github/license/JesseDolfin/co_learning_robot_personalities.svg?style=for-the-badge
[license-url]: https://github.com/JesseDolfin/co_learning_robot_personalities/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/Jesse-Dolfin
[product-screenshot]: https://preview.free3d.com/img/2015/06/2205987029685109856/qyz27g5f.jpg
[Python.py]: https://upload.wikimedia.org/wikipedia/commons/thumb/c/c3/Python-logo-notext.svg/1869px-Python-logo-notext.svg.png
[Python-url]: https://www.python.org/


