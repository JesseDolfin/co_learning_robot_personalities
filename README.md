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
  </a>

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
    <li>
      <a href="#about-the-project">About The Project</a>
    </li>
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

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This repository requires access to the TU Delft CoR gitlab iiwa [repository](https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/iiwa_ros).
Please follow the installation instructions from iiwa_ros before installing this package.


### Installation

1. Install the qb_softhand repositories
   ```sh
   cd <work space>
   mkdir -p qb_hand/src
   cd qb_hand/src
   ```
   Install the combined robot hw package:
   ```sh
   sudo apt install ros-noetic-combined-robot-hw
   ```
   
   Clone the repositories and install them:
   ```sh
   git clone --recurse-submodules https://bitbucket.org/qbrobotics/qbdevice-ros.git
   git clone https://bitbucket.org/qbrobotics/qbhand-ros.git
   cd ..
   catkin_make
   ```
   Source the repository and add it to the path:
   ```sh
   source devel/setup.bash
   echo "source <work space>/qb_hand/devel/setup.bash" >> ~/.bashrc
   ```

1. Install the realsense software inside of your workspace.
    Follow the installation steps from their [repo](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file).

4. Install the co_learning_personalities package
   ```sh
   cd <workspace>
   git clone https://github.com/JesseDolfin/co_learning_robot_personalities.git
   cd co_learning_robot_personalities
   catkin_make
   ```

   Source the repository and add it to the path:
   ```sh
   source devel/setup.bash
   echo "source <work space>/co_learning_robot_personalities/devel/setup.bash" >> ~/.bashrc
   ```

5. Posterior installations:
  ```sh
  pip install numpy==1.21
  pip install python-dateutill==2.8.2
  pip install mediapipe
  pip install pyrealsense2
  pip install ultralytics
  ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

After installing and sourcing the software the simulation may be started using the following roslaunch command:
```sh
roslaunch co_learning_controllers co_learning_test_setup.launch
```

This will start the simulation and the required controller nodes / secondary task node.

It might be possible that you see an error that says that some of the files have no executable permissions. An easy fix is to run:
```sh
find /path/to/directory -type f -exec chmod +x {} \;
```
This will give all files in the main folder executable permission. Don't run this unless you trust all the repos installed! 
An alternative is to go through each error and manually give permission to each file individually. 

If you want to start this on the real robot run the command with the prefix `simulation:=false`:
```sh
roslaunch co_learning_controllers co_learning_test_setup.launch simulation:=false
```

_For more examples, please refer to the [Documentation](https://google.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
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


