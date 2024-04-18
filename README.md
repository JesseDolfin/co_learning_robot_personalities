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

<h3 align="center">Co-learning for KUKA iiwa7</h3>

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

This repository requires access to the TU Delft CoR gitlab iiwa repository: https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/iiwa_ros.
Please follow the installation instructions from iiwa_ros before installing this package.


### Installation

1. Clone the repo in the same workspace you defined during the installation of iiwa_ros, and update the submodules
   ```sh
   cd <work space>
   git clone https://github.com/JesseDolfin/Co-Learning-KUKA-RL.git
   cd Co-Learning-KUKA-RL
   git submodule update --recursive --remote
   ```
2. Build the package
   ```sh
   catkin_make
   source devel/setup.bash
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

After installing the repository navigate to src/launch and run roslaunch to launch the gazebo simulation.
```sh
cd src
roslaunch launch/co_learning_test_setup.launch
```

Then you can start the control node and the secondary task node:
```sh
python ControlNode.py
python Secondary_task/src/secondary_task.py
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
[contributors-shield]: https://img.shields.io/github/contributors/JesseDolfin/Co-Learning-KUKA-RL.svg?style=for-the-badge
[contributors-url]: https://github.com/JesseDolfin/Co-Learning-KUKA-RL/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JesseDolfin/Co-Learning-KUKA-RL.svg?style=for-the-badge
[forks-url]: https://github.com/JesseDolfin/Co-Learning-KUKA-RL/network/members
[stars-shield]: https://img.shields.io/github/stars/JesseDolfin/Co-Learning-KUKA-RL.svg?style=for-the-badge
[stars-url]: https://github.com/JesseDolfin/Co-Learning-KUKA-RL/stargazers
[issues-shield]: https://img.shields.io/github/issues/JesseDolfin/Co-Learning-KUKA-RL.svg?style=for-the-badge
[issues-url]: https://github.com/JesseDolfin/Co-Learning-KUKA-RL/issues
[license-shield]: https://img.shields.io/github/license/JesseDolfin/Co-Learning-KUKA-RL.svg?style=for-the-badge
[license-url]: https://github.com/JesseDolfin/Co-Learning-KUKA-RL/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/Jesse-Dolfin
[product-screenshot]: https://preview.free3d.com/img/2015/06/2205987029685109856/qyz27g5f.jpg
[Python.py]: https://upload.wikimedia.org/wikipedia/commons/thumb/c/c3/Python-logo-notext.svg/1869px-Python-logo-notext.svg.png
[Python-url]: https://www.python.org/


