# Awesome Kalman Filter

![Awesome](https://awesome.re/badge.svg) [![Contributions](https://img.shields.io/github/issues-pr-closed-raw/mintisan/awesome-kalman-filter.svg?label=contributions)](https://github.com/mintisan/awesome-kalman-filter/pulls) [![Commits](https://img.shields.io/github/last-commit/mintisan/awesome-kalman-filter.svg?label=last%20contribution)](https://github.com/gigwegbe/tinyml-papers-and-projects/commits/main) ![GitHub stars](https://img.shields.io/github/stars/mintisan/awesome-kalman-filter.svg?style=social)

A curated list of awesome libraries, projects, tutorials, papers, and other resources related to Kalman Filter(KF). This repository aims to be a comprehensive and organized collection that will help researchers and developers in the world of KF!


## Table of Contents

- [Awesome Kalman Filter(KF)](#awesome-kalman-filter)
  - [Tutorial](#tutorial)
  - [Book](#book)
  - [Paper](#paper)
    - [Classical](#classical)
    - [Implementation](#implementtion)
    - [Survey](#survey)
    - [PaperWithCode](#paperwithcode)
  - [Code](#code)
    - [C](#c)
    - [C++](#c)
    - [Python](#python)
    - [Matlab](#matlab)
  - [Contributing](#contributing)
  - [License](#license)

## Tutorial

- [An Introduction to the Kalman Filter](https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf)
- [Understanding and Applying Kalman Filtering](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/kalman/kleeman_understanding_kalman.pdf)
- [A Brief Tutorial on the Ensemble Kalman Filter](https://arxiv.org/abs/0901.3725)
- [An Elementary Introduction to Kalman Filtering](https://arxiv.org/abs/1710.04055)
- [Unscented Kalman Filter Tutorial](https://www.researchgate.net/profile/Mohamed-Mourad-Lafifi/post/Unscented-Kalman-Filter/attachment/5a3401d34cde266d587b4efd/AS%3A571878130622464%401513357779148/download/Tutorial+UKF.pdf)
- [The Unscented Kalman Filter for Nonlinear Estimation](https://groups.seas.harvard.edu/courses/cs281/papers/unscented.pdf)
- [A Step by Step Mathematical Derivation and Tutorial on Kalman Filters](https://arxiv.org/abs/1910.03558)
- [Kalman Filter For Dummies](http://bilgin.esme.org/BitsAndBytes/KalmanFilterforDummies)
- [How a Kalman filter works, in pictures](https://www.bzarg.com/p/how-a-kalman-filter-works-in-pictures/)
- [Extended Kalman Filter Tutorial](https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf)
- [YouTube-Kalman Filter for Beginners, Part 1 - Recursive Filters & MATLAB Examples](https://www.youtube.com/watch?v=HCd-leV8OkU)
- [Tutorial: Understanding Nonlinear Kalman Filters, Part I: Selection between EKF and UKF](https://yugu.faculty.wvu.edu/research/interactive-robotics-letters/understanding-nonlinear-kalman-filters-part-i)
- [Tutorial: Understanding Nonlinear Kalman Filters, Part II: An Implementation Guide](https://yugu.faculty.wvu.edu/research/interactive-robotics-letters/understanding-nonlinear-kalman-filters-part-ii)
- [The Unscented Kalman Filter, simply the best! Python code](https://jgoslinski.medium.com/the-unscented-kalman-filter-simply-the-best-python-code-5cd5ebaebf5f)

## Book

- [**Kalman and Bayesian Filters in Python**](https://elec3004.uqcloud.net/2015/tutes/Kalman_and_Bayesian_Filters_in_Python.pdf)
- [Kalman Filter Made Easy](https://thekalmanfilter.com/kalman-filter-made-easy-ebook/)
- [Unscented Kalman Filter Made Easy](https://thekalmanfilter.com/unscented-kalman-filter-made-easy/)
- [Introduction and Implementations of the Kalman Filter](https://www.intechopen.com/books/7466)
  - [Introduction to Kalman Filter and Its Applications](https://www.intechopen.com/chapters/63164)

## Paper

- 1960-Kalman Filter-[A New Approach to Linear Filtering and Prediction Problems](https://www.unitedthc.com/DSP/Kalman1960.pdf) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1115/1.3662552)](https://juleskreuer.eu/citation-badge/)
### Classical

- 1985-Extended Kalman Filter-[Discovery of the Kalman Filter as a Practical Tool for Aerospace and Industry](https://ntrs.nasa.gov/api/citations/19860003843/downloads/19860003843.pdf)
- 1997-Unscented Kalman Filter-[A New Extension of the Kalman Filter to Nonlinear Systems](https://www.cs.unc.edu/~welch/kalman/media/pdf/Julier1997_SPIE_KF) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1117/12.280797)](https://juleskreuer.eu/citation-badge/)
- 2002-Square Root Uncented Kalman Filter-[The square-root unscented Kalman filter for state and parameter-estimation](https://ieeexplore.ieee.org/document/940586) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1109/ICASSP.2001.940586)](https://juleskreuer.eu/citation-badge/)
- 2003-Ensemble Kalman Filter-[The Ensemble Kalman Filter: Theoretical Formulation and Practical Implementation](https://www.ecmwf.int/sites/default/files/elibrary/2003/9321-ensemble-kalman-filter-theoretical-formulation-and-practical-implementation.pdf) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1007/s10236-003-0036-9)](https://juleskreuer.eu/citation-badge/)
- 2006-Kinematic Kalman filter-[Benefits of acceleration measurement in velocity estimation and motion control](https://www.sciencedirect.com/science/article/abs/pii/S096706610500242X) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1016/j.conengprac.2005.10.004)](https://juleskreuer.eu/citation-badge/)
- 2006-Quaternion Kalman Filter-[Novel quaternion Kalman filter](https://ieeexplore.ieee.org/document/1603413) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1109/TAES.2006.1603413)](https://juleskreuer.eu/citation-badge/)
- 2009-Cubature Kalman Filters-[Cubature Kalman Filters](https://ieeexplore.ieee.org/document/4982682) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1109/TAC.2009.2019800)](https://juleskreuer.eu/citation-badge/)

### Implementation

- 2017-[A Study about Kalman Filters Applied to Embedded Sensors](https://www.mdpi.com/1424-8220/17/12/2810) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.3390/s17122810)](https://juleskreuer.eu/citation-badge/)
- 2020-[Extended Kalman Filter with Reduced Computational Demands for Systems with Non-Linear Measurement Models](https://www.mdpi.com/1424-8220/20/6/1584) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.3390/s20061584)](https://juleskreuer.eu/citation-badge/)
- 2022-[Implementation of a C Library of Kalman Filters for Application on Embedded Systems](https://www.mdpi.com/2073-431X/11/11/165) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.3390/computers11110165)](https://juleskreuer.eu/citation-badge/)
- 2022-[Optimization of an unscented Kalman filter for an embedded platform](https://www.sciencedirect.com/science/article/abs/pii/S0010482522003493) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1016/j.compbiomed.2022.105557)](https://juleskreuer.eu/citation-badge/)

### Survey

- 2016-[A comparative study and review of different Kalman filters by applying an enhanced validation method](https://www.sciencedirect.com/science/article/abs/pii/S2352152X16302031) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1016/j.est.2016.10.004)](https://juleskreuer.eu/citation-badge/)
- 2021-[A Survey of Kalman Filter Algorithms and Variants in State Estimation](https://www.researchgate.net/publication/353897085_A_Survey_of_Kalman_Filter_Algorithms_and_Variants_in_State_Estimation) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.9734/bpi/castr/v15/11912D)](https://juleskreuer.eu/citation-badge/)
- 2021-[Research Article Kalman Filter: Historical Overview and Review of Its Use in Robotics 60 Years after Its Creation](https://onlinelibrary.wiley.com/doi/10.1155/2021/9674015) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1155/2021/9674015)](https://juleskreuer.eu/citation-badge/)
- 2023-[State of art on state estimation: Kalman filter driven by machine learning](https://www.sciencedirect.com/science/article/abs/pii/S1367578823000731) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1016/j.arcontrol.2023.100909)](https://juleskreuer.eu/citation-badge/)

### PaperWithCode

- 1999-[Adaptive Kalman Filtering for INS/GPS](https://link.springer.com/article/10.1007/s001900050236) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1007/s001900050236)](https://juleskreuer.eu/citation-badge/)  | [SeongchunYang/KF/Schwarz_AUKF.py](https://github.com/SeongchunYang/KF/blob/master/KF/Schwarz_AUKF.py)
- 1999-[Robust estimation with unknown noise statistics](https://ieeexplore.ieee.org/abstract/document/769393) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1109/9.769393)](https://juleskreuer.eu/citation-badge/) | [code-milsto/robust-kalman](https://github.com/milsto/robust-kalman)
- 2009-[An Adaptive Unscented Kalman Filter for Dead Reckoning Systems](https://ieeexplore.ieee.org/document/5365064) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.1109/ICIECS.2009.5365064)](https://juleskreuer.eu/citation-badge/)  | [SeongchunYang/KF/Zhang_AUKF.py](https://github.com/SeongchunYang/KF/blob/master/KF/Zhang_AUKF.py)
- 2018-[A Robust Adaptive Unscented Kalman Filter for Nonlinear Estimation with Uncertain Noise Covariance](https://www.mdpi.com/1424-8220/18/3/808)  [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.3390/s18030808)](https://juleskreuer.eu/citation-badge/) | [SeongchunYang/KF/Zheng_AUKF.py](https://github.com/SeongchunYang/KF/blob/master/KF/Zheng_AUKF.py)
- 2019-[Adaptive Kalman filter for detectable linear time invariant systems](https://sites.utexas.edu/renato/files/2019/03/main.pdf) [![Citation Badge](https://api.juleskreuer.eu/citation-badge.php?doi=10.2514/1.G004359)](https://juleskreuer.eu/citation-badge/) | [code-BenGravell/adaptive-kalman-filter](https://github.com/BenGravell/adaptive-kalman-filter)

## Code


### C

- [simondlevy/TinyEKF](https://github.com/simondlevy/TinyEKF) : Lightweight C/C++ Extended Kalman Filter with Python for prototyping | C/Python | ![Github stars](https://img.shields.io/github/stars/simondlevy/TinyEKF.svg)
- [sunsided/kalman-clib](https://github.com/sunsided/kalman-clib) : Microcontroller targeted C library for Kalman filtering | C | ![Github stars](https://img.shields.io/github/stars/sunsided/kalman-clib.svg)
- [dr-duplo/eekf](https://github.com/dr-duplo/eekf) : C-Implementation of an Extended Kalman Filter for use in embedded applications | C | ![Github stars](https://img.shields.io/github/stars/dr-duplo/eekf.svg)
- [commaai/rednose](https://github.com/commaai/rednose) : Kalman filter library | Python/C/C++ | ![Github stars](https://img.shields.io/github/stars/commaai/rednose.svg)
- [ivo-georgiev/ukfLib](https://github.com/ivo-georgiev/ukfLib) : Unscented Kalman filter C library | C | ![Github stars](https://img.shields.io/github/stars/ivo-georgiev/ukfLib.svg)
- [sunsided/libfixkalman](https://github.com/sunsided/libfixkalman) : Kalman filter fixed-point implementation based on libfixmatrix, targeted at embedded systems without an FPU and/or need for performance. | C | ![Github stars](https://img.shields.io/github/stars/sunsided/libfixkalman.svg)

### C++

- [mherb/kalman](https://github.com/mherb/kalman) : Header-only C++11 Kalman Filtering Library (EKF, UKF) based on Eigen3  | C++ | ![Github stars](https://img.shields.io/github/stars/mherb/kalman.svg)
- [pronenewbits/Embedded_EKF_Library](https://github.com/pronenewbits/Embedded_EKF_Library) : A compact Extended Kalman Filter (EKF) library for real time embedded system (with template for Teensy4/Arduino and STM32CubeIDE) | C++ | ![Github stars](https://img.shields.io/github/stars/pronenewbits/Embedded_EKF_Library.svg)
- [pronenewbits/Embedded_UKF_Library](https://github.com/pronenewbits/Embedded_UKF_Library) : A compact Unscented Kalman Filter (UKF) library for Teensy4/Arduino system (or any real time embedded system in general) | C++ | ![Github stars](https://img.shields.io/github/stars/pronenewbits/Embedded_UKF_Library.svg)
- [Moment-based Kalman Filter: Nonlinear Kalman Filtering with Exact Moment Propagation](https://github.com/purewater0901/MKF) | C++ | ![Github stars](https://img.shields.io/github/stars/purewater0901/MKF.svg)
- [kam3k/unscented](https://github.com/kam3k/unscented) : A flexible and powerful unscented Kalman filter C++11 library that makes no assumptions about what you're estimating or how you're measuring it. | C++ | ![Github stars](https://img.shields.io/github/stars/kam3k/unscented.svg)
- [sfwa/ukf](https://github.com/sfwa/ukf) : Unscented Kalman Filter library for state and parameter estimation | C++ | ![Github stars](https://img.shields.io/github/stars/sfwa/ukf.svg)
- [shazraz/Unscented-Kalman-Filter](https://github.com/shazraz/Unscented-Kalman-Filter) : UKF implementation in C++ using noisy LIDAR and RADAR data for object tracking | C++ | ![Github stars](https://img.shields.io/github/stars/shazraz/Unscented-Kalman-Filter.svg)
- [shazraz/Extended-Kalman-Filter](https://github.com/shazraz/Extended-Kalman-Filter) : Implementation of an EKF in C++ | C++ | ![Github stars](https://img.shields.io/github/stars/shazraz/Extended-Kalman-Filter.svg)
- [JunshengFu/tracking-with-Unscented-Kalman-Filter](https://github.com/JunshengFu/tracking-with-Unscented-Kalman-Filter) : Object (e.g Pedestrian, biker, vehicles) tracking by Unscented Kalman Filter (UKF), with fused data from both lidar and radar sensors. | C++ | ![Github stars](https://img.shields.io/github/stars/JunshengFu/tracking-with-Unscented-Kalman-Filter.svg)
- [udacity/CarND-Unscented-Kalman-Filter-Project](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project) : Self-Driving Car Nanodegree Program Starter Code for the Unscented Kalman Filter Project| C++ | ![Github stars](https://img.shields.io/github/stars/udacity/CarND-Unscented-Kalman-Filter-Project.svg)
- [denyssene/SimpleKalmanFilter](https://github.com/denyssene/SimpleKalmanFilter) : A basic implementation of Kalman Filter for single variable models. | C++ | ![Github stars](https://img.shields.io/github/stars/denyssene/SimpleKalmanFilter.svg)
- [TKJElectronics/KalmanFilter](https://github.com/TKJElectronics/KalmanFilter) : This is a Kalman filter used to calculate the angle, rate and bias from from the input of an accelerometer/magnetometer and a gyroscope | C++ | ![Github stars](https://img.shields.io/github/stars/TKJElectronics/KalmanFilter.svg)
- [artivis/kalmanif](https://github.com/artivis/kalmanif) : A small collection of Kalman Filters on Lie groups | C++ | ![Github stars](https://img.shields.io/github/stars/artivis/kalmanif.svg)

### Python

- [**rlabbe/Kalman-and-Bayesian-Filters-in-Python**](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) : Kalman Filter book using Jupyter Notebook. Focuses on building intuition and experience, not formal proofs. Includes Kalman filters,extended Kalman filters, unscented Kalman filters, particle filters, and more. All exercises include solutions. | Python | ![Github stars](https://img.shields.io/github/stars/rlabbe/Kalman-and-Bayesian-Filters-in-Python.svg)
- [sharathsrini/Kalman-Filter-for-Sensor-Fusion](https://github.com/sharathsrini/Kalman-Filter-for-Sensor-Fusion) : A Sensor Fusion Algorithm that can predict a State Estimate and Update if it is uncertain | Python | ![Github stars](https://img.shields.io/github/stars/sharathsrini/Kalman-Filter-for-Sensor-Fusion.svg)
- [rlabbe/filterpy](https://github.com/rlabbe/filterpy) : Python Kalman filtering and optimal estimation library. Implements Kalman filter, particle filter, Extended Kalman filter, Unscented Kalman filter, g-h (alpha-beta), least squares, H Infinity, smoothers, and more. Has companion book 'Kalman and Bayesian Filters in Python'. | Python | ![Github stars](https://img.shields.io/github/stars/rlabbe/filterpy.svg)
- [pykalman/pykalman](https://github.com/pykalman/pykalman) : Kalman Filter, Smoother, and EM Algorithm for Python | Python | ![Github stars](https://img.shields.io/github/stars/pykalman/pykalman.svg)
- [milsto/robust-kalman](https://github.com/milsto/robust-kalman) : Robust Kalman filter with adaptive noise statistics estimation. | Python | ![Github stars](https://img.shields.io/github/stars/milsto/robust-kalman.svg)

### Matlab

 - [mannyray/KalmanFilter](https://github.com/mannyray/KalmanFilter) : Kalman filter sanctuary - including continuous-discrete extended Kalman filter. Bring additional filters here for a bigger collection. | Matlab/C++/Python | ![Github stars](https://img.shields.io/github/stars/mannyray/KalmanFilter.svg)


## Contributing

We welcome your contributions! Please follow these steps to contribute:

1. Fork the repo.
2. Create a new branch (e.g., `feature/new-kf-resource`).
3. Commit your changes to the new branch.
4. Create a Pull Request, and provide a brief description of the changes/additions.

Please make sure that the resources you add are relevant to the field of Kalman Filter. Before contributing, take a look at the existing resources to avoid duplicates.

## License

This work is licensed under a [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).

