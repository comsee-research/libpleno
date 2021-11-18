![banner-logo](doc/imgs/banner-pleno.png)

---

The **libpleno** is an open-source C++ computer-vision library for plenoptic cameras modeling and processing. 

Quick Start
===========

### Pre-requisites

The libpleno library has a light dependency list:

 * [Eigen] version 3, a modern C++ matrix and linear-algebra library,
 * [boost] version 1.54 and up, portable C++ source libraries,
 * [OpenCV] version 3.2, a collection of algorithms and sample code for various computer vision problems,
 * [libv], a general purpose computer vision library developed at Pascal Institute (used for graphic and serialization),
 * [lma], a non-linear optimization library implementing the Levenberg Marquardt Algorithm,
 
and was compiled on:
  * Ubuntu 18.04.4 LTS, GCC 7.5.0, with Eigen 3.3.4, Boost 1.65.1, and OpenCV 3.2.0.
  
### Compilation & Installation 

If you are comfortable with Linux and CMake and have already installed the prerequisites above, the following commands should install libpleno on your system.

```
mkdir build && cd build
cmake .. -DUSE_OPEN_MP=true
make -j6
sudo make install
```

Applications
============

Currently available applications using the libpleno:
 * [COMPOTE](https://github.com/comsee-research/compote) (Calibration Of Multi-focus PlenOpTic camEra), a set of tools to pre-calibrate and calibrate (multifocus) plenoptic cameras.
 * [PRISM](https://github.com/comsee-research/prism) (Plenoptic Raw Image Simulator), a set of tools to generate and simulate raw images from (multifocus) plenoptic cameras.
 * [BLADE](https://github.com/comsee-research/blade) (Blur Aware Depth Estimation with a plenoptic camera), a set of tools to estimate depth map from raw images obtained by (multifocus) plenoptic cameras.
 * ...
 
Configuration file examples are given in the folder `./examples/config/` for the datasets `R12-A` (see below).
Observations for the dataset `R12-A` are also given in the folder `./examples/obs/`.
 
Datasets
========

* For calibration, datasets R12-A, R12-B, R12-C, R12-D and UPC-S can be downloaded [from here](https://github.com/comsee-research/plenoptic-datasets).
* For depth estimation, datasets R12-E, ES and ELP20 can be downloaded [from here](https://github.com/comsee-research/plenoptic-datasets).

Citing
======

If you use libpleno in an academic context, please cite the following publication:

	@inproceedings{labussiere2020blur,
	  title 	=	{Blur Aware Calibration of Multi-Focus Plenoptic Camera},
	  author	=	{Labussi{\`e}re, Mathieu and Teuli{\`e}re, C{\'e}line and Bernardin, Fr{\'e}d{\'e}ric and Ait-Aider, Omar},
	  booktitle	=	{Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
	  pages		=	{2545--2554},
	  year		=	{2020}
	}
	
or 

	@article{labussiere2021calibration
	  title	    =	{Leveraging blur information for plenoptic camera calibration},
	  author	=	{Labussi{\`e}re, Mathieu and Teuli{\`e}re, C{\'e}line and Bernardin, Fr{\'e}d{\'e}ric and Ait-Aider, Omar},
	  journal	=	{arXiv preprint arXiv:2111.05226},
	  year		=	{2021}
	}


License
=======

libpleno is licensed under the GNU General Public License v3.0. Enjoy!

[Ubuntu]: http://www.ubuntu.com
[CMake]: http://www.cmake.org
[CMake documentation]: http://www.cmake.org/cmake/help/cmake2.6docs.html
[git]: http://git-scm.com
[Eigen]: http://eigen.tuxfamily.org
[libv]: http://gitlab.ip.uca.fr/libv/libv
[lma]: http://gitlab.ip.uca.fr/libv/lma
[OpenCV]: https://opencv.org/
[Doxygen]: http://www.stack.nl/~dimitri/doxygen/
[boost]: http://www.boost.org/

---
