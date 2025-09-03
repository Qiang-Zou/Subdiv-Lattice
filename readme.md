# Subdivisional B-rep Evaluation of Lattice Structures (Subdiv-Lattice)

- By: Guoyue Luo and Qiang Zou
-  email: qzou.code@gmail.com
- webpage: https://qiang-zou.github.io/
- Latest Release: 2025.09


## !important
The source code was developed as a byproduct of the projects and methods presented in [1], as well as other related projects carried out in our group [2-7]. It implements a new boundary evaluation method of lattice structures. It solves the challenges of robustness towards generating lattice intersections and boundary shapes. The core idea is to replace strut-strut intersections with subdivision surfaces, and the design of those surfaces mimics the shapes of soap films.

It can be compiled with MSVC 14.0, and run on the operating system Windows 10.


1.Copyright
-----------

- SubdivLattice is developed and now maintained by Guoyue Luo and Qiang Zou for research use. All rights about the program are reserved by Guoyue Luo and Qiang Zou. This C++ source codes are available only to a primary use for academic purposes. No secondary use, such as copy, distribution, diversion, business purpose, etc., is allowed. In no event shall the author be liable to any party for direct, indirect, special, incidental, or consequential damage arising out of the use of this program. SubdivLattice is self-contained. 


2.Download
----------

- The source code, as well as the testing data, can be downloaded from the page: 
  
  webpage: https://github.com/Qiang-Zou/Subdiv-Lattice/


3.Installing & Compiling (Windows+MSVS14.0)
-------------------------------------------

- Simply download the source code to a suitable place, install Eigen, pmp, OpenMesh, OpenCascade Libraries, and use MSVC14.0 to build the project.

4.Usage
-------

- After the compilation you can run the tool SubDivLattice.exe inside the ./x64/release/ directory:
 
	- **Note** The data files should be located in ./data directory. Before using the tool, please unpack all model files to the directory ./data in advance. The models will be placed in results folder. 

- When running the .exe file, you need input the path of .node file and .edge file, for example:
	
"../../subDivLattice/data/cube.node"
"../../subDivLattice/data/cube.edge"
	

5.File format
-------------
- Node file: lattice node file (placed in ./data, and any extension can be used).
	- line 1:	edge_number dimension 0 0
	- lines 2-n:	x1 y1 z1
	- **Note** x1 y1 z1: coordinates of one end point

- Edge file: lattice edge file (placed in ./data, and any extension can be used).

	- line 1:	edge_number 1
	- lines 2-n:	n1 n2 x
	- **Note** n1 n2: the index of node; x: edge mark used in Tetgen


6.References
-------------
- [1] Guoyue Luo, Qiang Zou, Soap Film-inspired Subdivisional Lattice Structure Construction, Computer_Aided Design, 2025.
- [2] Qiang Zou, Guoyue Luo, Geometric modeling for microstructure design and manufacturing: A review, Computer-Aided Design (2024).
- [3] Sifan Chen, Guoyue Luo, Yuan Kong, Qiang Zou A Quasi-Optimal Shape Design Method for Lattice Structure Construction, ASME Trans. JMD (2025)
- [4] Qiang Zou, Yunzhu Gao, Guoyue Luo, Sifan Chen, Meta-meshing and triangulating lattice structures at a large scale, Computer-Aided Design (2024).
- [5] Yaonaiming Zhao, Qiang Zou, Guoyue Luo, Jiayu Wu, Sifan Chen, Depeng Gao, Minghao Xuan, Fuyu Wang, TPMS2STEP: error-controlled and C2 continuity-preserving translation of TPMS models to STEP files based on constrained-PIA, Computer-Aided Design (2024).
- [6] Junhao Ding, Qiang Zou, Shuo Qu, Paulo Bartolo, Xu Song, Charlie C. L. Wang, STL-free digital design and manufacturing paradigm for high-precision selective laser melting, CIRP Annals. 2021 70(1): 167-170.
- [7] Shengjun Liu, Tao Liu, Qiang Zou, Weiming Wang, Eugeni L. Doubrovski, Charlie C. L. Wang, Memory-Efficient Modeling and Slicing of Large-Scale Adaptive Lattice Structures, ASME Trans. JCISE. 21.6 (2021): 061003.