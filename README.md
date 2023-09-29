# IRIS-U^2 (Incremental Random Inspection-roadmap Search Under Uncertainty)

This is code for the paper **Inspection planning under execution uncertainty**. To cite this work, please use:

```
@article{alpert2023inspection,
  title={Inspection planning under execution uncertainty},
  author={Alpert, Shmuel David and Solovey, Kiril and Klein, Itzik and Salzman, Oren},
  journal={arXiv preprint arXiv:2309.06113},
  year={2023}
}
```

Link to the paper: [arXiv version](https://arxiv.org/pdf/2309.06113.pdf).

## Based on the IRIS algorithm

1. Science and Systems (RSS) 2019 paper *Toward Asymptotically-Optimal Inspection Planning via Efficient Near-Optimal Graph Search*. [[Paper](http://www.roboticsproceedings.org/rss15/p57.html)][[arXiv](https://arxiv.org/pdf/1907.00506.pdf)]

2. Extended code for IEEE International Conference on Robotics and Automation (ICRA) 2021 paper *Computationally-Efficient Roadmap-based Inspection Planning via Incremental Lazy Search*. [[Paper](https://ieeexplore.ieee.org/document/9561653)][[arXiv](https://arxiv.org/pdf/2103.13573.pdf)]

The original code can be found here: [[code](https://github.com/UNC-Robotics/IRIS)].

## Inspection planning under execution uncertainty

  Autonomous inspection tasks necessitate effective path-planning mechanisms to efficiently gather observations from points of interest POI. However, localization errors commonly encountered in urban environments can introduce execution uncertainty, posing challenges to the successful completion of such tasks. To tackle these challenges, we present IRIS-under uncertainty IRIS-U^2, an extension of the incremental random inspection-roadmap search IRIS algorithm, that addresses the offline planning problem via an A*-based approach, where the planning process occurs prior to the online execution.
The key insight  behind~\irisuu is transforming the computed localization uncertainty, obtained through Monte Carlo MC sampling, into a POI probability.
IRIS-U^2 offers insights into the expected performance of the execution task by providing confidence intervals CI for the expected coverage, expected path length, and collision probability, which becomes progressively tighter as the number of MC samples increases.
The efficacy of IRIS-U^2 is demonstrated through a case study focusing on structural inspections of bridges. Our approach exhibits improved expected coverage, reduced collision probability, and yields increasingly-precise CIs as the number of MC samples grows. Furthermore, we emphasize the potential advantages of computing bounded sub-optimal solutions  to reduce computation time while still maintaining the same CI boundaries.

## Requirements

* [GCC(for Linux)](https://gcc.gnu.org/) - v7.4
* [Boost](https://www.boost.org/) - v1.68.0
* [CMake](https://cmake.org/) - v3.8
* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) - v3.3.4
* [OMPL](https://ompl.kavrakilab.org/) - the Open Motion Planning Library v1.5.0

## Installation
1. Download 

1. First clone code to your local repository:

	```
	git clone https://github.com/CRL-Technion/IRIS-UU.git 
	git submodule update --init --recursive
	```

2. Additional bridge model can be found by downloading data from [google drive](https://drive.google.com/file/d/19DGtog4D4hAgwFu1bV_ct0h_n-G4BR1Z/view?usp=sharing) to your local respository, uncompress.

3. Install all dependencies. If you install dependencies to your specified directories, you will need to provide the information when compiling.

4. Compile:

	```
	cd {path to your local repository}
	mkdir build
	cd build
	cmake ..
	make
	```

5. Tested environment:

  * Ubuntu 18.04 (gcc 7.4.0)
  * Ubuntu 16.04 (gcc 7.4.0)
  * Ubuntu 14.04 (gcc 7.2.0)
  * Ubuntu 20.04.01 (gcc 9.4.0)
  * macOS Mojave Version 10.14.6 (clang-1001.0.46.4)
  * macOS Catalina Version 10.15 (clang-1100.0.33.8)
    
## Usage

1. Specify which scenario to use in ```include/global_common.h```:
	
	```
	#define USE_CRISP 0
	#define USE_PLANAR 0
	```

	If ```USE_CRISP``` is set to 1, then the CRISP robot is used.
	
	If ```USE_CRISP``` is set to 0 and ```USE_PLANAR``` is set to 1, then the planar robot is used.

	If both ```USE_CRISP``` and ```USE_PLANAR``` are set to 0, then the drone robot is used.

	*Important for macbook users:*

	OMPL uses C++17 deprecated functions, and clang reports errors when you set C++ standard to 17.
	A simple way to solve this is to disable drone robot (which is done by the current cmake files), all you need to do is avoid setting both ```USE_CRISP``` and ```USE_PLANAR``` to 0.
	If you still want to use the drone robot, please replace ```{OMPL_Source}/src/ompl/datastructures/NearestNeighborsGNAT.h``` with the one in external folder, rebuild and reinstall OMPL, finally change line 23 in root ```CMakeLists.txt``` to ```set(USE_C++17 1)```.

2. Build a graph (roadmap):

	```
	cd {path to your local repository}/build
	./app/build_graph seed num_vertex file_to_write 
	```

	When constructing the roadmap, we now allow rejection sampling to favor samples that increase inspection coverage. Set ```REJECT_SAMPLING``` (in ```include/global_common.h```) to 1 to enable this feature.

2. Search a graph:

	```
	cd {path to your local repository}/build
	./app/search_graph file_to_read initial_p initial_eps tightening_rate laziness_mode successor_mode batching_ratio file_to_write
	```

	Here, four different laziness modes are provided:

	* 0 -- No lazy computation
	* 1 -- Lazy SP (complete lazy)
    * 2 -- Lazy A* modified (validate when subsuming for the first time, final method in the paper)
    * 3 -- Lazy A* (validate only when popped from OPEN list, performance worse than 2, keep for reference)

    There are also three successor modes provided:

    * 0 -- direct neighboring successors on the roadmap (default, preferred)
    * 1 -- First neighbor that increases inspection coverage (keep for reference)
    * 2 -- first neighbor that increases inspection coverage and there's no other node increasing the coverage along the shortest path from its parent (keep for reference)

    In ```include/global_common.h```, there are also additional macros to enable different features, namely ```USE_NODE_REUSE```, ```KEEP_SUBSUMING_HISTORY```, and ```SAVE_PREDECESSOR```. ```USE_NODE_REUSE``` enables reusing search efforts from previous search iteration. ```KEEP_SUBSUMING_HISTORY``` enables saving detailed information about subsumed node, which is essential for lazy edge validation (laziness mode 3) and search effort reusing. ```SAVE_PREDECESSOR``` is an additional optimization for subusming history keeping, that saves memory footprint by saving the predecessor of the subsumed node instead of saving the subsumed node directly.
