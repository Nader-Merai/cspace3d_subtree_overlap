
# X,Y,$\theta$ planning in continuous space with soft duplicate detection using subtree overlap.

### About soft duplicate detection
Please refer to this paper: 

### Dependencies:
This package is only dependent on the sbpl library.
So install the sbpl library first: based on [this](https://github.com/SushiStar/sbpl_cspace) repo, which is from the Search-based planning Lab.

### Run this program:
To build this program:
```
mkdir build && cd build
cmake ../ && make
```
To run the planner:
```
./xythetac <mapname> <start&goal> <motion_primitive> <mode> <subtreeoverlapradius> <cvalue> <subtreedepth>
```
-The \<mapname\> parameter is replaced by the map filename, which is in the format```(*.cfg)``` the same as the one in the sbpl library, found under ```map```.
-The \<start&goal\> parameter is the start and goal setup file, which is in ```*.sg``` format, found under```map```.
-The \<motion_primitive\> parameter is in format ```*.mprim```, which could be generated from the sbpl library, or found under ```matlab\motion_primitive_gen```.
-The  \<mode\> parameter decides the planning algorithm used. 0 is for original soft duplicate detection, 1 is for subtree overlap labeler, and 2 is for subtree overlap hash table (hash table needs to be generated).
-The  \<subtreeoverlapradius\> parameter is the subtree overlap radius. value of 0.1 was used in the paper.
-The  \<cvalue\> parameter is the c value in the duplicity detection formula. value 0.1 was used in the paper.
-The  \<subtreedepth\> parameter is the depth of the subtree built for overlap. value 3 was used in the paper.
