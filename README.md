
# X,Y,theta planning in continuous space with soft duplicate detection using subtree overlap.

### About soft duplicate detection
Please refer to this paper: 

### Dependencies:
This package is only dependent on the sbpl library.
So install the sbpl library first: based on [this](https://github.com/Nader-Merai/sbpl_cspace) repo, which is from the Search-based planning Lab.

### Run this program:
To build this program:
```
mkdir build && cd build
cmake ../ && make
```
```
cd RRT/
mkdir build && cd build
cmake ../ && make
```
To run the planner:
```
./xythetac <mapname> <start&goal> <motion_primitive> <mode> <subtreeoverlapradius> <cvalue> <subtreedepth> <producehashtable>
```
-The \<mapname\> parameter is replaced by the map filename, which is in the format```(*.cfg)``` the same as the one in the sbpl library, found under ```map```. <br />
-The \<start&goal\> parameter is the start and goal setup file, which is in ```*.sg``` format, found under```map```. <br />
-The \<motion_primitive\> parameter is in format ```*.mprim```, which could be generated from the sbpl library, or found under ```matlab/motion_primitive_gen```. <br />
-The  \<mode\> parameter decides the planning algorithm used. 0 is for original soft duplicate detection, 1 is for subtree overlap labeler, and 2 is for subtree overlap hash table (hash table needs to be generated). <br />
-The  \<subtreeoverlapradius\> parameter is the subtree overlap radius. Value of 0.1 was used in the paper. <br />
-The  \<cvalue\> parameter is the c value in the duplicity detection formula. Value of 0.1 was used in the paper. <br />
-The  \<subtreedepth\> parameter is the depth of the subtree built for overlap. Value of 3 was used in the paper. <br />
-The  \<producehashtable\> parameter with a value of 1 produces the hash table values into "Hash_Table_Values.txt", and with a value of 0 reads the hash table values from "Hash_Table_Values.txt". value of 1 must be used at least once to generate the hash table values, and after that, value of 0 may be used to avoid hash table values recalculation.

### Reproducing results and plots:
The directory by the name of ```err_bars_mean_std_err``` is for reproducing results, specifically raw results, error bars, and Average +- Standard Error statistics. For each section of figures in the paper (sections being: Normal Results, Varying C, Varying R, Varying H), there exists a subdirectory with its scripts. <br />
In order to get the results for a certain section, the following steps must be done:<br />
-Build the program as shown above<br />
-Go into the section's subdirectory in ```err_bars_mean_std_err```<br />
-Run: chmod 777 Copy_Scripts.txt Make_Plot.txt<br />
-Run: ./Copy_Scripts, or copy each script to the specified directory in the "Copy_Scripts.txt" file<br />
-Go into the specified directory of each copied script and run it.<br />
-Go back into the section's subdirectory in ```err_bars_mean_std_err```<br />
-There you will find all the raw data.<br />
-To reproduce the plots, Run: ./Make_Plot.txt<br />
-To view the Average +- Standard Error statistics of a certain run, go back to the ```err_bars_mean_std_err``` directory and Run: python3 Average_STD ${path_to_raw_data_run}