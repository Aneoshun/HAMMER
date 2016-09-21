# HAMMER
high performing and light weight implementation of the HAMMER architecture

## Authors:
- Antoine Cully a.cully@imperial.ac.uk

## Dependencies:
- Boost
- TBB
- Eigen3
- NLopt (optional)


## Compilation:
from the Hammer root directory:
```
mkdir build
cd build
cmake ../
make
```

## Run the example:
The compilation process automatically detect all the experiment folders located in `exp/` and the executables are stored in `build/exp/"NAMEOFEXPERIMENTFOLDER/bin/'.


## Adding experiments:
New experiments can be added to hammer by creating a new folder in 'exp/' (from the Hammer root folder).
In this folder you need to add a `CMakeList.txt` file containing the creation of the executable. 
For instance in the `example` experiment:
```
add_executable(hammerExample ./main.cpp)

target_link_libraries(hammerExample ${TBB_LIBRARIES} ${Boost_LIBRARIES} ${NLOPT_LIBRARIES})
```
In order to compile the newly created experiment, you will need to regenerate the makefiles. Therefore you need to do:
```
cd build
cmake ../
make
``` 
