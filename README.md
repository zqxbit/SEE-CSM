# SEE-CSM
SEE-CSM: Sharp-Edged and Efficient Continous Semantic Mapping through Multi-entropy Kernel Inference 

<img src="https://github.com/BIT-DYN/SEE-CSM/blob/main/image/CSM.png" width="250"><img src="https://github.com/BIT-DYN/SEE-CSM/blob/main/image/BKI.png" width="250">
<img src="https://github.com/BIT-DYN/SEE-CSM/blob/main/image/DYN.png" width="250">

This is a novel continuous semantic mapping algorithm, which can complete dense but not thick semantic map reconstruction efficiently.

## Getting Started

### Building with catkin

```bash
catkin_ws/src$ git clone https://github.com/BIT-DYN/SEE-CSM
catkin_ws/src$ cd ..
catkin_ws$ catkin_make
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Building using Intel C++ compiler (optional for better speed performance)
```bash
catkin_ws$ source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64
catkin_ws$ catkin_make -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
catkin_ws$ source ~/catkin_ws/devel/setup.bash
```

### Running the Demo

```bash
$ roslaunch semantic_bki toy_example_node.launch
```

### Running
```bash
$ roslaunch semantic_bki kitti_node.launch
```


<img src="https://github.com/BIT-DYN/SEE-CSM/blob/main/image/25truth.png" width="1000">
<img src="https://github.com/BIT-DYN/SEE-CSM/blob/main/image/25.png" width="1000">
