# BKISemanticMapping
Bayesian Spatial Kernel Smoothing for Scalable Dense Semantic Mapping

<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki.png" width="300">
<img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_csm_variance.png" width="300"><img src="https://raw.githubusercontent.com/ganlumomo/BKISemanticMapping/master/github/toy_example_semantic_bki_variance.png" width="300">

This is a novel continuous semantic mapping algorithm, which can complete dense semantic map reconstruction more efficiently.

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
You will see semantic map in RViz. It also projects 3D grid onto 2D image for evaluation, stored at data/data_kitti_05/reproj_img.
