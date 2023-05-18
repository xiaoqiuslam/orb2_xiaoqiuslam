## 编译运行环境

- Ubuntu 20.04

```cpp
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml /media/q/q/data/TUM/rgbd_dataset_freiburg1_xyz

./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml /media/q/q/data/Euroc/MH_01_easy/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/MH01.txt 

./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml /home/q/orb2_xiaoqiuslam/MH_01_easy/mav0/cam0/data /home/q/orb2_xiaoqiuslam/MH_01_easy/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/MH01.txt

./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUM2.yaml /media/q/q/data/TUM/rgbd_dataset_freiburg1_xyz /home/q/ORB_SLAM2/Examples/RGB-D/associations/fr1_xyz.txt
```

运行此代码需要先下载如下数据集和词典并且放在下面的路径
/home/q/orb2_xiaoqiuslam/MH_01_easy
/home/q/orb2_xiaoqiuslam/Vocabulary/ORBvoc.txt.tar.gz