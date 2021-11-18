rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/kitti_param.yaml
jj="'02'"
i=5
rosparam set sequence $jj
rosparam set targetAlgorithm "SONNY"
rosparam set interval $i

OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/kitti_benchmark

