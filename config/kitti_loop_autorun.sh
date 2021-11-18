rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/kitti_param.yaml

min_rs=(0.5 2.0 4.0 6.0 8.0)
max_rs=(2.0 4.0 6.0 8.0 10.0)

for jj in 1 2 3 4 5 
do
    for alg in "TEASER"
    do
        rosparam set targetAlgorithm $alg
        rosparam set /loop/min_radius ${min_rs[$jj]}
        rosparam set /loop/max_radius ${max_rs[$jj]}
        OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/kitti_rot_benchmark
    done
done
