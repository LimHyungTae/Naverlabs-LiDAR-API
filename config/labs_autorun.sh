rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/labs_param.yaml

for i in 1 3 5
do
    rosparam set targetAlgorithm "SONNY_w_G_ICP"
    rosparam set interval $i
    OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/labs_benchmark
done

