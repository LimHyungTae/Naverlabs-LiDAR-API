rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/kitti_param.yaml
for jj in "'00'" # "'02'" "'05'" "'06'" "'07'" "'09'" # "'02'" "'03'" "'04'" "'05'" "'06'" "'07'" "'08'" "'09'" "'10'" 
do
    rosparam set sequence $jj
    for i in 5
    do
        rosparam set targetAlgorithm "SONNY"
        rosparam set interval $i
        OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/kitti_benchmark
    done
done
