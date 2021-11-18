rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/kitti_param.yaml
min_rs=(0.5 2.0 4.0 6.0 8.0)
max_rs=(2.0 4.0 6.0 8.0 10.0)
for ii in 1 2 3 4 5
do
    for jj in "'00'" "'02'" "'05'" "'06'" "'07'" "'08'" "'09'"
    do  
        rosparam set sequence $jj
        rosparam set /loop/min_radius ${min_rs[$ii]}
        rosparam set /loop/max_radius ${max_rs[$ii]}
        OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/save_tf
    done
done 
