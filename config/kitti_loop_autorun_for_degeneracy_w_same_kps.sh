rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/kitti_param.yaml

for jj in "'00'" "'05'" "'06'" "'07'" "'08'" "'09'" # "'02'" 
do  
    rosparam set sequence $jj
    rosparam set targetAlgorithm $alg
    OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/kitti_rot_w_same_kps
done

