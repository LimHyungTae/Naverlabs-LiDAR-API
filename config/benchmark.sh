rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/target_algorithm.yaml
rosparam load /home/shapelim/catkin_ws/src/deep_registration/config/benchmark.yaml
min_rs=(0.0 2.0 4.0 6.0 8.0 9.0 10.0)
max_rs=(2.0 4.0 6.0 8.0 10.0 12.0 12.0)

for jj in 7 # 5
do
    for seq in "'08'" "'09'" "'10'" # "'00'" "'02'" "'05'" "'06'" "'07'" "'08'"     
    do  
        for alg in "SONNY"
        do 
            rosparam set /loop/minRadius ${min_rs[$jj]}
            rosparam set /loop/maxRadius ${max_rs[$jj]}
            rosparam set /dataId $seq
            rosparam set /targetAlgorithm $alg
            OMP_NUM_THREADS=12 /home/shapelim/deep_registration/cmake-build-release/devel/lib/deep_registration/main_benchmark
        done
    done
done

