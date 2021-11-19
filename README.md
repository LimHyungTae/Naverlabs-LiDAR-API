# NaverLabs Indoor - LiDAR dataset API

![](materials/lidar.gif)

## Descriptions

* It's realistic department datset!
    * Complex Indoor environments
    * Hundreds of dynamic objects
* Please refer to [NAVER LABS](https://europe.naverlabs.com/blog/first-of-a-kind-large-scale-localization-datasets-in-crowded-indoor-spaces/) website
* https://www.naverlabs.com/en/storyDetail/211
* Unfortunately, HD Map & Localization Dataset request is available only to **Korean researchers and organizations**... :(. 

https://www.naverlabs.com/en/datasets/requestDataset

## Data format
```
data_path (`/nvlabs/abs_dir` in `lidar_publisher.launch` file)
_____images
     |___000000.bin
     |___000001.bin
     |___000002.bin
     |...
_____pointclouds_data
     |___000000.label
     |___000001.label
     |___000002.label
     |...
_____camera_parameters.txt
_____groundtruth.hdf5
_____map.pcd
   
```

## How to Run

```
$ catkin build naverlabs_api
$ roslaunch naverlabs_api lidar_publisher.launch
```

Note that **you can choose the lidar type among three modes, i.e. `lidar0`, `lidar1`, and `both`.

## Applications

Patchwork                  |  Concept of our method (CZM & GLE)
:-------------------------:|:-------------------------:
![](img/patchwork_concept_resized.jpg) |  ![](img/patchwork.gif)





