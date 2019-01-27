# Dependencies
- python 3.5.6
- Carla 0.8.4
- numpy
- pil

# Usage
- change to Carla installation folder and run Carla in server mode：```./CarlaUE4.sh -carla-server -windowed -ResX=400 -ResY=300```
- generate 40000 frames from 40 random scenes: 
```python generate.py -o carla_kitti```
- use ```python generate.py -h``` to show help message

# Output Directory Structure
- carla_kitti
    - episode_[i]: i=[0, 39]，40 scenes (random weather and initial points)
        - Camera[2/3][RGB/Depth]: 2 for left, 3 for right
            - [j].[png/pfm]: j=[0, 99], 40000 frames in total

# Setup
- stereo cameras setup:
    - [KITTI Sensor Setup](http://www.cvlibs.net/datasets/kitti/setup.php): Cam 2 and Cam 3
    - baseline: 0.54 m
- camera paeameters:
    - fu = 718.856
	- fv = 718.856
	- cu = 607.1928
	- cv = 185.2157
- FOV:
    - [Field of view in video games](https://en.wikipedia.org/wiki/Field_of_view_in_video_games)
    - [camera – 水平,垂直和对角线视场之间的关系](https://codeday.me/bug/20190123/557996.html)
        - HFOV = 2 * arctan( width / 2f ) = 2 * arctan(cu / fu) = 80.373247
- format:
    - use .png for RGB cameras
    - use .pfm for depth cameras
        - io code reference: [Sceneflow Datasets](https://lmb.informatik.uni-freiburg.de/resources/datasets/SceneFlowDatasets.en.html) - Data formats and organization - 8
        - disparity map: 32 bit float, unit pixels

# Tutorials for Carla
- [Getting started with CARLA](https://carla.readthedocs.io/en/latest/getting_started/): latest version
- [Getting started with CARLA](https://carla.readthedocs.io/en/stable/getting_started/): stable version



