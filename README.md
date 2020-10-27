---
layout: default
title: Home
nav_order: 1
---
![icp_match001_wish](cs/images/accelerated_small.png)
--------------------------------------------------------------------------------
## ICP outcome
[icp_match001_wish](https://github.com/wishchin/icp_match001_wish) gives a classical icp algorithm, it can  implement less than 100 ms! 

# ML solutions in Framework

Face Detection                                                                                                                 | Face Mesh                                                                                                       | Iris                                                                                                      | Hands                                                                                                      | Pose                                                                                                      | Hair Segmentation
:----------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------: | :--------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------: | :---------------:
[![face_detection](docs/images/mobile/face_detection_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/face_detection) | [![face_mesh](docs/images/mobile/face_mesh_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/face_mesh) | [![iris](docs/images/mobile/iris_tracking_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/iris) | [![hand](docs/images/mobile/hand_tracking_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/hands) | [![pose](docs/images/mobile/pose_tracking_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/pose) | [![hair_segmentation](docs/images/mobile/hair_segmentation_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/hair_segmentation)

Object Detection                                                                                                                     | Box Tracking                                                                                                                | Instant Motion Tracking                                                                                                                               | Objectron                                                                                                             | KNIFT
:----------------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------------: | :---------------------------------------------------------------------------------------------------------------------------------------------------: | :-------------------------------------------------------------------------------------------------------------------: | :---:
[![object_detection](docs/images/mobile/object_detection_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/object_detection) | [![box_tracking](docs/images/mobile/object_tracking_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/box_tracking) | [![instant_motion_tracking](docs/images/mobile/instant_motion_tracking_android_small.gif)](https://google.github.io/mediapipe/solutions/instant_motion_tracking) | [![objectron](docs/images/mobile/objectron_chair_android_gpu_small.gif)](https://google.github.io/mediapipe/solutions/objectron) | [![knift](docs/images/mobile/template_matching_android_cpu_small.gif)](https://google.github.io/mediapipe/solutions/knift)

<!-- []() in the first cell is needed to preserve table formatting in GitHub Pages. -->
<!-- Whenever this table is updated, paste a copy to solutions/solutions.md. -->

[]()                                                                                      | Android | iOS | Desktop | Python |
:---------------------------------------------------------------------------------------- | :-----: | :-: | :-----: | :----: | 
[Face Detection](https://google.github.io/mediapipe/solutions/face_detection)                   | ✅       | ✅   | ✅       |   
[Face Mesh](https://google.github.io/mediapipe/solutions/face_mesh)                             | ✅       | ✅   | ✅       |     
[Iris](https://google.github.io/mediapipe/solutions/iris)                                       | ✅       | ✅   | ✅       |       
[Hands](https://google.github.io/mediapipe/solutions/hands)                                     | ✅       | ✅   | ✅       |       
[Pose](https://google.github.io/mediapipe/solutions/pose)                                       | ✅       | ✅   | ✅       | 
[Hair Segmentation](https://google.github.io/mediapipe/solutions/hair_segmentation)             | ✅       |     | ✅       |     
[Object Detection](https://google.github.io/mediapipe/solutions/object_detection)               | ✅       | ✅   | ✅       |    
[Box Tracking](https://google.github.io/mediapipe/solutions/box_tracking)                       | ✅       | ✅   | ✅       |    
[Instant Motion Tracking](https://google.github.io/mediapipe/solutions/instant_motion_tracking) | ✅       |     |         |     
[Objectron](https://google.github.io/mediapipe/solutions/objectron)                             | ✅       |     |         |      
[KNIFT](https://google.github.io/mediapipe/solutions/knift)                                     | ✅       |     |         |     
[AutoFlip](https://google.github.io/mediapipe/solutions/autoflip)                               |         |     | ✅       |    
[MediaSequence](https://google.github.io/mediapipe/solutions/media_sequence)                    |         |     | ✅       |      
[YouTube 8M](https://google.github.io/mediapipe/solutions/youtube_8m)                           |         |     | ✅       |      

See also
[MediaPipe Models and Model Cards](https://google.github.io/mediapipe/solutions/models)
for ML models released in icp_match001_wish.

---
outcome
--------------------------------------------------------------------------------
![icp_match001_wish](docs/outcome/outcome.png)

time
--------------------------------------------------------------------------------
helllo,wishchin! 
filter time: 13 ms

Applying this rigid transformation to: cloud_in -> cloud_icp

Rotation matrix :
    |  0.924 -0.383  0.000 | 
R = |  0.383  0.924  0.000 | 
    |  0.000  0.000  1.000 | 
    
Translation vector :

t = <  0.000,  0.000,  0.400 >

Applied 16 ICP iteration(s) in 372 ms
