Collection of common ARPG applications including:

SensorViewer
logtool
LogViewer

and others such as:

CamToNode
PoseToNode
gps2cartesian


It is likely we will find another more cogent home for these.

###Use SensorViewer
1: Use SensorViewer to visualize a log file. Say I have a log file, which has camera data and imu data. Then to visualize it using SensorViewer, go tothe sensorviewer executable file folder, run
```
./SensorViewer -cam log://path_to_log_file -imu log://path_to_log_file
```
for example, I have a log file in path `/home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/test.log`, I need run
```
./SensorViwer -cam log:///home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/test.log -imu log:///home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/test.log
```
2: Use SensorViwer to visualize data from equipement.  Say you have realsense camera and imu 3DM-GX5-15 from microstrain. Make sure they work first, then connect them to the computer, run
```
./SensorViewer -cam realsense2:[rgb=1, depth=0, emitter=0, ir0=1, ir1=0] -imu microstrain:///dev/ttyACM0
```
`rgb=1` means enable realsense rgb camera, if it equals 0 then it means rgb is disabled.
3: Click the `log` button after you see the window of SensorViewer, you can save the data as a log file.
###Use logtool
Once you have a log file, you may want to extract the data in it such as image and imu. Go to logtool executable file, The way to extract the image, for example, is 
```
./logtool -extract_images -in path_to_log_file -out the_directory_to_save_image
```
Say you have log file in path `/home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/proto.log` and you want to save the image in directory `/home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/log1_data_extraction/image/` then simply run
```
./logtool -extract_images -in /home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/proto.log -out /home/zhaozhong/dataset/arpg_realsense_3DM-GX5-15/log1_data_extraction/image/
```
You can also extract the imu
```
./logtool -extract_imu -in path_to_log_file -out the_directory_to_save_imu
```
Other functions please check the code.
The data extracted may not be in the format you want then please modify the code by yourself, it won't be difficult.
