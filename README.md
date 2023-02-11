# ros2_image_benchmark

This repo provides some tests to understand the latency of image handling with ROS2 and OpenCV.


## Build the container
```
git clone https://github.com/maxc303/ros2_image_benchmark.git

docker compose build
docker compose up -d
docker exec -it ros2_image_benchmark bash

# Build the Package
colcon build
source install/setup.bash
```

## Test BGR and YUV420 conversion with OpenCV
Test the latency of conversion between IYUV420 and BGR of an image:

```
./install/image_latency_benchmark/lib/image_latency_benchmark/test_yuv_bgr_conversion <optional: test_image_path>

# OR

ros2 run image_latency_benchmark test_yuv_bgr_conversion  <optional: test_image_path>
```

Example output:
(Tested on a pc with i5-7600K)
```
No test image specified. Test conversion on grey image.
Test BGR<->YUV conversion with w=1080, h=1920
 Tested conversion on 100 images.
 - BGR to YUV_I420 time = 0.71ms.
 - YUV_I420 to BGR time = 0.69ms.
Test BGR<->YUV conversion with w=2160, h=3840
 Tested conversion on 100 images.
 - BGR to YUV_I420 time = 2.8ms.
 - YUV_I420 to BGR time = 2.75ms.
```

## Test image encode with OpenCV (JPG and PNG)
Test the latency of cv::imencode, which is used in [image_transport](https://github.com/ros-perception/image_transport_plugins/blob/humble/compressed_image_transport/src/compressed_publisher.cpp) for image compression .
```
./install/image_latency_benchmark/lib/image_latency_benchmark/test_cv_imencode <test_image_path>

# OR

ros2 run image_latency_benchmark test_cv_imencode <test_image_path>
```
Example output:

Note, the compresssion ratio depends on the image content.
```
Running test on test image: "ros2_image_benchmark/test_data/test_image_4k.jpg"
Test OpenCV imencode conversion with w=3840, h=2160, bgr8 raw data size = 24.8832 MB
Quality level=10, compressed data size=0.175301 MB, compression rate=0.00704495, avg time=21.1ms.
Quality level=20, compressed data size=0.222357 MB, compression rate=0.00893603, avg time=21.4ms.
Quality level=30, compressed data size=0.269238 MB,
............
```

## Test ROS2 Image Msg publish->subscribe latency 
The test is used to understand the latency to subscribe image message.
Test parameters can be configured in `config/pub_sub_test_params.yaml`. `colcon build` again to install the params file.

### Intra-process pub-sub
```
# Launch the publisher and subscriber from the same process
ros2 launch image_latency_benchmark composite_pub_sub.launch.py
```

### Inter-process pub-sub
```
# Launch the publisher
ros2 launch image_latency_benchmark pub_test.launch.py

# Launch the subscriber from another terminal
ros2 launch image_latency_benchmark sub_test.launch.py
```


