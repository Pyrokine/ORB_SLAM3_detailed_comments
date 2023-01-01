echo "Building ROS nodes"
(
  cd Examples/ROS/ORB_SLAM3_dense || exit
  mkdir build
  cd build || exit
  cmake .. -DROS_BUILD_TYPE=Release
  make -j"$(nproc)"
)
