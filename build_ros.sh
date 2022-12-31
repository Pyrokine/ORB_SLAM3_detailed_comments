echo "Building ROS nodes"

cd Examples/ROS/ORB_SLAM3_dense
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j`nproc`
