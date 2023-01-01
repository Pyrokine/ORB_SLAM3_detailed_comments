echo "Configuring and building Thirdparty/DBoW2 ..."
(
  cd Thirdparty/DBoW2 || exit
  mkdir build
  cd build || exit
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"$(nproc)"
)

echo "Configuring and building Thirdparty/g2o ..."
(
  cd Thirdparty/g2o || exit
  mkdir build
  cd build || exit
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"$(nproc)"
)

echo "Configuring and building Thirdparty/Sophus ..."
(
  cd Thirdparty/Sophus || exit
  mkdir build
  cd build || exit
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j"$(nproc)"
)

echo "Uncompress vocabulary ..."
(
  cd Vocabulary || exit
  tar -xf ORBvoc.txt.tar.gz
)

echo "Configuring and building ORB_SLAM3 ..."
(
  mkdir build
  cd build || exit
  cmake .. -DCMAKE_BUILD_TYPE=Release
  make -j8
)
