#!/bin/bash
 #	This script will search and replace all instances of pcl with pcl16 for the fuerte pcl16 namespace hack
 # Author: Ross Kidson

 for f in `find . -name \*.cpp -o -name \*.h -o -name \*.hpp`;
 do
 if egrep -rc 'pcl::|namespace\ pcl|pcl_gpu|pcl_cuda|include\ <pcl\/|include\ "pcl\/|pcl_round|pcl_lrint|pcl_sleep|PCLAPI|aligned_malloc|aligned_free|POINT_CLOUD_' $f > 0 ;
 then
  echo "Instances of pcl found in $f, replacing with pcl16"
  sed -i 's/pcl::/pcl16::/g' $f
  sed -i 's/namespace\ pcl/namespace\ pcl16/g' $f
  sed -i 's/pcl_gpu::/pcl_gpu16::/g' $f
  sed -i 's/pcl_cuda::/pcl_cuda16::/g' $f
  sed -i 's/include\ <pcl\//include <pcl16\//g' $f
  sed -i 's/include\ "pcl\//include "pcl16\//g' $f
  sed -i 's/pcl_round/pcl16_round/g' $f
  sed -i 's/pcl_lrint/pcl16_lrint/g' $f
  sed -i 's/pcl_sleep/pcl16_sleep/g' $f
  sed -i 's/PCLAPI/PCL16API/g' $f
  sed -i 's/aligned_malloc/pcl16_aligned_malloc/g' $f
  sed -i 's/aligned_free/pcl16_aligned_free/g' $f
  sed -i 's/POINT_CLOUD_/PCL16_POINT_CLOUD_/g' $f
 fi
done

for f in `find ./ -name CMakeLists.txt`;
do
  echo "Replacing pcl with pcl16 in CMakeLists.txt file  $f"
  sed -i 's/include\/pcl/include\/pcl16/g' $f
done
