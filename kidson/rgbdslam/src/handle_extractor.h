/*
 * HandleExtractor.h
 *
 *  Created on: 31.05.2012
 *      Author: ross
 */

#ifndef HANDLEEXTRACTOR_H_
#define HANDLEEXTRACTOR_H_

#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGBNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointCloudNormal;
typedef pcl::search::KdTree<Point> KdTree;
typedef KdTree::Ptr KdTreePtr;
//typedef pcl::KdTree<Point>::Ptr KdTreePtr;
//typedef typename pcl::search::Search<Point>::Ptr KdTreePtr;

class HandleExtractor {
public:
	HandleExtractor();
	void extractHandles(PointCloudNormal::Ptr& cloudInput, std::vector<int>& handles);
	virtual ~HandleExtractor();
};

#endif /* HANDLEEXTRACTOR_H_ */
