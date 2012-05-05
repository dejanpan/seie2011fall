/*
 * test.cpp
 *
 *  Created on: May 5, 2012
 *      Author: vsu
 */

#include <training.h>
#include <gtest/gtest.h>

// Runs Clustering with the same number of points as number of clusters
// Checks if returned cluster centers are the same as points
TEST(ClusteringTest, SameNumberOfPointsAndClusters)
{

	std::vector<featureType> features;
	std::vector<featureType> cluster_centers;
	std::vector<int> cluster_labels;
	int num_clusters = 10;

	for (int i = 0; i < num_clusters; i++) {
		featureType feature;

		for (int j = 0; j < featureLength; j++) {
			feature.histogram[j] = (float) rand() / (float) RAND_MAX;
		}
		features.push_back(feature);
	}

	cluster_features(features, num_clusters, cluster_centers, cluster_labels);

	for (int i = 0; i < num_clusters; i++) {
		featureType feature = features[i];
		featureType cluster_center = cluster_centers[cluster_labels[i]];
		for (int j = 0; j < featureLength; j++) {
			EXPECT_FLOAT_EQ(feature.histogram[j], cluster_center.histogram[j])
<<			"Feature and Cluster Center differ at index " << j;
		}
	}
}

TEST(ClusteringTest, SymetricDelta)
{

	std::vector<featureType> features;
	std::vector<featureType> cluster_centers;
	std::vector<int> cluster_labels;
	int num_clusters = 2;
	float delta = 0.01;

	featureType feature;

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 0;
	}
	features.push_back(feature);

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 0 - delta;
	}
	features.push_back(feature);

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 0 + delta;
	}
	features.push_back(feature);

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 1;
	}
	features.push_back(feature);

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 1 - delta;
	}
	features.push_back(feature);

	for (int j = 0; j < featureLength; j++) {
		feature.histogram[j] = 1 + delta;
	}
	features.push_back(feature);

	cluster_features(features, num_clusters, cluster_centers, cluster_labels);

	for (int i = 0; i < 3; i++) {
		featureType cluster_center = cluster_centers[cluster_labels[i]];
		for (int j = 0; j < featureLength; j++) {
			EXPECT_FLOAT_EQ(0.0f, cluster_center.histogram[j])
<<			"Cluster Center is not equal to 0 at index " << j;
		}
	}

	for (int i = 3; i < 6; i++) {
		featureType cluster_center = cluster_centers[cluster_labels[i]];
		for (int j = 0; j < featureLength; j++) {
			EXPECT_FLOAT_EQ(1.0f, cluster_center.histogram[j])
			<< "Cluster Center is not equal to 1 at index " << j;
		}
	}
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}