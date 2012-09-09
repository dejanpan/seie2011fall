/*
 * DenseReconstruction.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: Karol Hausman
 */

#include "textureless_objects_tracking/DenseReconstruction.h"

DenseReconstruction::DenseReconstruction(pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr saved_cloud) {

	cloud_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>);
	pcl::copyPointCloud(*saved_cloud,*cloud_);
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	cloud_operational_.reset(new pcl::PointCloud<pcl::PointXYZLRegionF>);
    pcl::io::savePCDFile("Aafter_beg.pcd",*cloud_);

	planeSegmentation(cloud_,coefficients,*inliers);
	planeExtraction(cloud_,inliers,*cloud_operational_);
    pcl::io::savePCDFile("Aafter_plane.pcd",*cloud_operational_);

	cloud_normals_.reset(new pcl::PointCloud<pcl::Normal>);
	normalsEstimation(cloud_operational_,cloud_normals_);
	region_grow_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::copyPointCloud(*cloud_operational_,*region_grow_point_cloud_);

    pcl::io::savePCDFile("Aafter_normals.pcd",*cloud_operational_);

}

DenseReconstruction::~DenseReconstruction() {
	// TODO Auto-generated destructor stub
}

void DenseReconstruction::normalsEstimation(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud,pcl::PointCloud<pcl::Normal>::Ptr &normals){


	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::copyPointCloud(*cloud,*temp_cloud);


	 // Create a KD-Tree
	tree_.reset(new pcl::search::KdTree<pcl::PointXYZRGBA>);

	pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
	ne.setInputCloud(temp_cloud);
	ne.setSearchMethod(tree_);
	ne.setRadiusSearch(0.03);
//	ne.compute(*cloud_normals_);
	ne.compute(*normals);
}

void DenseReconstruction::boundaryEstimation(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input,const pcl::PointCloud<pcl::Normal>::Ptr &normals, pcl::PointCloud<pcl::Boundary> &boundaries){

	 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
	 pcl::copyPointCloud(*cloud_input,*cloud_input_temp);
	 pcl::BoundaryEstimation<pcl::PointXYZRGBA, pcl::Normal, pcl::Boundary> est;
	 est.setInputCloud (cloud_input_temp);
	 est.setInputNormals (normals);
	 est.setRadiusSearch (0.02);   // 2cm radius
	 est.setSearchMethod (tree_);
	 est.compute (boundaries);


}



void DenseReconstruction::activeSegmentation (const pcl::PointCloud<pcl::PointXYZLRegionF> &cloud_input,
    float search_radius,
    double eps_angle,
    int fp_index,
    pcl::PointIndices &indices_out)
{




	  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	  pcl::PointCloud<pcl::Boundary> boundary;


	  normalsEstimation(cloud_input.makeShared(),normals);

	  boundaryEstimation(cloud_input.makeShared(),normals,boundary);


	  std::cerr<<"Index of seed point is: "<<fp_index<<std::endl;
	  std::cerr<<"Curvature of seed point: "<<normals->points[fp_index].curvature<<std::endl;


	pcl::PointCloud<pcl::PointXYZRGBA> cloud_in;
	pcl::copyPointCloud(cloud_input,cloud_in);

	  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
	  tree->setInputCloud(cloud_in.makeShared());


  if (fp_index > cloud_in.points.size () || fp_index <0)
  {
    PCL_ERROR ("[pcl::activeSegmentation] Fixation point given is invalid\n");
    return;
  }
  if (boundary.points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Boundary map given was built for a different dataset (%zu) than the input cloud (%zu)!\n",
        boundary.points.size () , cloud_in.points.size ());
    return;
  }
  if (tree->getInputCloud ()->points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        tree->getInputCloud ()->points.size (), cloud_in.points.size ());
    return;
  }
  if (normals->points.size () != cloud_in.points.size ())
  {
    PCL_ERROR ("[pcl::activeSegmentation] Input Normals are for a different point cloud dataset (%zu) than the input cloud (%zu)!\n",
        normals->points.size (), cloud_in.points.size ());
    return;
  }
  std::vector<int> seed_queue;
  std::vector<bool> processed (cloud_in.size(), false);
  seed_queue.push_back (fp_index);
  indices_out.indices.push_back (fp_index);
  processed[fp_index] = true;
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  //process while there are valid seed points

  for(size_t seed_idx = 0; seed_idx < seed_queue.size();++seed_idx)
  {

    int curr_seed;
    curr_seed = seed_queue[seed_idx];

    // Search for seeds
    if (!tree->radiusSearch (curr_seed, search_radius, nn_indices, nn_distances))
      continue;
    //process all points found in the neighborhood
    bool stop_growing = false;
    size_t indices_old_size = indices_out.indices.size();
    for (size_t i=1; i < nn_indices.size (); ++i)
    {
      if (processed[nn_indices.at (i)])
        continue;
      if (boundary.points[nn_indices.at (i)].boundary_point != 0)
      {
        stop_growing=true;
        indices_out.indices.push_back (nn_indices.at (i));
        processed[nn_indices.at (i)] = true;
        break;
      }

      bool is_convex = false;
      pcl::PointXYZRGBA temp;
      temp.x = cloud_in.points[fp_index].x - cloud_in.points[nn_indices.at (i)].x;
      temp.y = cloud_in.points[fp_index].y - cloud_in.points[nn_indices.at (i)].y;
      temp.z = cloud_in.points[fp_index].z - cloud_in.points[nn_indices.at (i)].z;

      double dot_p = normals->points[nn_indices.at (i)].normal[0] * temp.x
          + normals->points[nn_indices.at (i)].normal[1] * temp.y
          + normals->points[nn_indices.at (i)].normal[2] * temp.z;

      dot_p = dot_p>1? 1:dot_p;
      dot_p = dot_p<-1 ? -1:dot_p;

      if ((acos (dot_p) > eps_angle*M_PI / 180))
        is_convex = true;


      if (is_convex)
      {
        indices_out.indices.push_back (nn_indices.at (i));
        processed[nn_indices.at (i)] = true;
      }
      else
        break;
    }//new neighbor

    if(!stop_growing && (indices_old_size != indices_out.indices.size()))
    {
      for (size_t j = indices_old_size-1; j < indices_out.indices.size(); ++j)
        seed_queue.push_back(indices_out.indices.at (j));
    }
  }//new seed point

}














/*
 * it has a small bug that is deleting the regions some times.
 */
void DenseReconstruction::addSideWall(std::vector<pcl::PointIndices::Ptr> &clusters_input){







	  for (int i=0;i<clusters_input.size();i++){

		  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZLRegionF>);


		  pcl::copyPointCloud(*region_grow_point_cloud_,*clusters_input[i],*cloud_temp);
		  clusters_vec_point_cloud.push_back(cloud_temp);
	  }



for(int how_many=0;how_many<6;how_many++){

//	  std::vector<pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr> clusters_vec_point_cloud;
	  std::vector<pcl::PointCloud<pcl::Normal>::Ptr> clusters_vec_normals;
	  std::vector<pcl::PointCloud<pcl::Boundary> > clusters_vec_boundaries;
//	  std::vector<pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr> clusters_vec_only_boudaries;
	  std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters_vec_temp;

	  clusters_vec_only_boudaries.clear();


	  for (int i=0;i<clusters_vec_point_cloud.size();i++){
		  pcl::PointCloud<pcl::Normal>::Ptr normals_temp(new pcl::PointCloud<pcl::Normal>);
		  pcl::PointCloud<pcl::Boundary> boundary_temp;


		  normalsEstimation(clusters_vec_point_cloud[i],normals_temp);
		  clusters_vec_normals.push_back(normals_temp);

		  boundaryEstimation(clusters_vec_point_cloud[i],normals_temp,boundary_temp);
		  clusters_vec_boundaries.push_back(boundary_temp);
	  }

	  for (int i=0;i<clusters_vec_boundaries.size();i++){

		  pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr cloud_temp_boudary(new pcl::PointCloud<pcl::PointXYZLRegionF>);


		  for(int j = 0; j < clusters_vec_boundaries[i].size(); j++) {
				 if (clusters_vec_boundaries[i].points[j].boundary_point != 0) {
					 cloud_temp_boudary->push_back(clusters_vec_point_cloud[i]->points.at(j));
				 }
		  }

		  clusters_vec_only_boudaries.push_back(cloud_temp_boudary);

	  }

	  for (int i=0;i<clusters_vec_only_boudaries.size();i++){

			 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGBA>);
			 pcl::copyPointCloud(*clusters_vec_only_boudaries[i],*cloud_temp);
			 clusters_vec_temp.push_back(cloud_temp);
	  }

	  float score_array[clusters_vec_temp.size()][clusters_vec_temp.size()];

	  for(int i=0;i<clusters_vec_temp.size();i++){

		  for(int j=0;j<clusters_vec_temp.size();j++){
			  score_array[i][j]=0;
		  }

	  }

	  for (int i=0;i<clusters_vec_temp.size();i++){

		  for (int k=0;k<clusters_vec_temp.size();k++){
				if(k==i){
//					score_array[i][k]=0;
					continue;
				}
				for(int j=0;j<clusters_vec_temp[i]->size();j++){

					pcl::PointXYZRGBA searchPointTemp=clusters_vec_temp[i]->points[j];


					pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;


					if(clusters_vec_temp[k]->size()<1)
						continue;
					kdtree.setInputCloud(clusters_vec_temp[k]);

					std::vector<int> pointIdxRadiusSearch;
					std::vector<float> pointRadiusSquaredDistance;
					float radius=0.005;

					if (kdtree.radiusSearch(searchPointTemp, radius, pointIdxRadiusSearch,
							pointRadiusSquaredDistance) > 0) {

						score_array[i][k]=score_array[i][k]+1;


						}

				  }
				score_array[i][k]=score_array[i][k]/clusters_vec_temp[i]->size();
		  }

	  }

	  std::cout<<"Table: "<<std::endl;
	  for(int i=0;i<clusters_vec_temp.size();i++){

		  for(int j=0;j<clusters_vec_temp.size();j++){
			  std::cout<<score_array[i][j]<<" ";
		  }
		  std::cout<<std::endl;

	  }

	  //merging
	  std::vector<int> is_to_del;

	  for(int i=0;i<clusters_vec_temp.size();i++){

		  for(int j=0;j<clusters_vec_temp.size();j++){
			  if(score_array[i][j]>0.4){
				  clusters_vec_point_cloud[j]->points.insert(clusters_vec_point_cloud[j]->points.end(), clusters_vec_point_cloud[i]->points.begin(), clusters_vec_point_cloud[i]->points.end());
					is_to_del.push_back(i);


			  }

		  }

	  }

	  if(is_to_del.size()>0){
	  std::cout<<"IS TO DEL SIZE: "<<is_to_del.size()<<std::endl;
		std::sort(is_to_del.begin(), is_to_del.end());
		is_to_del.erase(std::unique(is_to_del.begin(), is_to_del.end()), is_to_del.end());
	  }



		for(int i=0;i<is_to_del.size();i++){

			std::cout<<"IS TO DEL: "<<is_to_del[i]<<std::endl;
//			is_to_del[i]=is_to_del[i]-i;
			clusters_vec_point_cloud.erase(clusters_vec_point_cloud.begin()+is_to_del[i]);

		}






}


//	  std::cerr<<"size of ADDSIDEWALL pcls: "<<clusters_vec_point_cloud.size()<<std::endl;
//	  std::cerr<<"size of ADDSIDEWALL normals: "<<clusters_vec_normals.size()<<std::endl;




}


void DenseReconstruction::mergeClusters(std::vector<pcl::PointIndices::Ptr> &clusters_input){

	bool mergeCluster=false;
	float region=-1;

	int points_number=0;

	for(int i=0;i<clusters_input.size();i++){
		points_number=+points_number+clusters_input[i]->indices.size();

		std::cout<<"cluster number respectively: "<<i<<std::endl;

		if(mergeCluster){
			for(int point=0;point<clusters_input[i]->indices.size();point++){
				cloud_operational_->points[clusters_input[i]->indices[point]].reg=region;
			}
			std::cerr<<"Merging cluster. Region number= "<<region<<std::endl;
			mergeCluster=false;
			region=-1;

			continue;
		}


		for(int cluster_point=0;cluster_point<clusters_input[i]->indices.size();cluster_point++){

			if(cloud_operational_->points[clusters_input[i]->indices[cluster_point]].f!=0){
				std::cerr<<"Merging cluster beginning."<<std::endl;
//				if(cloud_operational_->points[clusters_input[i]->indices[cluster_point]].reg!=0){
//					std::cout<<std::endl;
//					std::cout<<"THIS REGION WAS CONSIDERED! "<<std::endl;
//					std::cout<<std::endl;}
//					region=10;
//				else
					region=cloud_operational_->points[clusters_input[i]->indices[cluster_point]].f;
				mergeCluster=true;
				i--;
				break;

			}


		}




	}



	std::vector<int> is_to_del;
	bool delete_flag=false;
	float region_number=0;
	std::vector<float>region_numbers;
	for(int i=0;i<clusters_input.size();i++){

		if(cloud_operational_->points[clusters_input[i]->indices[1]].reg!=0){
			region_number=cloud_operational_->points[clusters_input[i]->indices[1]].reg;
			for(int region=0;region<region_numbers.size();region++)
				if(region_number==region_numbers[region]){
					delete_flag=true;
					is_to_del.push_back(i);
				}
			if (delete_flag){
				delete_flag=false;
				continue;
			}

			for(int j=i+1;j<clusters_input.size();j++){
				if(cloud_operational_->points[clusters_input[j]->indices[1]].reg==region_number){
//					std::cerr<<"REGION NUMBER: "<<region_number<<std::endl;
//					std::cerr<<"CLUSTERS SIZE BEFORE: "<<clusters_input[i]->indices.size()<<std::endl;
					clusters_input[i]->indices.insert(clusters_input[i]->indices.end(), clusters_input[j]->indices.begin(), clusters_input[j]->indices.end());
					region_numbers.push_back(region_number);

				}

			}



		}


	}


	//part to have merged clusters

	std::sort(is_to_del.begin(), is_to_del.end());
	is_to_del.erase(std::unique(is_to_del.begin(), is_to_del.end()), is_to_del.end());




	for(int i=0;i<is_to_del.size();i++){

		is_to_del[i]=is_to_del[i]-i;
		clusters_input.erase(clusters_input.begin()+is_to_del[i]);

	}






	std::cout<<"number of point in VECTORS: "<<points_number<<std::endl;
	std::cout<<"number of points in the PCL: "<<cloud_operational_->points.size()<<std::endl;








}


void DenseReconstruction::extractEuclideanClustersCurvature(std::vector<pcl::PointIndices::Ptr> &clusters){


	float tolerance=0.01;//0.01radius of KDTree radius search in region growing in meters
	double eps_angle=35*M_PI/180;//35, 10
	double max_curvature=0.1;//0.1  //max value of the curvature of the point form which you can start region growing
	unsigned int min_pts_per_cluster = 1;
	unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ()	;
    // Create a bool vector of processed point indices, and initialize it to false
    std::vector<bool> processed (region_grow_point_cloud_->size (), false);

    //create a random copy of indices
    std::vector<int> indices_rnd(region_grow_point_cloud_->size ());
    for(unsigned int i=0;i<indices_rnd.size();++i)
    {
      indices_rnd[i]=i;
    }
    //uncommnet myrandom part if you want indices to be truely
//    if(use_srand == 1)
//    {
//      std::random_shuffle(indices_rnd.begin(), indices_rnd.end(), myrandom);
//      ROS_ERROR("REAL RANDOM");
//    }
//    else
      std::random_shuffle(indices_rnd.begin(), indices_rnd.end());
    std::cerr<<"Processed size: "<<processed.size()<<std::endl;
    std::vector<int> index_lookup(indices_rnd.size());
    for(unsigned int i= 0; i<indices_rnd.size();++i)
      index_lookup[indices_rnd[i]] = i;

    std::vector<int> nn_indices;
    std::vector<float> nn_distances;
    // Process all points in the indices vector
    for (size_t i = 0; i < indices_rnd.size (); ++i)
    {

      if (processed[i] || cloud_normals_->points[indices_rnd[i]].curvature > max_curvature)
      {
        /*if(normals.points[indices_rnd[i]].curvature > max_curvature)
          std::cerr<<"Curvature of point skipped: "<<normals.points[indices_rnd[i]].curvature<<std::endl;*/
        continue;
      }
      pcl::PointIndices::Ptr seed_queue(new pcl::PointIndices());
      int sq_idx = 0;
      seed_queue->indices.push_back (indices_rnd[i]);

      processed[i] = true;

      while (sq_idx < (int)seed_queue->indices.size ())
      {
        // Search for sq_idx
        if (!tree_->radiusSearch (seed_queue->indices[sq_idx], tolerance, nn_indices, nn_distances))
        {
          sq_idx++;
          continue;
        }

        for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
        {
          // std::cerr<<nn_indices[j]<<std::endl;
          if (processed[index_lookup[nn_indices[j]]])                             // Has this point been processed before ?
            continue;

          // [-1;1]
          double dot_p =
			  cloud_normals_->points[indices_rnd[i]].normal[0] * cloud_normals_->points[nn_indices[j]].normal[0] +
			  cloud_normals_->points[indices_rnd[i]].normal[1] * cloud_normals_->points[nn_indices[j]].normal[1] +
			  cloud_normals_->points[indices_rnd[i]].normal[2] * cloud_normals_->points[nn_indices[j]].normal[2];
          if ( fabs (acos (dot_p)) < eps_angle )
          {
            processed[index_lookup[nn_indices[j]]] = true;
            seed_queue->indices.push_back (nn_indices[j]);
          }
        }
        sq_idx++;
      }

      // If this queue is satisfactory, add to the clusters
      if (seed_queue->indices.size () >= min_pts_per_cluster && seed_queue->indices.size () <= max_pts_per_cluster)
      {
        seed_queue->header = region_grow_point_cloud_->header;
        clusters.push_back (seed_queue);
      }
    }
    int unprocessed_counter = 0;
    for(unsigned int i =0; i<processed.size(); ++i)
    {
      if(processed[i] == false)
      {
        //std::cerr<<"Indice not processed at " <<i<<" : "<<indices_rnd[i]<<std::endl;
        unprocessed_counter++;
      }
    }
    //std::cerr<<"Number of unprocessed indices: "<<unprocessed_counter<<std::endl;





}

void DenseReconstruction::regionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &segments){

	pcl::RegionGrowingRGB<pcl::PointXYZRGBA> region_growing;
	region_growing.setCloud(region_grow_point_cloud_);
//    pcl::io::savePCDFile("to_use.pcd",*region_grow_point_cloud_);

	region_growing.setNormals(cloud_normals_);
	region_growing.setNeighbourSearchMethod(tree_);


//	region_growing.setResidualTest(true);
//	region_growing.setResidualThreshold(0.1);

	region_growing.setCurvatureTest(true);
	region_growing.setCurvatureThreshold(0.0001);

//	region_growing.setSmoothMode(true);
//	region_growing.setSmoothnessThreshold(80 * M_PI / 180);

//	region_growing.setPointColorThreshold(10.0);
//	region_growing.setRegionColorThreshold(200.0);

//	region_growing.setDistanceThreshold (10);

	region_growing.segmentPoints();
//	std::vector<int> vec=region_growing.getSegmentFromPoint(5);
//	std::cout<<"VEC SIZE: "<<vec.size()<<std::endl;
	segments = region_growing.getColoredCloud();


}

void DenseReconstruction::planeSegmentation(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud,
		pcl::ModelCoefficients &coefficients, pcl::PointIndices &inliers) {

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud,*cloud_temp);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.01);
	seg.setInputCloud(cloud_temp);
	seg.segment(inliers, coefficients);
}
void DenseReconstruction::planeExtraction(const pcl::PointCloud<pcl::PointXYZLRegionF>::Ptr &cloud_input,pcl::PointIndices::Ptr &inliers,pcl::PointCloud<pcl::PointXYZLRegionF> &cloud_output){

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input_temp(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI> cloud_output_temp;

	pcl::copyPointCloud(*cloud_input,*cloud_input_temp);

	pcl::ExtractIndices<pcl::PointXYZI> extract;
	extract.setInputCloud (cloud_input_temp);
	extract.setIndices (inliers);
	extract.setNegative (true);
	extract.filter (cloud_output_temp);
	pcl::copyPointCloud(cloud_output_temp,cloud_output);

	for (int i=0;i<cloud_input->points.size();i++){
		if(cloud_input->points[i].f!=0){

			pcl::PointXYZI searchPointTemp;
			searchPointTemp.x=cloud_input->points[i].x;
			searchPointTemp.y=cloud_input->points[i].y;
			searchPointTemp.z=cloud_input->points[i].z;
			searchPointTemp.intensity=cloud_input->points[i].f;

								pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;

								kdtree.setInputCloud(cloud_output_temp.makeShared());

								std::vector<int> pointIdxRadiusSearch;
								std::vector<float> pointRadiusSquaredDistance;
								float radius=0.002;

								if (kdtree.nearestKSearch(searchPointTemp, 1, pointIdxRadiusSearch,
										pointRadiusSquaredDistance) > 0) {

									cloud_output.points[pointIdxRadiusSearch[0]].f=cloud_input->points[i].f;

									}


		}
	}

}

