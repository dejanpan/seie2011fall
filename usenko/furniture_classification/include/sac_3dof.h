#ifndef SAC_3DOF_H_
#define SAC_3DOF_H_

#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

namespace pcl
{

template<typename PointT>
  class SampleConsensusModel3DOF : public SampleConsensusModel<PointT>
  {
    using SampleConsensusModel<PointT>::input_;
    using SampleConsensusModel<PointT>::indices_;
    using SampleConsensusModel<PointT>::samples_radius_search_;

  public:
    typedef typename SampleConsensusModel<PointT>::PointCloud PointCloud;
    typedef typename SampleConsensusModel<PointT>::PointCloudPtr PointCloudPtr;
    typedef typename SampleConsensusModel<PointT>::PointCloudConstPtr PointCloudConstPtr;

    SampleConsensusModel3DOF(const PointCloudConstPtr & cloud) :
      SampleConsensusModel<PointT> (cloud), eps(0.01)
    {
      setInputCloud(cloud);
    }

    PointCloudConstPtr getTarget() const
    {
      return target;
    }

    void setTarget(PointCloudConstPtr target)
    {
      this->target = target;
    }

    virtual bool computeModelCoefficients(const std::vector<int> & samples, Eigen::VectorXf & model_coefficients)
    {

      PointT input_point = input_->points[samples[0]];

      // Select points with the same height
      std::vector<int> idx;

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(target);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(input_point.z - eps, input_point.z + eps);
      pass.filter(idx);


      int rand_idx = rand() % idx.size();

      PointT target_point = target->points[idx[rand_idx]];

      model_coefficients[0] = target_point.x - input_point.x;
      model_coefficients[1] = target_point.y - input_point.y;
      model_coefficients[2] = atan2(target_point.normal_y, target_point.normal_x) - atan2(input_point.normal_y,
                                                                                          input_point.normal_x);

    }

    virtual void optimizeModelCoefficients(const std::vector<int> & inliers,
                                           const Eigen::VectorXf & model_coefficients,
                                           Eigen::VectorXf & optimized_coefficients)
    {
      optimized_coefficients = model_coefficients;
    }

    virtual void getDistancesToModel(const Eigen::VectorXf & model_coefficients, std::vector<double> & distances)
    {
      PointCloud transformed_input;

      Eigen::Affine3f transform;
      transform.translate(Eigen::Vector3f(model_coefficients[0], model_coefficients[1], 0));
      transform.rotate(Eigen::AngleAxisf(model_coefficients[2], Eigen::Vector3f(0, 0, 1)));

      pcl::transformPointCloudWithNormals(*input_, transformed_input, transform);

    }

    virtual void selectWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold, std::vector<
        int> & inliers)
    {
    }

    virtual int countWithinDistance(const Eigen::VectorXf & model_coefficients, const double threshold)
    {
    }

    virtual void projectPoints(const std::vector<int> & inliers, const Eigen::VectorXf & model_coefficients,
                               PointCloud & projected_points, bool copy_data_fields = true)
    {
    }

    virtual bool doSamplesVerifyModel(const std::set<int> & indices, const Eigen::VectorXf & model_coefficients,
                                      const double threshold)
    {
    }

    virtual SacModel getModelType() const
    {
    }

    inline virtual bool isModelValid(const Eigen::VectorXf &model_coefficients)
    {
      return true;
    }
    virtual bool isSampleGood(const std::vector<int> &samples) const
    {
      return (input_->points[samples[0]].normal_x != 0) && (input_->points[samples[0]].normal_y != 0);
    }

  protected:
    PointCloudConstPtr target;

    float eps;

  };
}

#endif
