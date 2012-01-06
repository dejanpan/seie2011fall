#ifndef _CAM_STATE_H_
#define _CAM_STATE_H_

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace people {
	class CamState;
	typedef boost::shared_ptr<CamState> CamStatePtr;

	class CamState
	{
	public:
		CamState():x_(0.0), y_(0.0), z_(0.0), yaw_(0.0), pitch_(0.0), roll_(0.0), timesec_(0.0) {};
		virtual ~CamState(){};
		
		btMatrix3x3 getRot() 
		{
			btMatrix3x3 ret;
			ret.setRPY(roll_, pitch_, yaw_);
			return ret;
		};

		btVector3 getTrans() 
		{
			btVector3 ret(x_, y_, z_);
			return ret;
		};

		inline virtual CamStatePtr clone() 
		{
			CamStatePtr ret = boost::make_shared<CamState>(CamState());
			ret->x_ = x_; 	ret->y_ = y_; 	ret->z_ = z_;
			ret->pitch_ = pitch_;	ret->yaw_ = yaw_;	ret->roll_ = roll_;
			ret->timesec_ = timesec_;
			return ret;
		}
		
		void print()
		{
			std::cout << "camera at " << timesec_ << " : " 
						<< " x " << x_
						<< " y " << y_
						<< " z " << z_
						<< " yaw " << yaw_
						<< " pitch " << pitch_
						<< " roll " << roll_ << std::endl;
		};

		double x_;
		double y_;
		double z_;

		double yaw_;
		double pitch_;
		double roll_;

		double timesec_;
	};
};
#endif // _CAM_STATE_H_
