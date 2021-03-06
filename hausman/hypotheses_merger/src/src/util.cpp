#include <common/util.h>

namespace people{
	bool in_any_rect(const std::vector<cv::Rect> &rts, const cv::Point2f &pt)
	{
		for(size_t i = 0; i < rts.size(); i++) {
			if(inrect(rts[i], pt)) return true;
		}
		return false;
	}

	bool inrect(const cv::Rect &rt, const cv::Point2f &pt)
	{
		bool ret = false;

		ret = (rt.x < pt.x) 
					&& (rt.y < pt.y)
					&& (rt.x + rt.width - 1 > pt.x)
					&& (rt.y + rt.height - 1 > pt.y);

		return ret;
	}

	float bb_overlap(const cv::Rect& rt1, const cv::Rect& rt2)
	{
		float ret = 0;

		float temp1 = max(rt1.x, rt2.x);
		float temp2 = max(rt1.y, rt2.y);

		float temp3 = min(rt1.x + rt1.width
		, rt2.x + rt2.width);

		float temp4 = min(rt1.y + rt1.height
		, rt2.y + rt2.height);

		if((temp1 < temp3) && (temp2 < temp4)) {
			ret = 2 * (temp3 - temp1) * (temp4 - temp2);
			ret /= (rt1.width * rt1.height) + (rt2.width * rt2.height);
		}

		return ret;
	}

	double gaussian_prob(double x, double m, double std)
	{
		double var = std * std;
		return 1 / sqrt(2 * M_PI * var) * exp(- pow(x - m, 2) / (2 * var));
	}

	double log_gaussian_prob(double x, double m, double std)
	{
		return -log(sqrt(2 * M_PI) * std) - ( pow((x - m) / std, 2) / 2.0 );
	}
	
	double log_gaussian_prob(cv::Mat &x, cv::Mat& m, cv::Mat &icov, double det)
	{
		my_assert(x.rows == m.rows);
		my_assert(x.rows == icov.cols);
		
		cv::Mat dx = x - m;
		cv::Mat temp = ( (dx.t() * icov) * dx  / 2.0 );
		
		my_assert(temp.rows == 1 && temp.cols == 1);

		double ret =  - x.rows * log(2 * M_PI) / 2 
			   - log(det) / 2  - temp.at<double>(0, 0);

		return ret;
	}

	double state_ground_dist(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->x_ - b->x_, 2) + pow(a->y_ - b->y_, 2));
	}

	double state_dist(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->x_ - b->x_, 2) + pow(a->y_ - b->y_, 2) + pow(a->z_ - b->z_, 2));
	}

	double feat_state_dist(GFeatStatePtr a, GFeatStatePtr b)
	{
		return sqrt(pow(a->x_ - b->x_, 2) + pow(a->y_ - b->y_, 2) + pow(a->z_ - b->z_, 2));
	}
#ifdef VEL_STATE
	double state_ground_vel_diff(PeopleStatePtr a, PeopleStatePtr b)
	{
		return sqrt(pow(a->vx_ - b->vx_, 2) + pow(a->vy_ - b->vy_, 2));
	}
#endif
	void getPairIndex(unsigned int min_idx, unsigned int max_idx, unsigned int &pair_index)
	{
		assert(min_idx < max_idx);
		pair_index = max_idx * (max_idx - 1) / 2 + min_idx;
		assert(pair_index < 10000);
	}

	void getPairFromIndex(unsigned int &min_idx, unsigned int &max_idx, unsigned int num_states, unsigned int pair_index)
	{
		unsigned int temp = 1;
		max_idx = 1;
		while(1) {
			if(pair_index < temp) {
				break;
			}
			pair_index -= temp;
			max_idx++;
			temp++;
		}
		min_idx = pair_index;

		if(!(min_idx < max_idx)) {
			std::cout << "min_idx:" << min_idx << " ";
			std::cout << "max_idx:" << max_idx << " ";
			std::cout << "num_states:" << num_states << " ";
			std::cout << "pair_index:" << pair_index << " ";
			std::cout << std::endl;
			my_assert(0);
		}

		if(!(max_idx < num_states)){
			std::cout << "min_idx:" << min_idx << " ";
			std::cout << "max_idx:" << max_idx << " ";
			std::cout << "num_states:" << num_states << " ";
			std::cout << "pair_index:" << pair_index << " ";
			std::cout << std::endl;
			my_assert(0);
		}
	}

	double getMinDist2Dets(const std::vector<cv::Rect> &dets, const cv::Rect &rt, const double &sx, const double &sy, const double &sh) {
		double ret = 1000.0f;
		double x1, y1, h1;
		double x2, y2, h2;
		double sx2, sy2, sh2;
		double dist;

		x1 = rt.x + rt.width / 2;
		h1 = rt.height;
		y1 = rt.y + h1 / 2;

		sx2 = h1 * sx;
		sy2 = h1 * sy;
		sh2 = h1 * sh;

		for(size_t i = 0; i < dets.size(); i++) {
			x2 = dets[i].x + dets[i].width / 2;
			h2 = dets[i].height;
			y2 = dets[i].y + h2 / 2;
			
			dist = pow((x1 - x2) / sx2, 2);
			// std::cout << x1 << " - " << x2 << " " << dist << std::endl;
			dist += pow((y1 - y2) / sy2, 2);
			// std::cout << y1 << " - " << y2 << " " << dist << std::endl;
			dist += pow((h1 - h2) / sh2, 2);
			// std::cout << h1 << " - " << h2 << " " << dist << std::endl;

			if(dist < ret) {
				ret = dist;
			}
		}

		return ret;
	}

	double soft_max(double x, double scale)
	{
		double ret = 0;
		ret = x / scale;
		ret = scale * ret / sqrt(1.0f + ret * ret);
		return ret;
	}
#ifdef MYDEBUG
	static FILE *dbg_fp = NULL;

	void open_dbg_file(const std::string &filename)
	{
		if(dbg_fp) close_dbg_file();
		dbg_fp = fopen(filename.c_str(), "w");
	}

	void print_dbg_file(const char *msg)
	{
		if(dbg_fp) fprintf(dbg_fp, "%s", msg);
		else	   printf("%s", msg);
	}

	void print_dbg_file(const std::string &msg)
	{
		if(dbg_fp) fprintf(dbg_fp, "%s", msg.c_str());
		else	   printf("%s", msg.c_str());
	}

	void close_dbg_file()
	{
		fclose(dbg_fp);
		dbg_fp = NULL;
	}
#endif
};
