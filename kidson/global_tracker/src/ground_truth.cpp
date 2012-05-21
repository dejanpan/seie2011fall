/*
 * ground_truth.cpp
 *
 *  Created on: Aug 18, 2011
 *      Author: engelhar
 */

#include "ground_truth.h"





bool Ground_truth::getTrafoAt(ros::Time stamp, stamped_trafo& trafo){


//  cout << "trafos.size: " << trafos.size() << endl;

 // discard trafos before
 for (  ; pos < trafos.size() && trafos.at(pos).time < stamp; pos++) { };

 if (pos >= trafos.size())
   return false;


 if (pos < 1) return false;
 ros::Duration before = stamp - trafos.at(pos-1).time;
 ros::Duration after  = trafos.at(pos).time-stamp;

// cout << "nodetime: " << stamp << endl;
// cout << "next gt " << trafos.at(pos).time << endl;
// cout << "last gt " << trafos.at(pos-1).time << endl;


 trafo = (before < after)?trafos[pos-1]:trafos[pos];

// ROS_INFO("trafo found");

// double dt = abs((trafo.time - stamp).toSec());

// ROS_INFO("best fitting: gt %i with dt %f", pos, dt);
// cout << "trafo: " << trafo.trafo << endl;


 if ( abs((trafo.time - stamp).toSec()) > 0.1){

//   ROS_ERROR("no gt at t = %i, dt = %i", stamp, trafo.time-stamp );
//   cerr << "NO GT: next trafo: " << trafo.time << "  node-stamp " << stamp << endl;
//   cout << "pos:  " << pos << endl;

  return false;
 }


 return true;
}




Ground_truth::Ground_truth(string filename)
{

 fstream ifs; ifs.open(filename.c_str());
 assert(ifs.is_open());

 double time;


 while (true){
  stamped_trafo s_t;

  ifs >> time;
  if (ifs.eof()) break;

  int sec = int(time);
  int nsec = int((time-sec)*1000*1000*1000);


  s_t.time = ros::Time(sec, nsec);
//  cout <<  s_t.time << endl;


#define FROM_QUATERNION


#ifdef FROM_QUATERNION


  g2o::Vector7d quat;


  for (int i=0; i<7; ++i) {
    ifs >> quat[i];
//    cout << quat[i] << ", ";
  }
//  cout << endl;


  g2o::SE3Quat pose;
  pose.fromVector(quat);

  s_t.trafo = pose.to_homogenious_matrix();
//  cout << "hom: " << endl << s_t.trafo << endl;
//  for (int i=0; i<4; i++){
//   for (int j=0; j<4; j++){
//     cout << s_t.trafo(i,j) << " " << endl;
//   }
//    cout << endl;
//  }
//
//  cout << endl;

#else

  for (int i=0; i<4; i++){
   for (int j=0; j<4; j++){

    //      ifs >> time;
    //      cout << time << " ";
    ifs >>  s_t.trafo(i,j);
//    cout << s_t.trafo(i,j) << " " << endl;
   }
   // cout << endl;
  }
#endif

  trafos.push_back(s_t);
 }




 ROS_INFO("Loaded %i stamped trafos from %s", (int) trafos.size(), filename.c_str());
 pos = 0;

}
