// For an explanation of headers, see SFMExample.cpp
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/geometry/Cal3DS2_Base.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <gtsam/nonlinear/DoglegOptimizer.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace std;
using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;

std::vector<gtsam::Pose3> all_poses;
std::vector<gtsam::Point3> all_points;

/// Define the structure for the camera poses
//typedef PinholeCamera<Cal3Bundler> SfM_Camera;
typedef gtsam::PinholeCamera<Cal3DS2> PinholeCamera_Cal3DS2;

/// A measurement with its camera index
typedef std::pair<size_t, Point2> SfM_Measurement;

/// Define the structure for the 3D points
struct SfM_Track {
  SfM_Track():p(0,0,0) {}
  Point3 p; ///< 3D position of the point
  float r, g, b; ///< RGB color of the 3D point
  std::vector<SfM_Measurement> measurements; ///< The 2D image projections (id,(u,v))
  size_t number_measurements() const {
    return measurements.size();
  }
};

/// Define the structure for SfM data
struct SfM_data {
  std::vector<SfM_Track> tracks; ///< Sparse set of points
  size_t number_tracks() const {
    return tracks.size();
  } ///< The number of reconstructed 3D points
};

bool readBAL(const string& filename, SfM_data &data) {
  // Load the data file
  ifstream is(filename.c_str(), ifstream::in);
  if (!is) {
    cout << "Error in readBAL: can not find the file!!" << endl;
    return false;
  }

  // Get the number of camera poses and 3D points
  size_t nrPoses, nrPoints, nrObservations;
  is >> nrPoses >> nrPoints >> nrObservations;
  std::cout << nrPoses << " " << nrPoints << " " << nrObservations << std::endl;

  data.tracks.resize(nrPoints);

  // Get the information for the observations
  for (size_t k = 0; k < nrObservations; k++) {
    size_t i = 0, j = 0;
    float u = 0.0, v=0.0;
    is >> i >> j >> u >> v;
    //std::cout << k << " " << i << " " << j << " " << u << " " << v << std::endl;
    data.tracks[j].measurements.emplace_back(i, gtsam::Point2(u, v));
    //v.emplace_back(gtsam::Point2(u,v));
  }

  // Get the information for the camera poses
  for (size_t k = 0; k < nrPoses; k++) {
    gtsam::Matrix4 m_pose;
    for(size_t m=0; m<4; m++){
      // Need to write this, read 4x4 matrix for pose
      float i, j, u, v;
      is >> i >> j >> u >> v;
      m_pose(m,0) = i;
      m_pose(m,1) = j;
      m_pose(m,2) = u;
      m_pose(m,3) = v;
      //std::cout << i << " " << j << " " << u << " " << v << std::endl;
    }
    gtsam::Pose3 pose(m_pose); 
    //std::cout << pose << std::endl;
    //data.cameras.emplace_back(pose,K);
    all_poses.push_back(pose);
  }

  // Get the information for the 3D points
  for (size_t j = 0; j < nrPoints; j++) {
    // Get the 3D position
    float x, y, z;
    is >> x >> y >> z;
    SfM_Track& track = data.tracks[j];
    track.p = gtsam::Point3(x, y, z);
    track.r = 0.4f;
    track.g = 0.4f;
    track.b = 0.4f;
    all_points.push_back(track.p);
  }

  is.close();
  return true;
}

/* ************************************************************************* */
int main (int argc, char* argv[]) {

  std::cout << "performing BA" << std::endl;

  // Find default file, but if an argument is given, try loading a file
  //string filename = "/home/jrebello/projects/gtsam/examples/Data/dubrovnik-3-7-pre.txt";
  
  string filename = "/home/jrebello/catkin_ws/src/acdcc_slam/data/true_bal.txt";
  if (argc>1) filename = string(argv[1]);

  boost::shared_ptr<Cal3DS2> K(new Cal3DS2(863.64399, 862.81874, 0, 639.69557, 363.08595, -0.00206, 0.00042, -0.00021, 0.000230));

  // Load the SfM data from file
  SfM_data mydata;
  readBAL(filename, mydata);
  cout << boost::format("read %1% tracks on %2% cameras\n") % all_points.size() % all_poses.size();

  // Create a factor graph
  NonlinearFactorGraph graph;

  // We share *one* noiseModel between all projection factors
  noiseModel::Isotropic::shared_ptr pixelNoise = noiseModel::Isotropic::Sigma(2, 0.001); // one pixel in u and v
  noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.001), Vector3::Constant(0.003)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
  noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.001);

  graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), all_poses[0], poseNoise); // add directly to graph
  graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', 0), all_points[0], pointNoise); // add directly to graph

  //initialEstimate.print("Initial Estimates:\n");
  double total_error = 0.0;
  std::cout << std::endl;
  std::cout << "==============================" << std::endl;
  for(size_t t=0; t<mydata.number_tracks(); t++)
  {
    size_t point_idx = t;
    for(size_t m=0; m<mydata.tracks[t].measurements.size(); m++)
    {
      size_t camera_idx = mydata.tracks[t].measurements[m].first; 
      Point2 uv = mydata.tracks[t].measurements[m].second;
      graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3DS2> >(uv, pixelNoise, Symbol('x', camera_idx), Symbol('l', point_idx), K);
      //std::cout << camera_idx << " " << point_idx << std::endl;
      PinholeCamera_Cal3DS2 cam(all_poses[camera_idx], *K);
      //std::cout << "Camera pose: " << cam.pose() << std::endl;
      //std::cout << "Point Location: " << all_points[point_idx] << std::endl;
      Point2 meas = cam.project(all_points[point_idx]);
      std::cout << "true pixels:" << uv << " , projected pixels:" << meas << std::endl;
      //std::cout << (uv - meas).norm() << std::endl;
      total_error += (uv-meas).norm();
      //std::cout << "==============================" << std::endl;
      std::cout << std::endl;
    }
  }
  std::cout << "Total pixel error: " << total_error << std::endl;

    Values initialEstimate;
  for(size_t po=0; po<all_poses.size(); po++){
  	Pose3 temp_pose = all_poses[po];
  	Rot3 r = temp_pose.rotation();
  	Point3 t = temp_pose.translation();

  	Pose3 pose_new(r,t); 

  	//std::cout << r  << " " << t << std::endl;
    //initialEstimate.insert(Symbol('x',po), all_poses[po]);
    //std::cout << all_poses[po] << std::endl;
    initialEstimate.insert(Symbol('x',po), pose_new);
  }
  for(size_t pt=0; pt<mydata.number_tracks(); pt++){
    initialEstimate.insert<Point3>(Symbol('l',pt), mydata.tracks[pt].p);
    //std::cout << mydata.tracks[pt].p << std::endl;
  }
  //graph.print("Factor Graph:\n");
  
  Values result;
  try {
    LevenbergMarquardtParams params;
    params.setVerbosity("ERROR");
    LevenbergMarquardtOptimizer lm(graph, initialEstimate, params);
    result = lm.optimize();
  } catch (exception& e) {
    cout << e.what();
  }
  cout << "final error: " << graph.error(result) << endl;
    /* Optimize the graph and print results */
  //Values result = DoglegOptimizer(graph, initialEstimate).optimize();
  //result.print("Final results:\n");
  //cout << "initial error = " << graph.error(initialEstimate) << endl;
  //cout << "final error = " << graph.error(result) << endl;
  return 0;
}
/* ************************************************************************* */

