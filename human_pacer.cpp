#include <Pacer/robot.h>
#include <map>
#include <string>
#include <Ravelin/Pose3d.h>
#include <Ravelin/SVector6d.h>
#include <Ravelin/Quatd.h>
#include <Ravelin/Origin3d.h>

using namespace Pacer;
using namespace Ravelin;

int main(){
  // 1.57 1.57 0 0 0 0 1.57 0 0 0 1.57 0 0 0 0 0 0
  std::map<std::string,double> q,qd;
  q["0left_ankle"   ] = 1.57;
  q["0right_ankle"   ] = 1.57;
  q["0left_knee"   ] = 0;
  q["0right_knee"] =  0;
  q["0left_hip"   ] = 0;
  q["0right_hip"   ] = 0;
  // q["0transpelvic_link"   ] = 1.57;
  q["0lower_spine"] =  0;
  q["0upper_spine"   ] = 0;
  // q["0clavicular_link"   ] = 0;
  q["0neck"   ] = 1.57;
  q["0left_shoulder"] =  0;
  q["0right_shoulder"   ] = 0;
  q["0left_elbow"   ] = 0;
  q["0right_elbow"   ] = 0;
  q["0left_wrist"] =  0;
  q["0right_wrist"] = 0;

  qd["0left_ankle"   ] = 0;
  qd["0right_ankle"   ] = 0;
  qd["0left_knee"   ] = 0;
  qd["0right_knee"] =  0;
  qd["0left_hip"   ] = 0;
  qd["0right_hip"   ] = 0;
  // qd["0transpelvic_link"   ] = 0;
  qd["0lower_spine"] =  0;
  qd["0upper_spine"   ] = 0;
  // qd["0clavicular_link"   ] = 0;
  qd["0neck"   ] = 0;
  qd["0left_shoulder"] =  0;
  qd["0right_shoulder"   ] = 0;
  qd["0left_elbow"   ] = 0;
  qd["0right_elbow"   ] = 0;
  qd["0left_wrist"] =  0;
  qd["0right_wrist"] = 0;
 
  // TODO: Confirm if this is right 
  Ravelin::SVector6d base_xd(1,0,0,0,0,0);
  boost::shared_ptr<Ravelin::Pose3d>
      base_pose = boost::shared_ptr<Ravelin::Pose3d>(
                    new Ravelin::Pose3d(
                      Ravelin::Quatd::identity(),
                      Ravelin::Origin3d(0, 0, 0),
                      Moby::GLOBAL)
                    );

  // Init a robot from the model we want
  std::string sdf_file("human-model.sdf");
  std::string init_file("human.pacer");
  boost::shared_ptr<Robot> robot
      = boost::shared_ptr<Robot>(new Robot(sdf_file, init_file));

  boost::shared_ptr<const RobotData> data =
            Robot::gen_vars_from_model(
                  q,qd,
                  boost::shared_ptr<const Ravelin::Pose3d>(base_pose),
                  base_xd,
                  robot);
  
  const VectorNd& v = data->generalized_qd;
  const MatrixNd& N = data->N;
  const MatrixNd& M = data->M;
  const MatrixNd& ST = data->D;
  const VectorNd& f = data->generalized_fext;

  std::cout << "v: " << v << std::endl;
  std::cout << "N: " << N << std::endl;
  std::cout << "M: " << M << std::endl;
  std::cout << "ST: " << ST << std::endl;
  std::cout << "f: " << f << std::endl;

  // get a Jacobian for a particular point on a humanoid link
  // 1. I'll specify a random link
  Moby::RCArticulatedBody body = robot->get_articulated_body();
  std::vector<Moby::RigidBodyPtr> links = body->get_links();
  std::random_shuffle(links.begin(), links.end());
  Moby::RigidBodyPtr link = links.front();

  // 2. Now we'll pick a frame defined at (1,0,0) w.r.t. the link
  shared_ptr<Ravelin::Pose3d> P;
  P->x = Ravelin::Origin3d(1.0, 0.0, 0.0);
  P->rpose = link->get_pose();

  // 3. Finally we get the Jacobian of the humanoid (R) 
  Ravelin::MatrixNd R;
  body->calc_jacobian(P, link, R);
  std::cout << "R: " << std::endl << R;
}
