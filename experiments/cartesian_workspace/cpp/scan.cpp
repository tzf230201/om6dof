// Offline Cartesian workspace experiment: no ROS node, publisher or hardware I/O.
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Vec = Eigen::Matrix<double, 6, 1>;
using Mat = Eigen::Matrix<double, 6, 6>;
using V3 = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
namespace fs = std::filesystem;
constexpr double pi = 3.14159265358979323846;
constexpr double inf = std::numeric_limits<double>::infinity();

struct Options {
  fs::path urdf, output;
  std::string base = "world", tip = "end_effector_link", orientation = "radial";
  double spacing = 50, radius = 0, margin = .02, collision_radius = 25;
  double pos_tol = 1, rot_tol = .5, length = 300, singular = .01;
  int samples = 2000, seeds = 4, iterations = 100;
  unsigned seed = 42;
  V3 rpy = V3::Zero();
  bool self_test = false, probe = false;
  Vec probe_q = Vec::Zero();
};

std::string read_file(const fs::path &p) {
  std::ifstream f(p);
  if (!f) throw std::runtime_error("Cannot read " + p.string());
  return {std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()};
}
std::string quoted(const std::string &s) {
  std::ostringstream o; o << '"';
  for (unsigned char c : s) {
    if (c == '"' || c == '\\') o << '\\' << c;
    else if (c < 32) o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else o << c;
  }
  o << '"'; return o.str();
}
template<class T> void json_array(std::ostream &o, const T &a) {
  o << '['; for (int i = 0; i < a.size(); ++i) { if (i) o << ','; o << a[i]; } o << ']';
}
double number(const std::string &s) {
  size_t n; double x = std::stod(s, &n);
  if (n != s.size() || !std::isfinite(x)) throw std::runtime_error("Invalid finite number: " + s);
  return x;
}
int integer(const std::string &s) {
  double x = number(s);
  if (x < 0 || x > 10000000 || std::floor(x) != x) throw std::runtime_error("Invalid count: " + s);
  return static_cast<int>(x);
}
Options parse(int argc, char **argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    std::string key = argv[i];
    auto next = [&]() -> std::string { if (++i >= argc) throw std::runtime_error("Missing value for " + key); return argv[i]; };
    if (key == "--urdf") o.urdf = next();
    else if (key == "--output") o.output = next();
    else if (key == "--base-link") o.base = next();
    else if (key == "--tip-link") o.tip = next();
    else if (key == "--orientation") o.orientation = next();
    else if (key == "--spacing-mm") o.spacing = number(next());
    else if (key == "--radius-mm") { o.radius = number(next()); if (o.radius <= 0) throw std::runtime_error("Radius must be positive"); }
    else if (key == "--joint-margin-rad") o.margin = number(next());
    else if (key == "--collision-radius-mm") o.collision_radius = number(next());
    else if (key == "--position-tolerance-mm") o.pos_tol = number(next());
    else if (key == "--orientation-tolerance-deg") o.rot_tol = number(next());
    else if (key == "--length-scale-mm") o.length = number(next());
    else if (key == "--singular-threshold") o.singular = number(next());
    else if (key == "--samples") o.samples = integer(next());
    else if (key == "--seeds") o.seeds = integer(next());
    else if (key == "--iterations") o.iterations = integer(next());
    else if (key == "--seed") o.seed = integer(next());
    else if (key == "--rpy-deg") for (int j = 0; j < 3; ++j) o.rpy[j] = std::remainder(number(next()), 360.) * pi / 180;
    else if (key == "--probe") { o.probe = true; for (int j = 0; j < 6; ++j) o.probe_q[j] = number(next()); }
    else if (key == "--self-test") o.self_test = true;
    else if (key == "--help") {
      std::cout << "Offline only. --urdf MODEL.urdf --output NEW_DIR [--spacing-mm 50] [--radius-mm N]\n"
                << "[--samples 2000 --seeds 4 --iterations 100 --seed 42]\n"
                << "[--orientation radial|fixed --rpy-deg R P Y --joint-margin-rad .02]\n"
                << "[--collision-radius-mm 25 --position-tolerance-mm 1 --orientation-tolerance-deg .5]\n"
                << "[--base-link world --tip-link end_effector_link --length-scale-mm 300 --singular-threshold .01]\n"
                << "Diagnostics: --self-test or --probe q1 q2 q3 q4 q5 q6 (radians).\n";
      std::exit(0);
    } else throw std::runtime_error("Unknown option " + key);
  }
  if (o.urdf.empty() || (!o.self_test && !o.probe && o.output.empty())) throw std::runtime_error("--urdf and --output are required for scans");
  if (o.spacing <= 0 || o.margin < 0 || o.samples < 1 || o.seeds < 1 || o.iterations < 1 ||
      o.collision_radius <= 0 || o.pos_tol <= 0 || o.rot_tol <= 0 || o.length <= 0 || o.singular <= 0 ||
      (o.orientation != "radial" && o.orientation != "fixed")) throw std::runtime_error("Invalid scan settings");
  return o;
}

M3 gripper_rotation(const V3 &rpy) {
  M3 tip_from_gripper;
  tip_from_gripper << 0,0,-1, 0,1,0, 1,0,0;
  return (Eigen::AngleAxisd(rpy[2], V3::UnitZ()) * Eigen::AngleAxisd(rpy[1], V3::UnitY()) *
          Eigen::AngleAxisd(rpy[0], V3::UnitX())).toRotationMatrix() * tip_from_gripper.transpose();
}
V3 rotation_error(const M3 &target, const M3 &actual) {
  Eigen::AngleAxisd aa(target * actual.transpose());
  return aa.angle() * aa.axis();
}
double segment_distance(const V3 &p1, const V3 &p2, const V3 &p3, const V3 &p4) {
  V3 d1 = p2-p1, d2 = p4-p3, r = p1-p3;
  double a=d1.squaredNorm(), e=d2.squaredNorm(), f=d2.dot(r), s=0, t=0;
  if (a <= 1e-9 && e <= 1e-9) return r.norm();
  if (a <= 1e-9) t=std::clamp(f/e, 0., 1.);
  else {
    double c=d1.dot(r);
    if (e <= 1e-9) s=std::clamp(-c/a, 0., 1.);
    else {
      double b=d1.dot(d2), denom=a*e-b*b;
      if (denom > 1e-9) s=std::clamp((b*f-c*e)/denom, 0., 1.);
      t=(b*s+f)/e;
      if (t < 0) { t=0; s=std::clamp(-c/a, 0., 1.); }
      else if (t > 1) { t=1; s=std::clamp((b-c)/a, 0., 1.); }
    }
  }
  return (p1+d1*s-p3-d2*t).norm();
}

class Robot {
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::JntArray joints{6};
  KDL::Jacobian jac{6};
  void set(const Vec &q) { for (int i=0; i<6; ++i) joints(i)=q[i]; }
public:
  Vec lower, upper, hard_lower, hard_upper;
  std::vector<std::string> joint_names;
  Robot(const std::string &xml, const Options &o) {
    urdf::Model model; KDL::Tree tree;
    if (!model.initString(xml) || !kdl_parser::treeFromUrdfModel(model, tree) ||
        !tree.getChain(o.base, o.tip, chain) || chain.getNrOfJoints() != 6)
      throw std::runtime_error("Could not build six-joint URDF chain");
    int index=0;
    for (const auto &segment : chain.segments) {
      if (segment.getJoint().getType() == KDL::Joint::None) continue;
      auto name=segment.getJoint().getName(); auto j=model.getJoint(name);
      if (!j || j->type != urdf::Joint::REVOLUTE || !j->limits ||
          !std::isfinite(j->limits->lower) || !std::isfinite(j->limits->upper))
        throw std::runtime_error("Expected finite revolute limits: " + name);
      joint_names.push_back(name);
      hard_lower[index]=j->limits->lower; hard_upper[index]=j->limits->upper; ++index;
    }
    lower=hard_lower.array()+o.margin; upper=hard_upper.array()-o.margin;
    if ((lower.array() >= upper.array()).any()) throw std::runtime_error("Margin removes joint range");
    fk_solver=std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
    jac_solver=std::make_unique<KDL::ChainJntToJacSolver>(chain);
  }
  Vec clamp(const Vec &q) const { return q.cwiseMax(lower).cwiseMin(upper); }
  std::pair<V3,M3> fk(const Vec &q) {
    set(q); KDL::Frame f;
    if (fk_solver->JntToCart(joints, f) < 0) throw std::runtime_error("FK failed");
    V3 p; M3 r;
    for (int i=0;i<3;++i) { p[i]=f.p(i); for(int j=0;j<3;++j) r(i,j)=f.M(i,j); }
    return {p,r};
  }
  Mat jacobian(const Vec &q) {
    set(q);
    if (jac_solver->JntToJac(joints,jac) < 0) throw std::runtime_error("Jacobian failed");
    return jac.data;
  }
  std::vector<V3> skeleton(const Vec &q) {
    set(q); std::vector<V3> pts{V3::Zero()}; KDL::Frame f=KDL::Frame::Identity();
    unsigned j=0;
    for (const auto &segment : chain.segments) {
      double value=segment.getJoint().getType() == KDL::Joint::None ? 0 : q[j++];
      f=f*segment.pose(value); pts.emplace_back(f.p.x(),f.p.y(),f.p.z());
    }
    return pts;
  }
  double bound() {
    auto pts=skeleton(Vec::Zero()); double result=0;
    for (size_t i=1;i<pts.size();++i) result+=(pts[i]-pts[i-1]).norm();
    return result;
  }
  bool collides(const Vec &q, double radius) {
    auto pts=skeleton(q);
    for (size_t i=0;i+1<pts.size();++i) for(size_t j=i+3;j+1<pts.size();++j) {
      if ((pts[i+1]-pts[i]).norm()<1e-6 || (pts[j+1]-pts[j]).norm()<1e-6) continue;
      if (segment_distance(pts[i],pts[i+1],pts[j],pts[j+1]) < 2*radius) return true;
    }
    return false;
  }
  double conditioning(const Vec &q, double length) {
    Mat j=jacobian(q); j.topRows<3>()/=length;
    Eigen::JacobiSVD<Mat> svd(j);
    return svd.singularValues()[5]/std::max(1e-15,svd.singularValues()[0]);
  }
  Vec solve(Vec q, const V3 &pos, const M3 &rot, bool pose, int iterations) {
    q=clamp(q); Vec weight; weight << 1/.3,1/.3,1/.3,1/(2*pi),1/(2*pi),1/(2*pi);
    if (!pose) weight.tail<3>().setZero();
    auto error=[&](const Vec &v) { auto f=fk(v); Vec e; e.head<3>()=pos-f.first;
      e.tail<3>()=rotation_error(rot,f.second); return e; };
    Vec e=error(q); double cost=(e.array().square()*weight.array()).sum();
    for (int iteration=0;iteration<iterations && cost>=1e-12;++iteration) {
      Mat j=jacobian(q), h=j.transpose()*weight.asDiagonal()*j;
      h.diagonal().array()+=cost+.002;
      Vec step=h.ldlt().solve(j.transpose()*weight.asDiagonal()*e);
      if (!step.allFinite()) break;
      bool accepted=false;
      for (int k=0;k<=8;++k) {
        Vec trial=clamp(q+std::ldexp(1.,-k)*step), trial_error=error(trial);
        double trial_cost=(trial_error.array().square()*weight.array()).sum();
        if (trial_cost<cost) { q=trial; e=trial_error; cost=trial_cost; accepted=true; break; }
      }
      if (!accepted) break;
    }
    return q;
  }
};

struct Candidate { Vec q; double ep=inf, er=inf, rcond=0; bool collision=false; };
struct Row {
  V3 position, rpy;
  bool position_found=false, pose_found=false;
  std::string status="position_unresolved";
  double best_error=inf, margin=inf;
  Candidate witness;
};
Row scan_point(Robot &robot, const V3 &position, const std::vector<Vec> &seeds, const Options &o) {
  Row row; row.position=position; row.rpy=o.rpy;
  if (o.orientation=="radial") row.rpy[2]+=std::atan2(position[1],position[0]);
  M3 target=gripper_rotation(row.rpy);
  std::vector<Candidate> free, poses;
  bool collision_seen=false;
  auto evaluate=[&](const Vec &q) {
    if (!q.allFinite() || (q.array()<robot.lower.array()-1e-10).any() ||
        (q.array()>robot.upper.array()+1e-10).any()) return;
    auto fk=robot.fk(q); Candidate c;
    c.q=q; c.ep=(position-fk.first).norm()*1000; c.er=rotation_error(target,fk.second).norm()*180/pi;
    if(!std::isfinite(c.ep) || !std::isfinite(c.er)) return;
    row.best_error=std::min(row.best_error,c.ep);
    if (c.ep>o.pos_tol) return;
    c.collision=robot.collides(q,o.collision_radius/1000);
    if (c.collision) { collision_seen=true; return; }
    free.push_back(c);
    if (c.er<=o.rot_tol) {
      c.rcond=robot.conditioning(q,o.length/1000);
      if(std::isfinite(c.rcond)) poses.push_back(c);
    }
  };
  for (const auto &seed : seeds) evaluate(robot.solve(seed,position,target,false,o.iterations));
  std::vector<Vec> pose_seeds;
  for(size_t i=0;i<std::min(size_t(2),free.size());++i) pose_seeds.push_back(free[i].q);
  pose_seeds.insert(pose_seeds.end(),seeds.begin(),seeds.end());
  for (const auto &seed : pose_seeds) evaluate(robot.solve(seed,position,target,true,o.iterations));
  row.position_found=!free.empty(); row.pose_found=!poses.empty();
  if (row.pose_found) {
    row.witness=*std::max_element(poses.begin(),poses.end(),[](const auto &a,const auto &b){return a.rcond<b.rcond;});
    row.status=row.witness.rcond<o.singular ? "pose_near_singular" : "pose_found";
  } else if (row.position_found) {
    row.witness=*std::min_element(free.begin(),free.end(),[](const auto &a,const auto &b){return a.ep<b.ep;});
    row.status="orientation_unresolved";
  } else if (collision_seen) row.status="collision_candidates_only";
  if (row.position_found) row.margin=(row.witness.q-robot.lower).cwiseMin(robot.upper-row.witness.q).minCoeff()*180/pi;
  return row;
}
std::vector<V3> grid(double radius, double spacing) {
  double steps=radius/spacing;
  if (!std::isfinite(steps) || spacing<=0 || radius<=0 || steps>100)
    throw std::runtime_error("Invalid/too large grid: use coarser spacing");
  int n=int(std::floor(steps));
  std::vector<V3> points;
  for(int x=-n;x<=n;++x) for(int y=-n;y<=n;++y) for(int z=-n;z<=n;++z) {
    V3 p=V3(x,y,z)*spacing;
    if(p.norm()<=radius+1e-12) points.push_back(p);
  }
  return points;
}
void field(std::ostream &s,double x) { if(std::isfinite(x)) s<<x; }
void write_row(std::ostream &s,const Row &r) {
  s<<r.position[0]*1000<<','<<r.position[1]*1000<<','<<r.position[2]*1000<<','
   <<r.position_found<<','<<r.pose_found<<','; field(s,r.best_error); s<<',';
  if(r.pose_found) s<<r.witness.rcond;
  s<<','; field(s,r.margin); s<<',';
  if(r.position_found) s<<r.witness.er;
  s<<',';
  if(r.position_found) s<<r.witness.ep;
  s<<','<<r.status;
  for(int i=0;i<6;++i) { s<<','; if(r.position_found) s<<r.witness.q[i]; }
  for(int i=0;i<3;++i) s<<','<<r.rpy[i]*180/pi;
  s<<'\n';
}
void require(bool condition,const char *message) { if(!condition) throw std::runtime_error(message); }

int main(int argc,char **argv) try {
  Options o=parse(argc,argv); std::string xml=read_file(o.urdf); Robot robot(xml,o);
  std::cout<<std::setprecision(16);
  if(o.probe) {
    auto f=robot.fk(o.probe_q);
    std::cout<<"{\"position\":"; json_array(std::cout,f.first); std::cout<<",\"rotation\":[";
    for(int i=0;i<3;++i) for(int j=0;j<3;++j) { if(i||j)std::cout<<',';std::cout<<f.second(i,j); }
    std::cout<<"],\"rcond\":"<<robot.conditioning(o.probe_q,o.length/1000)<<"}\n"; return 0;
  }
  Vec ready; ready<<0,-.6806,1.3613,0,.8901,0; ready=robot.clamp(ready);
  if(o.self_test) {
    require(grid(.05,.05).size()==7,"Grid sphere test failed");
    require(grid(.2,.1).size()==33,"Grid boundary test failed");
    require(std::abs(rotation_error(Eigen::AngleAxisd(pi,V3::UnitZ()).toRotationMatrix(),M3::Identity()).norm()-pi)<1e-10,"180 degree rotation error failed");
    require(segment_distance({0,0,0},{1,0,0},{.5,-1,0},{.5,1,0})<1e-10,"Collision segment crossing test failed");
    auto row=scan_point(robot,{.15,0,.3},{ready},o);
    require(row.pose_found,"Known 150/0/300 target not found");
    require(!robot.collides(row.witness.q,o.collision_radius/1000),"Known target witness collision");
    std::cout<<"Native self-tests passed; known-target error "<<row.witness.ep<<" mm / "<<row.witness.er<<" deg\n"; return 0;
  }
  if(fs::exists(o.output)) throw std::runtime_error("Output exists; choose a new directory");
  double bound=robot.bound(), radius=o.radius>0 ? o.radius/1000 : bound;
  auto points=grid(radius,o.spacing/1000);
  auto started=std::chrono::steady_clock::now();
  fs::create_directories(o.output);
  std::ofstream(o.output/"model.urdf")<<xml;
  std::vector<Vec> bank(o.samples); std::vector<V3> bank_xyz(o.samples);
  std::mt19937 rng(o.seed); std::uniform_real_distribution<double> uniform(0,1);
  std::ofstream bank_file(o.output/"seed_bank.csv"); bank_file<<std::setprecision(17);
  bank_file<<"q1,q2,q3,q4,q5,q6,x,y,z\n";
  for(int i=0;i<o.samples;++i) {
    for(int j=0;j<6;++j) bank[i][j]=robot.lower[j]+uniform(rng)*(robot.upper[j]-robot.lower[j]);
    bank_xyz[i]=robot.fk(bank[i]).first;
    for(int j=0;j<6;++j) bank_file<<bank[i][j]<<',';
    bank_file<<bank_xyz[i][0]<<','<<bank_xyz[i][1]<<','<<bank_xyz[i][2]<<'\n';
  }
  std::ofstream csv(o.output/"points.csv"); csv<<std::setprecision(17);
  csv<<"x_mm,y_mm,z_mm,position_found,pose_found,best_position_error_mm,rcond,joint_margin_deg,orientation_error_deg,position_error_mm,status,q1_rad,q2_rad,q3_rad,q4_rad,q5_rad,q6_rad,target_roll_deg,target_pitch_deg,target_yaw_deg\n";
  std::vector<std::pair<double,int>> nearest(o.samples);
  std::map<std::string,int> counts; int free_count=0,pose_count=0;
  std::cout<<"Offline C++ scan: "<<points.size()<<" points; "<<o.spacing<<" mm grid\n";
  for(size_t index=0;index<points.size();++index) {
    for(int i=0;i<o.samples;++i) nearest[i]={(bank_xyz[i]-points[index]).squaredNorm(),i};
    int k=std::min(o.samples,o.seeds);
    std::partial_sort(nearest.begin(),nearest.begin()+k,nearest.end());
    std::vector<Vec> seeds; for(int i=0;i<k;++i) seeds.push_back(bank[nearest[i].second]);
    seeds.push_back(ready); seeds.push_back(robot.clamp(Vec::Zero()));
    Row row=scan_point(robot,points[index],seeds,o); write_row(csv,row);
    ++counts[row.status]; free_count+=row.position_found; pose_count+=row.pose_found;
    if((index+1)%250==0 || index+1==points.size()) {
      csv.flush();std::cout<<index+1<<'/'<<points.size()<<" points\n"<<std::flush;
    }
  }
  csv.close(); bank_file.close();
  double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  std::ofstream summary(o.output/"summary.json"); summary<<std::setprecision(17);
  summary<<"{\n\"engine\":\"C++17 Eigen/KDL, bounded multi-start LM\",\n\"points\":"<<points.size()
    <<",\n\"radius_mm\":"<<radius*1000<<",\n\"conservative_urdf_bound_mm\":"<<bound*1000
    <<",\n\"elapsed_seconds\":"<<seconds<<",\n\"position_found\":"<<free_count<<",\n\"pose_found\":"<<pose_count
    <<",\n\"arguments\":{\"spacing_mm\":"<<o.spacing<<",\"samples\":"<<o.samples<<",\"seeds\":"<<o.seeds
    <<",\"iterations\":"<<o.iterations<<",\"seed\":"<<o.seed<<",\"orientation\":"<<quoted(o.orientation)
    <<",\"base_link\":"<<quoted(o.base)<<",\"tip_link\":"<<quoted(o.tip)<<",\"joint_margin_rad\":"<<o.margin
    <<",\"collision_radius_mm\":"<<o.collision_radius<<",\"position_tolerance_mm\":"<<o.pos_tol
    <<",\"orientation_tolerance_deg\":"<<o.rot_tol<<",\"length_scale_mm\":"<<o.length
    <<",\"singular_threshold\":"<<o.singular<<",\"rpy_deg\":";json_array(summary,V3(o.rpy*180/pi));
  summary<<"},\n\"hard_lower_rad\":";json_array(summary,robot.hard_lower);
  summary<<",\n\"hard_upper_rad\":";json_array(summary,robot.hard_upper);
  summary<<",\n\"effective_lower_rad\":";json_array(summary,robot.lower);
  summary<<",\n\"effective_upper_rad\":";json_array(summary,robot.upper);
  summary<<",\n\"counts\":{";bool first=true;
  for(const auto &item:counts) {if(!first)summary<<',';first=false;summary<<quoted(item.first)<<':'<<item.second;}
  summary<<"},\n\"limitations\":[\"Finite grid and seeds; unresolved is not proven unreachable\","
    <<"\"One orientation per point; not exhaustive SO(3)\",\"Endpoint capsule collision only, no trajectory/obstacles/dynamics\","
    <<"\"Full sphere includes negative Z; no table\",\"Singularity refers to best tested pose witness, not all configurations\"]}\n";
  require(bool(csv)&&bool(bank_file)&&bool(summary),"Failed writing complete experiment output");
  std::cout<<"Complete: "<<seconds<<" seconds; position="<<free_count<<", pose="<<pose_count<<"; "<<o.output<<'\n';
  return 0;
} catch(const std::exception &e) { std::cerr<<"Error: "<<e.what()<<'\n';return 1; }
