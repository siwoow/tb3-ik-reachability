// Read-only adapter: compile against the EXISTING, unchanged executor headers.
// No ROS, controller, network, or hardware dependencies.
#include <pick_and_place/candidate_grasp.hpp>
#include <cstdlib>
#include <iomanip>
#include <iostream>

void array(const std::vector<double>& v) {
  std::cout << '[';
  for (size_t i=0;i<v.size();++i) {if(i)std::cout<<',';std::cout<<v[i];}
  std::cout << ']';
}
void path(const std::vector<std::vector<double>>& v) {
  std::cout << '[';
  for (size_t i=0;i<v.size();++i) {if(i)std::cout<<',';array(v[i]);}
  std::cout << ']';
}
int main(int argc,char** argv) {
  if(argc!=10) return 2;
  double v[9];
  for(int i=0;i<9;++i) {char* end=nullptr;v[i]=std::strtod(argv[i+1],&end);
    if(!end || *end || !std::isfinite(v[i]))return 2;}
  pick_and_place::GraspPlan plan;
  const bool ok=pick_and_place::planCandidateExecution(v[0],v[1],v[2],v[3],
    {v[4],v[5],v[6],v[7]},plan,nullptr,v[8]);
  std::cout << std::setprecision(17) << "{\"feasible\":" << (ok?"true":"false");
  if(ok) {std::cout<<",\"pitch_degrees\":"<<plan.pitch_degrees<<",\"body_depth_m\":"<<plan.surface_to_body_depth;
    std::cout<<",\"pregrasp\":";array(plan.pregrasp);std::cout<<",\"descent\":";path(plan.descent);
    std::cout<<",\"lift\":";path(plan.lift);}
  std::cout << "}\n";
}
