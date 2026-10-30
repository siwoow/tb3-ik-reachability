#ifndef PICK_AND_PLACE__CANDIDATE_GRASP_HPP_
#define PICK_AND_PLACE__CANDIDATE_GRASP_HPP_

#include "pick_and_place/grasp_kinematics.hpp"
#include <string>

namespace pick_and_place
{
// Stationary body candidate planner shared by trials and automatic cleanup.
struct CandidateGrasp
{
  double pitch_degrees{0.0};
  double body_depth{0.0};
  double path_floor_clearance{0.0};
  std::vector<double> hover;
  std::vector<std::vector<double>> descent;
};

struct CandidateDiagnostics
{
  int invalid_input{0};
  int insufficient_body_depth{0};
  int hover_unreachable{0};
  int approach_clearance{0};
  int descent_unreachable{0};
};

inline std::string candidateRejectionSummary(const CandidateDiagnostics & counts)
{
  return "invalid_input=" + std::to_string(counts.invalid_input) +
    ", insufficient_body_depth=" + std::to_string(counts.insufficient_body_depth) +
    ", hover_unreachable=" + std::to_string(counts.hover_unreachable) +
    ", approach_clearance=" + std::to_string(counts.approach_clearance) +
    ", descent_unreachable=" + std::to_string(counts.descent_unreachable);
}

inline bool planCandidateGrasp(double x, double y, double surface, double floor,
  const std::vector<double> & start, CandidateGrasp & result,
  CandidateDiagnostics * diagnostics = nullptr, double min_body_depth = 0.006)
{
  CandidateDiagnostics counts;
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(surface) ||
    !std::isfinite(floor) || surface - floor < 0.020 || surface - floor > 0.150 ||
    start.size() != 4 || !std::isfinite(min_body_depth) ||
    min_body_depth < 0.004 || min_body_depth > 0.020)
  {
    counts.invalid_input = 1;
    if (diagnostics) {*diagnostics = counts;}
    return false;
  }
  const std::array<double, 4> lower{{-2.8, -1.75, -0.92, -1.75}};
  const std::array<double, 4> upper{{2.8, 1.55, 1.35, 2.0}};
  auto within_limits = [&](const std::vector<double> & q, double margin) {
      for (size_t j = 0; j < 4; ++j) {
        if (!std::isfinite(q[j]) || q[j] < lower[j] + margin ||
          q[j] > upper[j] - margin) {return false;}
      }
      return true;
    };
  if (!within_limits(start, 0.0)) {
    counts.invalid_input = 1;
    if (diagnostics) {*diagnostics = counts;}
    return false;
  }
  double best_score = -std::numeric_limits<double>::infinity();
  for (double pitch = -80.0; pitch <= -15.0; pitch += 2.5) {
    const double offset = fingerOffsetZ(pitch * M_PI / 180.0);
    const double hover_z = surface + 0.050 - offset;
    const double body_z = std::max(surface - std::min(0.020, (surface - floor) * 0.4),
      floor + 0.008 - offset);
    const double body_depth = surface - body_z;
    // Body penetration is a grasp-quality requirement, separate from floor clearance.
    if (body_depth < min_body_depth) {++counts.insufficient_body_depth; continue;}
    CandidateGrasp plan;
    if (!solve4DofIK(x, y, hover_z, plan.hover, pitch, false) ||
      !within_limits(plan.hover, 0.025)) {++counts.hover_unreachable; continue;}
    if (!fingerPathClear(start, {plan.hover}, floor)) {
      ++counts.approach_clearance; continue;
    }
    double clearance = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 200; ++i) {
      std::vector<double> q(4);
      for (size_t j = 0; j < 4; ++j) {
        q[j] = start[j] + (plan.hover[j] - start[j]) * i / 200.0;
      }
      clearance = std::min(clearance, fingerFloorClearance(q, floor));
    }
    // The open approach must stay above the selected surface, including between goals.
    if (clearance < surface - floor + 0.045) {++counts.approach_clearance; continue;}
    bool valid = true;
    for (int i = 1; i <= 10; ++i) {
      std::vector<double> q;
      if (!solve4DofIK(x, y, hover_z + (body_z - hover_z) * i / 10.0,
          q, pitch, false) || !within_limits(q, 0.025)) {valid = false; break;}
      plan.descent.push_back(q);
    }
    if (!valid || !fingerPathClear(plan.hover, plan.descent, floor)) {
      ++counts.descent_unreachable; continue;
    }
    // Prefer usable body penetration, then the previously inspected -45 degree pitch.
    const double score = 1000.0 * body_depth - 0.15 * std::abs(pitch + 45.0);
    if (score <= best_score) {continue;}
    best_score = score;
    plan.pitch_degrees = pitch;
    plan.body_depth = body_depth;
    plan.path_floor_clearance = clearance;
    result = plan;
  }
  if (diagnostics) {*diagnostics = counts;}
  return std::isfinite(best_score);
}

inline bool planCandidateExecution(double x, double y, double surface, double floor,
  const std::vector<double> & start, GraspPlan & result,
  CandidateDiagnostics * diagnostics = nullptr, double min_body_depth = 0.006)
{
  CandidateGrasp candidate;
  if (!planCandidateGrasp(x, y, surface, floor, start, candidate, diagnostics,
      min_body_depth)) {return false;}
  result = GraspPlan{};
  result.pitch_degrees = candidate.pitch_degrees;
  result.surface_to_body_depth = candidate.body_depth;
  result.pregrasp = candidate.hover;
  result.descent = candidate.descent;
  // Compatibility representation only; the candidate executor skips INSERT.
  result.insertion = {candidate.descent.back()};
  for (auto it = candidate.descent.rbegin() + 1; it != candidate.descent.rend(); ++it) {
    result.lift.push_back(*it);
  }
  result.lift.push_back(candidate.hover);
  result.lift_height = 0.050 - fingerOffsetZ(candidate.pitch_degrees * M_PI / 180.0) +
    candidate.body_depth;
  return fingerPathClear(candidate.descent.back(), result.lift, floor);
}
}  // namespace pick_and_place
#endif
