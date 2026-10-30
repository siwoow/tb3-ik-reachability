#ifndef PICK_AND_PLACE__GRASP_KINEMATICS_HPP_
#define PICK_AND_PLACE__GRASP_KINEMATICS_HPP_

#include <cmath>
#include <algorithm>
#include <vector>
#include <array>
#include <limits>

namespace pick_and_place
{
inline bool solve4DofIK(
        double x, double y, double z,
        std::vector<double>& out_joints,
        double preferred_pitch = -60.0, bool allow_pitch_fallback = true)
    {
        // OpenMANIPULATOR-X 링크 기하 구조 치수 (단위: m)
        const double X_OFFSET = 0.012;   // Joint 1 회전축 X 오프셋
        const double Z_OFFSET = 0.0765;  // Joint 2 회전축 Z 높이
        const double L2 = 0.13025;       // Joint 2 to Joint 3 링크 길이
        const double ALPHA2 = 0.1853;    // Link 3 굽힘 각도 (atan2(0.024, 0.128))
        const double OFFSET2 = M_PI / 2.0 - ALPHA2; // 1.3855 rad (79.38°)
        const double L3 = 0.124;         // Joint 3 to Joint 4 링크 길이
        const double L4 = 0.126;         // Joint 4 to Gripper Tip 링크 길이

        // 1. Joint 1 (Base Yaw) 계산
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
            !std::isfinite(preferred_pitch)) {return false;}
        out_joints.resize(4);
        double dx = x - X_OFFSET;
        double dy = y;
        out_joints[0] = std::atan2(dy, dx);

        // 2. 평면 2D 좌표계 변환 (Joint 1/2 회전 중심 기준)
        double r = std::sqrt(dx * dx + dy * dy);
        double z_rel = z - Z_OFFSET;

        // 3. 다양한 접근 피치 각도 탐색 (preferred_pitch 우선)
        std::vector<double> pitch_candidates;
        pitch_candidates.push_back(preferred_pitch);
        for (double p = -85.0; allow_pitch_fallback && p <= 15.0; p += 2.5) {
            if (std::abs(p - preferred_pitch) > 1e-3) {
                pitch_candidates.push_back(p);
            }
        }

        for (double pitch_deg : pitch_candidates) {
            double phi = pitch_deg * M_PI / 180.0; // 수평 기준 End-Effector 절대 피치각 (하향은 음수)

            // 손목 관절(Joint 4) 위치 역산
            double rw = r - L4 * std::cos(phi);
            double zw = z_rel - L4 * std::sin(phi);

            double D_sq = rw * rw + zw * zw;
            double cos_d = (D_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
            if (cos_d < -1.0 || cos_d > 1.0) continue; // 작업 영역 밖

            // Elbow-Up 형상 선택
            double delta_theta = std::acos(cos_d);

            double gamma = std::atan2(zw, rw);
            double delta = std::atan2(L3 * std::sin(delta_theta), L2 + L3 * std::cos(delta_theta));
            double theta2_abs = gamma + delta;
            double theta3_abs = theta2_abs - delta_theta;

            // URDF 관절 각도 변환
            double q2 = OFFSET2 - theta2_abs;
            double q3 = -theta3_abs - q2;
            double q4 = -phi + theta3_abs;

            // OpenMANIPULATOR-X URDF 물리적 관절 리미트 검사
            // Joint 2: -1.79 ~ 1.57 rad (-102° ~ 90°)
            // Joint 3: -0.94 ~ 1.38 rad (-54° ~ 79°)
            // Joint 4: -1.79 ~ 2.04 rad (-102° ~ 117°)
            if (q2 >= -1.75 && q2 <= 1.55 &&
                q3 >= -0.92 && q3 <= 1.35 &&
                q4 >= -1.75 && q4 <= 2.00)
            {
                out_joints[1] = q2;
                out_joints[2] = q3;
                out_joints[3] = q4;
                return true;
            }
        }

        return false;
    }


// X/Z convex hull of both vendor finger meshes in link5 (metres).
// Joint translation is along Y, so opening does not change floor clearance.
inline double fingerOffsetZ(double phi)
{
  static const std::array<std::array<double, 2>, 18> hull = {{
    {{0.05860000, 0.01900000}},
    {{0.05860000, -0.01900000}},
    {{0.06720000, -0.02900000}},
    {{0.08345812, -0.02900000}},
    {{0.14258324, -0.01246905}},
    {{0.14362244, -0.01215258}},
    {{0.14448065, -0.01148657}},
    {{0.14593864, -0.00985304}},
    {{0.14650321, -0.00892495}},
    {{0.14670000, -0.00785661}},
    {{0.14670000, 0.00785661}},
    {{0.14650321, 0.00892495}},
    {{0.14593864, 0.00985304}},
    {{0.14448065, 0.01148657}},
    {{0.14362244, 0.01215258}},
    {{0.14258324, 0.01246905}},
    {{0.08345812, 0.02900000}},
    {{0.06720000, 0.02900000}}
  }};
  double lowest = std::numeric_limits<double>::infinity();
  for (const auto & p : hull) {
    lowest = std::min(lowest, (p[0] - 0.126) * std::sin(phi) + p[1] * std::cos(phi));
  }
  return lowest;
}

inline double fingerFloorClearance(const std::vector<double> & q, double floor_z)
{
  if (q.size() != 4 || !std::isfinite(floor_z)) {return -1.0;}
  for (double joint : q) {if (!std::isfinite(joint)) {return -1.0;}}
  const double theta2 = M_PI / 2.0 - 0.1853 - q[1];
  const double theta3 = -q[1] - q[2];
  const double phi = theta3 - q[3];
  const double tcp_z = 0.0765 + 0.13025 * std::sin(theta2) +
    0.124 * std::sin(theta3) + 0.126 * std::sin(phi);
  return tcp_z + fingerOffsetZ(phi) - floor_z;
}

inline bool fingerPathClear(const std::vector<double> & start,
  const std::vector<std::vector<double>> & path, double floor_z)
{
  auto previous = start;
  for (const auto & next : path) {
    if (previous.size() != 4 || next.size() != 4) {return false;}
    // Position-only controller trajectories interpolate linearly in joint space.
    for (int i = 0; i <= 100; ++i) {
      std::vector<double> q(4);
      for (int j = 0; j < 4; ++j) {q[j] = previous[j] + (next[j]-previous[j])*i/100.0;}
      if (fingerFloorClearance(q, floor_z) < 0.006) {return false;}
    }
    previous = next;
  }
  return true;
}

struct GraspPlan
{
  double pitch_degrees{0.0};
  double insertion_depth{0.0};
  double lift_height{0.0};
  double surface_to_body_depth{0.0};
  std::vector<double> pregrasp;
  std::vector<std::vector<double>> descent;
  std::vector<std::vector<double>> insertion;
  std::vector<std::vector<double>> lift;
};

// The observed surface is not the grasp cavity: advance the tip a bounded
// distance along the chosen tool axis, and validate the entire Cartesian path.
inline bool planGrasp(double x, double y, double z, double floor_z,
  double insertion_depth, GraspPlan & result)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
    !std::isfinite(floor_z) || !std::isfinite(insertion_depth) ||
    insertion_depth < 0.0 || insertion_depth > 0.025) {return false;}
  // RGB-D observes the upper skin, not the volume between the fingers.
  // On this flat-floor task, lower the grasp into the upper-middle body.
  // Bound this to 20mm; tip-floor clearance is checked separately below.
  const double body_depth = std::min(0.020, 0.4 * (z - floor_z));
  if (body_depth <= 0.0) {return false;}
  const double surface_z = z;

  const double yaw = std::atan2(y, x - 0.012);
  for (double pitch : {-60.0, -57.5, -62.5, -55.0, -65.0}) {
    const double phi = pitch * M_PI / 180.0;
    // Reserve 8mm at waypoints and 6mm throughout joint interpolation.
    const double minimum_tcp_z = floor_z + 0.008 - fingerOffsetZ(phi);
    if (minimum_tcp_z > surface_z + 0.012) {continue;}
    z = std::max(surface_z - body_depth, minimum_tcp_z);
    const double actual_insertion = std::min(insertion_depth,
      std::max(0.0, (z - minimum_tcp_z) / -std::sin(phi)));
    const double dx = actual_insertion * std::cos(phi) * std::cos(yaw);
    const double dy = actual_insertion * std::cos(phi) * std::sin(yaw);
    const double dz = actual_insertion * std::sin(phi);

    for (double height : {0.06, 0.04}) {
      GraspPlan candidate;
      candidate.pitch_degrees = pitch;
      candidate.insertion_depth = actual_insertion;
      candidate.lift_height = height;
      candidate.surface_to_body_depth = surface_z - z;
      if (!solve4DofIK(x, y, surface_z + height,
          candidate.pregrasp, pitch, false)) {continue;}
      bool valid = true;
      for (int i = 1; i <= 10 && valid; ++i) {
        std::vector<double> joints;
        valid = solve4DofIK(x, y, z + (surface_z + height - z) * (1.0 - i / 10.0),
          joints, pitch, false);
        candidate.descent.push_back(joints);
      }
      for (int i = 1; i <= 4 && valid; ++i) {
        const double f = i / 4.0;
        std::vector<double> joints;
        valid = solve4DofIK(x + f * dx, y + f * dy, z + f * dz, joints, pitch, false);
        candidate.insertion.push_back(joints);
      }
      for (int i = 1; i <= 10 && valid; ++i) {
        std::vector<double> joints;
        valid = solve4DofIK(x + dx, y + dy, z + dz + height * i / 10.0,
          joints, pitch, false);
        candidate.lift.push_back(joints);
      }
      if (valid && fingerPathClear(candidate.pregrasp, candidate.descent, floor_z) &&
        fingerPathClear(candidate.descent.back(), candidate.insertion, floor_z) &&
        fingerPathClear(candidate.insertion.back(), candidate.lift, floor_z)) {
        result = candidate; return true;
      }
    }
  }
  return false;
}

inline bool graspAndLiftReachable(double x, double y, double z)
{
  GraspPlan plan;
  return planGrasp(x, y, z, -0.101, 0.012, plan);
}
}  // namespace pick_and_place
#endif
