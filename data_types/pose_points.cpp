#include "pose_points.hpp"

PosePoints::PosePoints(const Pose& pose, const ArmDimensions& arm_dimensions)
    : pose_(pose), arm_dimensions_(arm_dimensions)
{
  CalcArmPoints();
  CalcArmEdges();
}

void PosePoints::CalcArmPoints()
{
  if (!arm_points_.empty()) {
    return; // Already computed
  }

  // Redefine for convenience
  float J1 = Deg2Rad(pose_.J1);
  float J2 = Deg2Rad(pose_.J2);
  float J3 = Deg2Rad(pose_.J3);
  float J4 = Deg2Rad(pose_.J4);
  float J5 = Deg2Rad(pose_.J5);
  float J6 = Deg2Rad(pose_.J6);
  float L1 = arm_dimensions_.L1_mm;
  float L2 = arm_dimensions_.L2_mm;
  float L3 = arm_dimensions_.L3_mm;
  float L4 = arm_dimensions_.L4_mm;
  float L5 = arm_dimensions_.L5_mm;
  float L6 = arm_dimensions_.L6_mm;
  float L7 = arm_dimensions_.L7_mm;

  // Forward kinematics equations for arbitrary 6DOF arm
  // PX is the XYZ point corresponding to joint X
  Vec3 P_base(0, 0, -L1);
  Vec3 P1(P_base.x(), P_base.y(), P_base.z() + L1);
  Vec3 P2(P1.x() + L2*std::sin(J1), P1.y() - L2*std::cos(J1), P1.z());
  Vec3 P3(P2.x() + L3*std::sin(J2)*std::sin(M_PI/2.0 - J1),
          P2.y() + L3*std::sin(J2)*std::cos(M_PI/2.0 - J1),
          P2.z() + L3*std::cos(J2));
  Vec3 P4(P3.x() - L4*std::cos(M_PI/2.0 - J1),
          P3.y() + L4*std::sin(M_PI/2.0 - J1),
          P3.z());
  Vec3 P5(P4.x() + L5*std::cos(J3)*std::cos(J1),
          P4.y() + L5*std::cos(J3)*std::sin(J1),
          P4.z() + L5*std::sin(J3));
  Vec3 P6(P5.x() + L6*std::cos(J4)*std::sin(J1) - L6*std::sin(J4)*std::cos(M_PI/2.0 - J3)*std::cos(J1),
          P5.y() - L6*std::cos(J4)*std::cos(J1) - L6*std::sin(J4)*std::cos(M_PI/2.0 - J3)*std::sin(J1),
          P5.z() + L6*std::sin(J4)*std::sin(M_PI/2.0 - J3));
  Vec3 P_wrist(P6.x() + L7*std::cos(J5)*std::cos(J3)*std::cos(J1)
                      - L7*std::sin(J5)*std::cos(J4)*std::sin(J3)*std::cos(J1)
                      - L7*std::sin(J5)*std::sin(J4)*std::sin(J1),
               P6.y() + L7*std::cos(J5)*std::cos(J3)*std::sin(J1)
                      - L7*std::sin(J5)*std::cos(J4)*std::sin(J3)*std::sin(J1)
                      + L7*std::sin(J5)*std::sin(J4)*std::cos(J1),
               P6.z() + L7*std::sin(J5)*std::cos(J4)*std::cos(J3) + L7*std::cos(J5)*std::sin(J3));
  
  // End effector; assume simple 1-link EE orthogonal to wrist face
  Vec3 EE_unit_vector_pre_rotation = (P6 - P5).Cross(P_wrist - P6).Normalized(); // Orthogonal to J6
  Vec3 perpendicular_unit_vec = (P_wrist - P6).Cross(EE_unit_vector_pre_rotation).Normalized(); // Also orthogonal to J6
  Vec3 EE_vec = (EE_unit_vector_pre_rotation.Scaled(std::cos(J6)) - perpendicular_unit_vec.Scaled(std::sin(J6)))
                  .Scaled(arm_dimensions_.EE_length_mm);
  Vec3 P_EE = P_wrist + EE_vec;

  arm_points_ = {P_base, P1, P2, P3, P4, P5, P6, P_wrist, P_EE};
}

void PosePoints::CalcArmEdges()
{
  if (!arm_edges_.empty()) {
    return; // Already computed
  }
  for (int i = 0; i < arm_points_.size() - 1; i++) {
    arm_edges_.push_back(Edge(arm_points_[i], arm_points_[i + 1]));
  }
}
