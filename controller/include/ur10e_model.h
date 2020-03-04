#pragma once

#include "ur10e_linear.h"
#include <ros/ros.h>

struct ur10e
{
    // dh parameters (updated after the calibration)
    Vec6 dh_a = {0, -0.6127, -0.57155, 0, 0, 0};
    Vec6 dh_d = {0.1807, 0, 0, 0.17415, 0.11985, 0.11655};
    Vec6 dh_alpha = {M_PI_2, 0, 0, M_PI_2, -M_PI_2, 0};
    Vec6 dh_theta = {0,0,0,0,0,0};

    // precomputed terms (updated after the calibration)
    Vec6 cos_alpha;
    Vec6 sin_alpha;
    bool is_calibrated = false;

    // member functions
    void reset();
    void calibrate();
    void fk(Vec6& q, Vec3& pos_result, Mat3& Q_result);
    // Perform the inverse kinematics.
    // @param tar_pos: the cartesian position to reach
    // @param tar_rot: the rotation matrix representing the orientation to reach
    // @param joint_guess: the initial guess in joint space
    // @param joint_result: the joint position where the result is stored
    // @out : true if the inverse kinematics converges
    //
    // - joint_result is only modified if the inverse kinematics converges (i.e. if output is true) it does NOT contain the closest point it found.
    // - all other params are never modified
    bool ik(Vec3& tar_pos, Mat3& tar_rot, Vec6& joint_guess, Vec6& joint_result);
};