#include "robot_model.h"
#include <Eigen/Cholesky>




void RobotModel::calibrate()
{
    Vec6 delta_theta =
    {
        0-1.30345832091358083e-07,
        -0.631610028604681184,
        6.7237875234669966,
        0.191002142182551654,
        -1.95655373920987363e-06,
        -1.58166497364609082e-08
    };
    Vec6 delta_a =
    {
        2.64410839472259099e-05,
        0.11867330610075455,
        0.0108717601887146076,
        -5.50747272256598333e-05,
        2.98251640722243668e-05,
        0
    };
    Vec6 delta_d =
    {
        0.000170790308430895932,
        232.03252517714526,
        -209.704674478528091,
        -22.327777478289434,
        -9.64568561949152858e-05,
        -0.000968874706858524615
    };
    Vec6 delta_alpha =
    {
        0-0.000271200013223005243,
        -0.00155762044922334582,
        -0.00485564255655856949,
        -0.00103097088128212278,
        -7.00896995613486951e-05,
        0
    };
    dh_a += delta_a;
    dh_d += delta_d;
    dh_theta += delta_theta;
    dh_alpha += delta_alpha;

    for (uint joint = 0; joint < 6; joint++)
    {
        cos_alpha[joint] = cos(dh_alpha[joint]);
        sin_alpha[joint] = sin(dh_alpha[joint]);
    }

    is_calibrated = true;
}
void RobotModel::fk(Vec6& q, Vec3& pos_result, Mat3& Q_result)
{
    Vec6 cos_th;
    Vec6 sin_th;
    Vec3 a[6];
    Mat3 Q[6];
    Vec3 r2, r3, r4, r5, r6;
    Mat3 P1, P2, P3, P4, P5;
    for (uint joint = 0; joint < 6; joint++)
    {
        cos_th[joint] = cos(q[joint] + dh_theta[joint]);
        sin_th[joint] = sin(q[joint] + dh_theta[joint]);

        Q[joint](0, 0) = cos_th[joint];
        Q[joint](1, 0) = sin_th[joint];
        Q[joint](2, 0) = 0;

        Q[joint](0, 1) = -cos_alpha[joint] * sin_th[joint];
        Q[joint](1, 1) = cos_alpha[joint] * cos_th[joint];
        Q[joint](2, 1) = sin_alpha[joint];

        Q[joint](0, 2) = sin_alpha[joint] * sin_th[joint];
        Q[joint](1, 2) = -sin_alpha[joint] * cos_th[joint];
        Q[joint](2, 2) = cos_alpha[joint];

        a[joint] =
        {
            dh_a[joint] * cos_th[joint],
            dh_a[joint] * sin_th[joint],
            dh_d[joint]
        };
    }

    P1 = Q[0];
    P2 = P1 * Q[1];
    P3 = P2 * Q[2];
    P4 = P3 * Q[3];
    P5 = P4 * Q[4];
    Q_result = P5 * Q[5];

    r6 = P5 * a[5];
    r5 = r6 + P4 * a[4];
    r4 = r5 + P3 * a[3];
    r3 = r4 + P2 * a[2];
    r2 = r3 + P1 * a[1];
    pos_result = r2 + a[0];
}
bool RobotModel::ik(Vec3 tar_pos, Mat3 tar_rot, Vec6 guess_joint, Vec6 &result_joint)
{
    Vec6 q = guess_joint;
    bool success = false;

    double epsilon = 0.000000005;
    Vec6 cos_th;
    Vec6 sin_th;
    Mat3 Q[6];
    Mat3 P1, P2, P3, P4, P5, P6;
    Vec3 e1 = {0, 0, 1};
    Vec3 e2, e3, e4, e5, e6;
    Vec3 r1, r2, r3, r4, r5, r6;
    Vec3 e1r1, e2r2, e3r3, e4r4, e5r5, e6r6;
    Vec3 a[6];
    Mat3 EQ1, EQ2, EQ3, EQ4, EQ5, EQ6;
    Mat12x6 J;
    Vec12 f;
    Mat3 Q_error;
    Vec3 p_error;
    Mat3 E = {0, 1, 0, -1, 0, 0, 0, 0, 0};
    Mat6 Hk;
    Mat3 Q45, Q345, Q2345;

    /* eigen stuff */
    Eigen::Map<Eigen::VectorXd> eigen_q(q.data, 6, 1);
    Eigen::Map<Eigen::VectorXd> eigen_f(f.data, 12, 1);
    Eigen::Map<Eigen::MatrixXd> eigen_J(J.data, 12, 6);
    Eigen::Map<Eigen::MatrixXd> eigen_Hk(Hk.data, 6, 6);
    Eigen::MatrixXd eigen_Jinv(6, 12);

    for (uint iter = 0; iter < 10; iter++)
    {

        // compute the rotation matrices
        for (uint joint = 0; joint < 6; joint++)
        {

            cos_th[joint] = cos(q[joint] + dh_theta[joint]);
            sin_th[joint] = sin(q[joint] + dh_theta[joint]);

            Q[joint](0, 0) = cos_th[joint];
            Q[joint](1, 0) = sin_th[joint];
            Q[joint](2, 0) = 0;

            Q[joint](0, 1) = -cos_alpha[joint] * sin_th[joint];
            Q[joint](1, 1) = cos_alpha[joint] * cos_th[joint];
            Q[joint](2, 1) = sin_alpha[joint];

            Q[joint](0, 2) = sin_alpha[joint] * sin_th[joint];
            Q[joint](1, 2) = -sin_alpha[joint] * cos_th[joint];
            Q[joint](2, 2) = cos_alpha[joint];

            a[joint] =
            {
                dh_a[joint] * cos_th[joint],
                dh_a[joint] * sin_th[joint],
                dh_d[joint]
            };
        }

        // compute intermediate values used to fill in the jacobian matrix
        P1 = Q[0];
        P2 = P1 * Q[1];
        P3 = P2 * Q[2];
        P4 = P3 * Q[3];
        P5 = P4 * Q[4];
        P6 = P5 * Q[5];

        r6 = P5 * a[5];
        r5 = r6 + P4 * a[4];
        r4 = r5 + P3 * a[3];
        r3 = r4 + P2 * a[2];
        r2 = r3 + P1 * a[1];
        r1 = r2 + a[0];

        e2 = P1 * e1;
        e3 = P2 * e1;
        e4 = P3 * e1;
        e5 = P4 * e1;
        e6 = P5 * e1;

        Q45 = Q[4] * Q[5];
        Q345 = Q[3] * Q45;
        Q2345 = Q[2] * Q345;
        EQ6 = P5 * E * Q[5];
        EQ5 = P4 * E * Q45;
        EQ4 = P3 * E * Q345;
        EQ3 = P2 * E * Q2345;
        EQ2 = P1 * E * Q[1] * Q2345;
        EQ1 = E * P6;

        // fill in the jacobian (first 9 rows)
        // --> this works because the matrices are column major
        std::copy(EQ1.begin(), EQ1.end(), J.pointer_to(0, 0));
        std::copy(EQ2.begin(), EQ2.end(), J.pointer_to(0, 1));
        std::copy(EQ3.begin(), EQ3.end(), J.pointer_to(0, 2));
        std::copy(EQ4.begin(), EQ4.end(), J.pointer_to(0, 3));
        std::copy(EQ5.begin(), EQ5.end(), J.pointer_to(0, 4));
        std::copy(EQ6.begin(), EQ6.end(), J.pointer_to(0, 5));

        // fill in the last 3 rows of the jacobian (position)
        e1r1 = e1.cross(r1);
        e2r2 = e2.cross(r2);
        e3r3 = e3.cross(r3);
        e4r4 = e4.cross(r4);
        e5r5 = e5.cross(r5);
        e6r6 = e6.cross(r6);
        std::copy(e1r1.begin(), e1r1.end(), J.pointer_to(9, 0));
        std::copy(e2r2.begin(), e2r2.end(), J.pointer_to(9, 1));
        std::copy(e3r3.begin(), e3r3.end(), J.pointer_to(9, 2));
        std::copy(e4r4.begin(), e4r4.end(), J.pointer_to(9, 3));
        std::copy(e5r5.begin(), e5r5.end(), J.pointer_to(9, 4));
        std::copy(e6r6.begin(), e6r6.end(), J.pointer_to(9, 5));

        // compute the performance of the current guess (into f: 12x1 vector)
        Q_error = P6 - tar_rot;
        p_error = r1 - tar_pos;
        std::copy(Q_error.begin(), Q_error.end(), f.data);
        std::copy(p_error.begin(), p_error.end(), f.data + 9);

        // update guess
        /* remember:
            - eigen_Hk is mapped to Hk
            - eigen_J is mapped to J
            - eigen_f is mapped to f
            - eigen_q is mapped to q
            - eigen_Jinv is NOT mapped
        */
        double Ek = 0.5 * norm_squared(f);
        Hk = J.transpose_times_self() + eye6(Ek);
        eigen_Jinv = eigen_Hk.ldlt().solve(eigen_J.transpose());
        eigen_q -= eigen_Jinv * eigen_f;

        if (Ek < 1e-6)
        {
            result_joint = q;
            success = true;
            break;
        }
    }
    return success;
}
