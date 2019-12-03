#pragma once
#include <iostream>
#include <vector>
#include <cmath>
using std::sqrt;

typedef unsigned int uint;

struct Vec3;
struct Vec6;
struct Vec12;
struct Mat3;
struct Mat6;
struct Mat12x6; // used in the inverse kinematics of the ur10e (jacobian)



inline bool small(double v, double eps = 1e-6) {return v > -eps && v < eps;}

// vector of 3 doubles (always initialized to zero)
struct Vec3
{
    // data
    union
    {
        double data[3] = {0};
        struct
        {
            double x, y, z;
        };
    };

    Vec3() {};
    Vec3(std::initializer_list<double> rhs);
    void operator=(std::initializer_list<double> rhs);
    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}

    // methods
    void zero();
    void constant(double value);

    double& operator()(uint index); //safe
    double& operator[](uint index); //unsafe
    Vec3& operator=(const Vec3& rhs);
    void operator+=(Vec3 rhs);
    Vec3 operator+(Vec3 rhs);
    void operator-=(Vec3 rhs);
    Vec3 operator-(Vec3 rhs);
    Vec3 operator*(double rhs);
    void operator*=(double rhs);
    void operator/=(double rhs);

    Vec3 cross(Vec3 rhs);
    double dot(Vec3 rhs);
};

// vector of 6 doubles (always initialized to zero)
struct Vec6
{
    union
    {
        double data[6] = {0};
        struct {double x, y, z, wx, wy, wz;};
        struct {double j1, j2, j3, j4, j5, j6;};
    };

    Vec6() {};
    Vec6(std::initializer_list<double> rhs);
    Vec6(std::vector<double> rhs);
    void operator=(std::initializer_list<double> rhs);
    void operator+=(const Vec6& rhs);
    Vec6 operator+(const Vec6& rhs);
    Vec3 first3();
    Vec3 last3();

    double& operator()(uint index); // safe
    double& operator[](uint index); // unsafe
    double operator[](uint index) const;
    void operator=(const std::vector<double>& rhs);
    void operator*=(double rhs);
    Vec6 operator*(double rhs) const;
    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}
};

struct Vec12
{
    double data[12] = {0};

    Vec12() {};
    Vec12(std::initializer_list<double> rhs);
    void operator=(std::initializer_list<double> rhs);
    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}

    double& operator()(uint index); // safe
    double& operator[](uint index); // unsafe
};

struct Quat
{
    union
    {
        double data[4] = {1, 0, 0, 0};
        struct
        {
            double w; // cos(alpha/2)
            double x; // ux * sin(alpha/2)
            double y; // uy * sin(alpha/2)
            double z; // uz * sin(alpha/2)
        };
    };

    Quat() {}
    Quat(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}
    void operator=(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}

    void operator*=(const Quat& rhs);
    Quat operator*(const Quat& rhs) const;
    Mat3 to_matrix();
    void normalize();
    double norm_squared();
    double norm();
    bool is_unit(double eps = 1e-6);
};

// 3x3 matrix of doubles (always initialized to zero)
struct Mat3
{
    union
    {
        double data[9] = {0};
        struct {double a,b,c,d,e,f,g,h,i;};
    };

    Mat3() {}
    Mat3(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}
    void operator=(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}

    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}

    void zero();
    void constant(double value);
    void operator=(const Mat3& rhs);
    void operator+=(const Mat3& rhs);
    void operator-=(const Mat3& rhs);
    Mat3 operator-(const Mat3& rhs);
    Vec3 operator*(const Vec3& rhs) const;
    Mat3 operator*(const Mat3& rhs) const;
    Mat3 transpose() const;
    Mat3 operator*(double rhs) const;
    void operator*=(double rhs);
    void operator*=(Mat3& rhs);
    double& operator()(unsigned row, unsigned col); // safe
    double& operator[](unsigned i);                 // unsafe faster
    double* pointer_to(uint row, uint col);

    Vec3 col(uint col);
    bool is_unit(double eps=1e-5);
    void normalize();
};
Mat3 eye3();
Mat3 eye3(double value);

// 3x3 matrix of doubles (always initialized to zero)
struct Mat6
{
    //-- data members
    double data[36] = {0};

    Mat6() {}
    Mat6(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}
    void operator=(std::initializer_list<double> rhs) {std::copy(rhs.begin(), rhs.end(), data);}

    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}

    void zero();
    void constant(double value);
    void operator=(const Mat6& rhs);
    void operator+=(const Mat6& rhs);
    void operator-=(const Mat6& rhs);
    void operator*=(double rhs);
    Mat6 operator+(const Mat6& rhs) const;
    Mat6 operator-(const Mat6& rhs) const;
    Vec6 operator*(const Vec6& rhs) const;
    Mat6 operator*(const Mat6& rhs) const;
    Mat6 transpose() const;
    Mat6 operator*(double rhs) const;
    double& operator()(unsigned row, unsigned col); // safe
    double& operator[](unsigned i);                 // unsafe faster
    double* pointer_to(uint row, uint col);

    Vec6 col(uint col);
};
Mat6 eye6();
Mat6 eye6(double value);

// column major matrix used for the jacobian matrix
struct Mat12x6
{
    double data[72] = {0};

    double* begin() {return std::begin(data);}
    double* end() {return std::end(data);}

    double* pointer_to(uint row, uint col);
    Mat6 transpose_times_self();
};



inline double norm_squared(Vec3 v)
{
    return v.x * v.x + v.y * v.y + v.z * v.z;
}
inline double norm(Vec3 v)
{
    return sqrt(norm_squared(v));
}
inline double norm_squared(Vec6 v)
{
    double result = 0.0;
    for (uint i = 0; i < 6; i++)
        result += v.data[i] * v.data[i];
    return result;
}
inline double norm_squared(Vec12 v)
{
    double result = 0.0;
    for (uint i = 0; i < 12; i++)
        result += v.data[i] * v.data[i];
    return result;
}
inline double det(Mat3& M)
{
    double result;
    result = M.a*(M.e*M.i - M.f*M.h);
    result -= M.b*(M.d*M.i - M.g*M.f);
    result += M.c*(M.d*M.h - M.e*M.g);
    return result;
}
inline bool small(const Vec6& v)
{
    return small(norm_squared(v));
}
inline bool small(const Vec3& v)
{
    return small(norm_squared(v));
}

inline bool is_unit(const Vec3& v)
{
    return small(norm_squared(v) - 1.0);
}
