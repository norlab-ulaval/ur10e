#include "linear.h"
#include <ros/assert.h>





Vec3::Vec3(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 3, "Initializer should contain 3 elements, contains %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
void Vec3::operator=(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 3, "Initializer should contain 3 elements, contains %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
void Vec3::zero()
{
    x = y = z = 0;
}
void Vec3::constant(double value)
{
    x = y = z = value;
}
double& Vec3::operator()(uint index)
{
    ROS_ASSERT_MSG(index < 3, "Vector index out of bounds");
    return data[index];
}
double& Vec3::operator[](uint index)
{
    return data[index];
}
Vec3& Vec3::operator=(const Vec3& rhs)
{
    memcpy(data, rhs.data, 3 * sizeof(double));
    return *this;
}
void Vec3::operator+=(Vec3 rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
}
Vec3 Vec3::operator+(Vec3 rhs)
{
    Vec3 res;
    res.x = x + rhs.x;
    res.y = y + rhs.y;
    res.z = z + rhs.z;
    return res;
}
void Vec3::operator-=(Vec3 rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
}
Vec3 Vec3::operator-(Vec3 rhs)
{
    Vec3 res;
    res.x = x - rhs.x;
    res.y = y - rhs.y;
    res.z = z - rhs.z;
    return res;
}
Vec3 Vec3::operator*(double rhs)
{
    Vec3 res;
    res.x = x * rhs;
    res.y = y * rhs;
    res.z = z * rhs;
    return res;
}
void Vec3::operator*=(double rhs)
{
    x *= rhs;
    y *= rhs;
    z *= rhs;
}
void Vec3::operator/=(double rhs)
{
    x/=rhs;
    y/=rhs;
    z/=rhs;
}
Vec3 Vec3::cross(Vec3 rhs)
{
    Vec3 res;
    res[0] = y * rhs.z - z * rhs.y;
    res[1] = z * rhs.x - x * rhs.z;
    res[2] = x * rhs.y - y * rhs.x;
    return res;
}
double Vec3::dot(Vec3 rhs)
{
    double res = x * rhs.x + y * rhs.y + z * rhs.z;
    return res;
}
Vec3 operator*(double lhs, Vec3& rhs)
{
    return rhs * lhs;
}




Vec6::Vec6(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 6, "Initializer should contain 6 elements, contains %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
Vec6::Vec6(std::vector<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 6, "Initializer should contain 6 elements, contains %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
void Vec6::operator=(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 6, "Initializer should contain 6 elements, contains %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
Vec3 Vec6::first3()
{
    Vec3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}
Vec3 Vec6::last3()
{
    Vec3 result;
    result.x = wx;
    result.y = wy;
    result.z = wz;
    return result;
}
double& Vec6::operator()(uint index)
{
    ROS_ASSERT_MSG(index < 6, "Vector index out of bounds");
    return data[index];
}
double& Vec6::operator[](uint index)
{
    return data[index];
}
double Vec6::operator[](uint index) const
{
    return data[index];
}
void Vec6::operator=(const std::vector<double>& rhs)
{
    ROS_ASSERT_MSG(
        rhs.size() == 6,
        "can only assign a vector that is the right size. size: %lu", rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
void Vec6::operator+=(const Vec6& rhs)
{
    data[0] += rhs[0];
    data[1] += rhs[1];
    data[2] += rhs[2];
    data[3] += rhs[3];
    data[4] += rhs[4];
    data[5] += rhs[5];
}
void Vec6::operator-=(const Vec6& rhs)
{
    j1 -= rhs.j1;
    j2 -= rhs.j2;
    j3 -= rhs.j3;
    j4 -= rhs.j4;
    j5 -= rhs.j5;
    j6 -= rhs.j6;
}
Vec6 Vec6::operator+(const Vec6& rhs) const
{
    auto result = *this;
    result += rhs;
    return result;
}
Vec6 Vec6::operator-(const Vec6& rhs) const
{
    auto result = *this;
    result -= rhs;
    return result;
}
void Vec6::operator*=(double rhs)
{
    j1 *= rhs;
    j2 *= rhs;
    j3 *= rhs;
    j4 *= rhs;
    j5 *= rhs;
    j6 *= rhs;
}
Vec6 Vec6::operator*(double rhs) const
{
    Vec6 result = *this;
    result *= rhs;
    return result;
}





Vec12::Vec12(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 12, "Initializer should contain 12 elements, contains %lu",
        rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}
void Vec12::operator=(std::initializer_list<double> rhs)
{
    ROS_ASSERT_MSG(rhs.size() == 12, "Initializer should contain 12 elements, contains %lu",
        rhs.size());
    std::copy(rhs.begin(), rhs.end(), data);
}




void Quat::operator*=(const Quat& rhs)
{
    w = w*rhs.w - x*rhs.x - y*rhs.y - z*rhs.z;
    x = w*rhs.x + x*rhs.w + y*rhs.z - z*rhs.y;
    y = w*rhs.y - x*rhs.z * y*rhs.w + z*rhs.x;
    z = w*rhs.z + x*rhs.y - y*rhs.x + z*rhs.w;
}
Quat Quat::operator*(const Quat& rhs) const
{
    Quat result = *this;
    result *= rhs;
    return result;
}
Mat3 Quat::to_matrix()
{
    Mat3 result;
    // remember column major storage
    result[0] = 1.0 - 2.0*y*y - 2.0*z*z;
    result[1] = 2.0*x*y + 2.0*w*z;
    result[2] = 2.0*x*z - 2.0*w*y;

    result[3] = 2.0*x*y - 2.0*w*z;
    result[4] = 1.0 - 2.0*x*x - 2.0*z*z;
    result[5] = 2.0*y*z - 2.0*w*x;

    result[6] = 2.0*x*z + 2.0*w*y;
    result[7] = 2.0*y*z + 2.0*w*x;
    result[8] = 1.0 - 2*x*x - 2.0*y*y;
    return result;
}
void Quat::normalize()
{
    double div = 1.0/norm();
    w*=div;
    x*=div;
    y*=div;
    z*=div;
}
double Quat::norm_squared()
{
    return w*w + x*x + y*y + z*z;
}
double Quat::norm()
{
    return sqrt(norm_squared());
}
bool Quat::is_unit(double eps)
{
    return (norm_squared() < eps);
}




void Mat3::zero()
{
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
    data[8] = 0;
}
void Mat3::constant(double value)
{
    data[0] = value;
    data[1] = value;
    data[2] = value;
    data[3] = value;
    data[4] = value;
    data[5] = value;
    data[6] = value;
    data[7] = value;
    data[8] = value;
}
void Mat3::operator=(const Mat3& rhs)
{
    for (unsigned i = 0; i < 9; i++)
        data[i] = rhs.data[i];
}
void Mat3::operator+=(const Mat3& rhs)
{
    for (unsigned i = 0; i < 9; i++)
        data[i] += rhs.data[i];
}
void Mat3::operator-=(const Mat3& rhs)
{
    for (unsigned i = 0; i < 9; i++)
        data[i] -= rhs.data[i];
}
Mat3 Mat3::operator-(const Mat3& rhs)
{
    Mat3 result = *this;
    result -= rhs;
    return result;
}
Vec3 Mat3::operator*(const Vec3& rhs) const
{
    Vec3 res;
    res.x = data[0] * rhs.data[0] + data[3] * rhs.data[1] + data[6] * rhs.data[2];
    res.y = data[1] * rhs.data[0] + data[4] * rhs.data[1] + data[7] * rhs.data[2];
    res.z = data[2] * rhs.data[0] + data[5] * rhs.data[1] + data[8] * rhs.data[2];
    return res;
}
Mat3 Mat3::operator*(const Mat3& rhs) const
{
    Mat3 res;
    res[0] = data[0] * rhs.data[0] + data[3] * rhs.data[1] + data[6] * rhs.data[2];
    res[1] = data[1] * rhs.data[0] + data[4] * rhs.data[1] + data[7] * rhs.data[2];
    res[2] = data[2] * rhs.data[0] + data[5] * rhs.data[1] + data[8] * rhs.data[2];

    res[3] = data[0] * rhs.data[3] + data[3] * rhs.data[4] + data[6] * rhs.data[5];
    res[4] = data[1] * rhs.data[3] + data[4] * rhs.data[4] + data[7] * rhs.data[5];
    res[5] = data[2] * rhs.data[3] + data[5] * rhs.data[4] + data[8] * rhs.data[5];

    res[6] = data[0] * rhs.data[6] + data[3] * rhs.data[7] + data[6] * rhs.data[8];
    res[7] = data[1] * rhs.data[6] + data[4] * rhs.data[7] + data[7] * rhs.data[8];
    res[8] = data[2] * rhs.data[6] + data[5] * rhs.data[7] + data[8] * rhs.data[8];
    return res;
}
Mat3 Mat3::transpose() const
{
    Mat3 res;
    res.data[0] = data[0];
    res.data[1] = data[3];
    res.data[2] = data[6];
    res.data[3] = data[1];
    res.data[4] = data[4];
    res.data[5] = data[7];
    res.data[6] = data[2];
    res.data[7] = data[5];
    res.data[8] = data[8];
    return res;
}
Mat3 Mat3::operator*(double rhs) const
{
    Mat3 res;
    for (uint i = 0; i < 9; i++)
        res.data[i] = data[i] * rhs;
    return res;
}
void Mat3::operator*=(double rhs)
{
    for (unsigned i = 0; i < 9; i++)
        data[i] *= rhs;
}
void Mat3::operator*=(Mat3& rhs)
{
    data[0] = data[0] * rhs.data[0] + data[3] * rhs.data[1] + data[6] * rhs.data[2];
    data[1] = data[1] * rhs.data[0] + data[4] * rhs.data[1] + data[7] * rhs.data[2];
    data[2] = data[2] * rhs.data[0] + data[5] * rhs.data[1] + data[8] * rhs.data[2];

    data[3] = data[0] * rhs.data[3] + data[3] * rhs.data[4] + data[6] * rhs.data[5];
    data[4] = data[1] * rhs.data[3] + data[4] * rhs.data[4] + data[7] * rhs.data[5];
    data[5] = data[2] * rhs.data[3] + data[5] * rhs.data[4] + data[8] * rhs.data[5];

    data[6] = data[0] * rhs.data[6] + data[3] * rhs.data[7] + data[6] * rhs.data[8];
    data[7] = data[1] * rhs.data[6] + data[4] * rhs.data[7] + data[7] * rhs.data[8];
    data[8] = data[2] * rhs.data[6] + data[5] * rhs.data[7] + data[8] * rhs.data[8];
}
double& Mat3::operator()(uint row, uint col)
{
    ROS_ASSERT_MSG(col < 3 && row < 3, "Index out of bounds, col: %u, row: %u.", col, row);
    return data[col * 3 + row];
}
double& Mat3::operator[](uint i)
{
    return data[i];
}
double* Mat3::pointer_to(uint row, uint col)
{
    ROS_ASSERT_MSG(row < 3 && col < 3, "index out of bounds, (%u, %u)", row, col);
    return &data[col * 3 + row];
}
Vec3 Mat3::col(uint col_index)
{
    ROS_ASSERT_MSG(col_index < 3, "Index out of bounds, col value must be 0, 1, 2. col:%u", col_index);
    Vec3 result;
    std::memcpy(result.data, data, 3 * sizeof(double));
    return result;
}
bool Mat3::is_unit(double eps)
{
    Mat3 I = *this * transpose();

    return small(I[0] - 1.0, eps)
        && small(I[1], eps)
        && small(I[2], eps)
        && small(I[3], eps)
        && small(I[4] - 1.0, eps)
        && small(I[5], eps)
        && small(I[6], eps)
        && small(I[7], eps)
        && small(I[8] - 1.0, eps)
        && small(det(*this) - 1.0, eps);
}
void Mat3::normalize()
{
    Vec3 x = this->col(0);
    Vec3 y = this->col(1);
    double error_over_2 = x.dot(y) * 0.5;
    Vec3 x_ort = x - error_over_2*y;
    Vec3 y_ort = y - error_over_2*x;
    Vec3 z_ort = x.cross(y);
    Vec3 x_n = 0.5*(3.0-x_ort.dot(x_ort)) * x_ort;
    Vec3 y_n = 0.5*(3.0-y_ort.dot(y_ort)) * y_ort;
    Vec3 z_n = 0.5*(3.0-z_ort.dot(z_ort)) * z_ort;
    std::copy(x_n.begin(), x_n.end(), this->pointer_to(0,0));
    std::copy(x_n.begin(), x_n.end(), this->pointer_to(0,1));
    std::copy(x_n.begin(), x_n.end(), this->pointer_to(0,2));
}
Mat3 eye3(double value)
{
    Mat3 result;
    for (uint i = 0; i < 3; i++)
        result[4 * i] = value;
    return result;
}
Mat3 eye3()
{
    return eye3(1.0);
}





void Mat6::zero()
{
    for (uint i = 0; i < 36; i++)
        data[i] = 0.0;
}
void Mat6::constant(double value)
{
    for (uint i = 0; i < 36; i++)
        data[i] = value;
}
void Mat6::operator=(const Mat6& rhs)
{
    for (unsigned i = 0; i < 36; i++)
        data[i] = rhs.data[i];
}
void Mat6::operator+=(const Mat6& rhs)
{
    for (unsigned i = 0; i < 36; i++)
        data[i] += rhs.data[i];
}
void Mat6::operator-=(const Mat6& rhs)
{
    for (unsigned i = 0; i < 36; i++)
        data[i] -= rhs.data[i];
}
Mat6 Mat6::operator-(const Mat6& rhs) const
{
    Mat6 result = *this;
    result -= rhs;
    return result;
}
Mat6 Mat6::operator+(const Mat6& rhs) const
{
    Mat6 result = *this;
    result += rhs;
    return result;
}
Vec6 Mat6::operator*(const Vec6& rhs) const
{
    Vec6 res;
    for (uint i = 0; i < 6; i++)
    {
        res[i] =
            data[i] * rhs.data[0] +
            data[6 + i] * rhs.data[1] +
            data[12 + i] * rhs.data[2] +
            data[18 + i] * rhs.data[3] +
            data[24 + i] * rhs.data[4] +
            data[30 + i] * rhs.data[5];
    }
    return res;
}
Mat6 Mat6::operator*(const Mat6& rhs) const
{
    Mat6 res;
    for (uint row = 0; row < 6; row++)
    {
        for (uint col = 0; col < 6; col++)
        {
            res[row + 6 * col] =
                data[row] * rhs.data[0 + 6 * col] +
                data[6 + row] * rhs.data[1 + 6 * col] +
                data[12 + row] * rhs.data[2 + 6 * col] +
                data[18 + row] * rhs.data[3 + 6 * col] +
                data[24 + row] * rhs.data[4 + 6 * col] +
                data[30 + row] * rhs.data[5 + 6 * col];
        }
    }
    return res;
}
Mat6 Mat6::transpose() const
{
    Mat6 res;
    for (uint i = 0; i < 6; i++)
    {
        for (uint j = 0; j < 6; j++)
        {
            res.data[6 * j + i] = data[6 * i + j];
        }
    }
    return res;
}
Mat6 Mat6::operator*(double rhs) const
{
    Mat6 res = *this;
    res *= rhs;
    return res;
}
void Mat6::operator*=(double rhs)
{
    for (unsigned i = 0; i < 36; i++)
        data[i] *= rhs;
}
double& Mat6::operator()(uint row, uint col)
{
    ROS_ASSERT_MSG(col < 6 && row < 6, "Index out of bounds, col: %u, row: %u.", col, row);
    return data[col * 6 + row];
}
double& Mat6::operator[](uint i)
{
    return data[i];
}
Vec6 Mat6::col(uint col_index)
{
    ROS_ASSERT_MSG(col_index < 6, "Index out of bounds, col value must be 0, 1, 2. col:%u", col_index);
    Vec6 result;
    std::copy(data + (6 * col_index), data + (6 * (col_index + 1)), result.data);
    return result;
}
Mat6 eye6()
{
    Mat6 result;
    for (uint i = 0; i < 6; i++)
        result[7 * i] = 1.0;
    return result;
}
Mat6 eye6(double value)
{
    Mat6 result;
    for (uint i = 0; i < 6; i++)
        result[7 * i] = value;
    return result;
}





double* Mat12x6::pointer_to(uint row, uint col)
{
    ROS_ASSERT_MSG(row < 12 && col < 6, "index out of bounds, (%u, %u)", row, col);
    return &data[col * 12 + row];
}
Mat6 Mat12x6::transpose_times_self()
{
    Mat6 result;
    for (uint i = 0; i < 6; i++)
    {
        for (uint j = 0; j < 6; j++)
        {
            result[6 * j + i] =
                data[12 * i + 0] * data[12 * j + 0] +
                data[12 * i + 1] * data[12 * j + 1] +
                data[12 * i + 2] * data[12 * j + 2] +
                data[12 * i + 3] * data[12 * j + 3] +
                data[12 * i + 4] * data[12 * j + 4] +
                data[12 * i + 5] * data[12 * j + 5] +
                data[12 * i + 6] * data[12 * j + 6] +
                data[12 * i + 7] * data[12 * j + 7] +
                data[12 * i + 8] * data[12 * j + 8] +
                data[12 * i + 9] * data[12 * j + 9] +
                data[12 * i + 10] * data[12 * j + 10] +
                data[12 * i + 11] * data[12 * j + 11];
        }
    }
    return result;
}









