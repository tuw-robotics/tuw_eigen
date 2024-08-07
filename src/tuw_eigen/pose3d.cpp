
#include <cmath>
#include <iostream>
#include "tuw_eigen/pose3d.hpp"

using namespace tuw_eigen;

Pose3D::Pose3D(double x, double y, double z) 
: Transform(MatrixType::Identity()){
  this->translate(Eigen::Vector3d(x, y, z));
}

Pose3D::Pose3D(const Pose3D &src) 
: Transform(src)
{
}
Pose3D::Pose3D(const Eigen::Transform<double, 3, Eigen::Affine> &src)
: Transform(src)
{
}

Pose3D::Pose3D(const Point3D &position, const Point3D &point_ahead)
: Transform(MatrixType::Identity()){
set(position, point_ahead);
}

Pose3D& Pose3D::set(const Point3D &position, const Point3D &point_ahead){ 
    // Compute direction vector from origin to target
    Eigen::Vector3d direction = (point_ahead - position).normalized();
    // Handle the case when up and target are collinear


    // Compute yaw 
    double yaw = std::atan2(direction.y(), direction.x());


    // Compute yaw 
    double pitch = std::asin(-direction.z());

    Eigen::Matrix3d  rotationMatrix;
    rotationMatrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

    this->linear() = rotationMatrix;
    this->translation() = position;

    return *this;
}

double &Pose3D::x()
{
    //return this->translation().x();
    return this->m_matrix(0,3);
}
const double &Pose3D::x() const
{
    //return this->translation().x();
    return this->m_matrix(0,3);
}

const double &Pose3D::y() const
{
    //return this->translation().y();
    return this->m_matrix(1,3);
}

double &Pose3D::y()
{
    //return this->translation().y();
    return this->m_matrix(1,3);
}

const double &Pose3D::z() const
{
    //return this->translation().z();
    return this->m_matrix(2,3);
}

double &Pose3D::z()
{
    //return this->translation().z();
    return this->m_matrix(2,3);
}

Point3D Pose3D::position() const{
    return this->translation();
}
std::string Pose3D::str(const char *format) const
{
    Vector3d t = this->translation();
    Vector3d r = this->eulerAngles();
    char str[0xFF];
    sprintf(str, format, t[0], t[1], t[2], r[0], r[1], r[2]);
    return std::string(str);
}


Vector3d Pose3D::eulerAngles() const{
    return this->rotation().eulerAngles(0, 1, 2);
}

const Pose3D Pose3D::Identity(){
    return Pose3D(MatrixType::Identity());
}