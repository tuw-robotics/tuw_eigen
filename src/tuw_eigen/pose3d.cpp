
#include <cmath>
#include <iostream>
#include "tuw_eigen/pose3d.hpp"

using namespace tuw_eigen;

double &Pose3D::x()
{
    return this->translation().x();
    //return this->m_matrix[3];
}
const double &Pose3D::x() const
{
    return this->translation().x();
    //return this->m_matrix(3,0);
}

const double &Pose3D::y() const
{
    return this->translation().y();
}

double &Pose3D::y()
{
    return this->translation().y();
}

const double &Pose3D::z() const
{
    return this->translation().z();
}

double &Pose3D::z()
{
    return this->translation().z();
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