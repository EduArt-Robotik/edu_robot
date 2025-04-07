/**
 * Copyright EduArt Robotik GmbH 2025
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <Eigen/Core>

namespace eduart {
namespace robot {

template <typename DataType>
class Plane
{
public:
  using Point  = Eigen::Vector<DataType, 3>;
  using Vector = Eigen::Vector<DataType, 3>;

  Plane(const Point& origin, const Point& a, const Point& b)
    : _origin(origin)
    , _n((a - origin).cross(b - origin)) 
  { }
  Plane(const Point& origin, const Vector& n)
    : _origin(origin)
    , _n(n)
  { }

  inline const Point& origin() const { return _origin; }
  inline const Point& n() const { return _n; }

private:
  Point _origin;
  Vector _n;
};

} // end namespace eduart
} // end namespace robot

namespace std {

template <typename DataType>
inline ostream& operator<<(ostream& os, const eduart::robot::Plane<DataType>& plane)
{
  os << "[p = " << plane.origin().transpose() << ", n = " << plane.n().transpose() << "]";
  return os;
}

} // end namespace std