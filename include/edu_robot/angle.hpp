/**
 * Copyright EduArt Robotik GmbH 2022
 * This class is copied from https://github.com/franc0r/libfrancor/blob/master/francor_base/include/francor_base/angle.h
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <cmath>
#include <ostream>

namespace eduart {
namespace robot {

class Angle
{
public:
  constexpr Angle(const double radian = 0.0) : _value(radian) { }
  ~Angle() = default;

  inline constexpr double radian() const noexcept { return _value; }
  inline constexpr double degree() const noexcept { return convertToDegree(_value); }

  inline constexpr void setRadian(const double value) noexcept { _value = value; }
  inline constexpr void setDegree(const double value) noexcept { _value = convertToRadian(value); }

  inline constexpr Angle operator+(const Angle& operant) const noexcept { return { _value + operant._value }; }
  inline constexpr Angle operator+(const double operant) const noexcept { return { _value + operant        }; }
  inline constexpr Angle operator-(const Angle& operant) const noexcept { return { _value - operant._value }; }
  inline constexpr Angle operator-(const double operant) const noexcept { return { _value - operant        }; }
  inline constexpr Angle operator/(const double operant) const noexcept { return { _value / operant        }; }
  inline constexpr Angle operator*(const double operant) const noexcept { return { _value * operant        }; }

  inline constexpr Angle& operator+=(const Angle& operant) noexcept { _value += operant._value; return *this; }
  inline constexpr Angle& operator+=(const double operant) noexcept { _value += operant       ; return *this; }
  inline constexpr Angle& operator-=(const Angle& operant) noexcept { _value -= operant._value; return *this; }
  inline constexpr Angle& operator-=(const double operant) noexcept { _value -= operant       ; return *this; }
  inline constexpr Angle& operator= (const double operant) noexcept { _value  = operant       ; return *this; }

  inline constexpr operator double() const noexcept { return _value; }

  static constexpr Angle createFromDegree(const double angle) { return { convertToRadian(angle) }; }

private:
  static constexpr double convertToDegree(const double value) { return value * 180.0 / M_PI; }
  static constexpr double convertToRadian(const double value) { return value * M_PI / 180.0; }

protected:
  double _value;
};


namespace impl {

enum class AngleRange {
  PI_TO_PI,
  NULL_TO_2PI,
  PI_2_TO_PI_2,
};

template<AngleRange Range>
class NormalizedAngle : public Angle
{
public:
  constexpr NormalizedAngle(const double radian = 0.0) : Angle(radian) { this->normalize(); }
  constexpr NormalizedAngle(const Angle& angle) : Angle(angle) { this->normalize(); }
  ~NormalizedAngle() = default;

  inline constexpr void setRadian(const double value) noexcept { Angle::setRadian(value); this->normalize(); }
  inline constexpr void setDegree(const double value) noexcept { Angle::setDegree(value); this->normalize(); }

  inline constexpr NormalizedAngle operator+(const Angle& operant) const noexcept { return { Angle::operator+(operant) }; }
  inline constexpr NormalizedAngle operator+(const double operant) const noexcept { return { Angle::operator+(operant) }; }
  inline constexpr NormalizedAngle operator-(const Angle& operant) const noexcept { return { Angle::operator-(operant) }; }
  inline constexpr NormalizedAngle operator-(const double operant) const noexcept { return { Angle::operator-(operant) }; }

  inline constexpr NormalizedAngle& operator+=(const Angle& operant) { Angle::operator+=(operant); this->normalize(); return *this;}
  inline constexpr NormalizedAngle& operator+=(const double operant) { Angle::operator+=(operant); this->normalize(); return *this;}
  inline constexpr NormalizedAngle& operator-=(const Angle& operant) { Angle::operator-=(operant); this->normalize(); return *this;}
  inline constexpr NormalizedAngle& operator-=(const double operant) { Angle::operator-=(operant); this->normalize(); return *this;}

  inline constexpr NormalizedAngle& operator=(const Angle& operant) { Angle::operator=(operant); this->normalize(); return *this; }
  inline constexpr NormalizedAngle& operator=(const double operant) { Angle::operator=(operant); this->normalize(); return *this; }

  constexpr void normalize();
};

template<>
inline constexpr void NormalizedAngle<AngleRange::PI_2_TO_PI_2>::normalize()
{
  // add or sub pi/2 from angle until it is in range of [-pi/2, pi/2]
  while (_value >   M_PI_2) _value -= M_PI;
  while (_value <= -M_PI_2) _value += M_PI;
}

template<>
inline constexpr void NormalizedAngle<AngleRange::PI_TO_PI>::normalize()
{
  // add or sub 2pi from angle until it is in range of ]-pi, pi]
  while (_value >   M_PI) _value -= 2.0 * M_PI;
  while (_value <= -M_PI) _value += 2.0 * M_PI;
}

template<>
inline constexpr void NormalizedAngle<AngleRange::NULL_TO_2PI>::normalize()
{
  // add or sub 2pi from angle until it is in range of [0, pi[
  while (_value >= 2.0 * M_PI) _value -= 2.0 * M_PI;
  while (_value  <        0.0) _value += 2.0 * M_PI;
}

} // end namespace impl

using AnglePiToPi   = impl::NormalizedAngle<impl::AngleRange::PI_TO_PI>;
using AnglePi2ToPi2 = impl::NormalizedAngle<impl::AngleRange::PI_2_TO_PI_2>;
using Angle0To2Pi   = impl::NormalizedAngle<impl::AngleRange::NULL_TO_2PI>;

} // end namespace robot
} // end namespace eduart



namespace std {

inline ostream& operator<<(ostream& os, const eduart::robot::Angle& angle)
{
  os << "angle [radian = " << angle.radian() << ", degree = " << angle.degree() << "]";

  return os;
}

} // end namespace std