#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "quad_midbridge_mpcc/mpcc_types.hpp"

namespace quad_midbridge_mpcc
{

inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(x, hi));
}

inline double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3 add(const Vec3 & a, const Vec3 & b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vec3 sub(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vec3 mul(const Vec3 & a, double s)
{
  return {a.x * s, a.y * s, a.z * s};
}

inline double norm(const Vec3 & v)
{
  return std::sqrt(dot(v, v));
}

inline double wrapAngle(double a)
{
  constexpr double kPi = 3.14159265358979323846;
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

inline Vec3 toVec3(const geometry_msgs::msg::Point & p)
{
  return {static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
}

inline geometry_msgs::msg::Vector3 toRosVector3(const Vec3 & v)
{
  geometry_msgs::msg::Vector3 out;
  out.x = v.x;
  out.y = v.y;
  out.z = v.z;
  return out;
}

inline double estimatePathLength(const quad_midbridge_msgs::msg::LocalIntent & intent)
{
  if (intent.centerline_points.size() < 2) {
    return 0.0;
  }
  double s = 0.0;
  for (size_t i = 0; i + 1 < intent.centerline_points.size(); ++i) {
    s += norm(sub(toVec3(intent.centerline_points[i + 1]), toVec3(intent.centerline_points[i])));
  }
  return s;
}

inline PathFrame projectToPath(const quad_midbridge_msgs::msg::LocalIntent & intent, const Vec3 & query)
{
  PathFrame best{};
  if (intent.centerline_points.size() < 2) {
    return best;
  }

  double accumulated_s = 0.0;
  double best_dist2 = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i + 1 < intent.centerline_points.size(); ++i) {
    const Vec3 p0 = toVec3(intent.centerline_points[i]);
    const Vec3 p1 = toVec3(intent.centerline_points[i + 1]);
    const Vec3 d = sub(p1, p0);
    const double len2 = dot(d, d);
    const double len = std::sqrt(std::max(len2, 1e-12));
    if (len < 1e-6) {
      continue;
    }

    const Vec3 rel = sub(query, p0);
    const double alpha = clamp(dot(rel, d) / len2, 0.0, 1.0);
    const Vec3 proj = add(p0, mul(d, alpha));
    const Vec3 err = sub(query, proj);
    const double dist2 = dot(err, err);
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      const Vec3 tangent = mul(d, 1.0 / len);
      best.position = proj;
      best.tangent = tangent;
      best.s = accumulated_s + alpha * len;
      best.lag_error = dot(sub(query, proj), tangent);
      const Vec3 contour_vec = sub(sub(query, proj), mul(tangent, best.lag_error));
      best.contour_error = norm(contour_vec);
      best.distance_to_path = std::sqrt(dist2);
      best.valid = true;
    }
    accumulated_s += len;
  }
  return best;
}

inline PathFrame samplePathAtS(const quad_midbridge_msgs::msg::LocalIntent & intent, double target_s)
{
  PathFrame out{};
  if (intent.centerline_points.size() < 2) {
    return out;
  }

  double accumulated_s = 0.0;
  for (size_t i = 0; i + 1 < intent.centerline_points.size(); ++i) {
    const Vec3 p0 = toVec3(intent.centerline_points[i]);
    const Vec3 p1 = toVec3(intent.centerline_points[i + 1]);
    const Vec3 d = sub(p1, p0);
    const double len = norm(d);
    if (len < 1e-6) {
      continue;
    }

    if (target_s <= accumulated_s + len || i + 2 == intent.centerline_points.size()) {
      const double local = clamp((target_s - accumulated_s) / len, 0.0, 1.0);
      out.position = add(p0, mul(d, local));
      out.tangent = mul(d, 1.0 / len);
      out.s = accumulated_s + local * len;
      out.valid = true;
      return out;
    }
    accumulated_s += len;
  }

  out.position = toVec3(intent.centerline_points.back());
  out.tangent = {1.0, 0.0, 0.0};
  out.s = accumulated_s;
  out.valid = true;
  return out;
}

}  // namespace quad_midbridge_mpcc
