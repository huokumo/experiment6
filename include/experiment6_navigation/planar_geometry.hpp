#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace experiment6 {
struct Pose2 { double x{0}, y{0}, yaw{0}; };
struct Box2 { double x, y, yaw, half_x, half_y; };
inline double wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
inline Pose2 integrate(const Pose2 &p, double vx, double vy, double w, double dt) {
  double dx, dy;
  if (std::abs(w) < 1e-9) { dx = vx * dt; dy = vy * dt; }
  else {
    const double a = std::sin(w * dt) / w;
    const double b = (1.0 - std::cos(w * dt)) / w;
    dx = a * vx - b * vy; dy = b * vx + a * vy;
  }
  return {p.x + std::cos(p.yaw)*dx - std::sin(p.yaw)*dy,
          p.y + std::sin(p.yaw)*dx + std::cos(p.yaw)*dy,
          wrap(p.yaw + w*dt)};
}
// Sweep a circular robot against a conservatively expanded oriented box.
// This is a simulation collision guard, never an input to the obstacle detector.
inline bool segmentHitsBox(double x0, double y0, double x1, double y1,
                           const Box2 &box, double radius) {
  const double c=std::cos(box.yaw), s=std::sin(box.yaw);
  auto local = [&](double x, double y) {
    return std::pair<double,double>{c*(x-box.x)+s*(y-box.y),
                                  -s*(x-box.x)+c*(y-box.y)};
  };
  const auto a=local(x0,y0), b=local(x1,y1);
  const double origin[2]={a.first,a.second};
  const double delta[2]={b.first-a.first,b.second-a.second};
  const double extent[2]={box.half_x+radius,box.half_y+radius};
  double lo=0, hi=1;
  for(int i=0;i<2;++i) {
    if(std::abs(delta[i])<1e-12) {
      if(origin[i]<-extent[i] || origin[i]>extent[i]) return false;
    } else {
      double t0=(-extent[i]-origin[i])/delta[i];
      double t1=( extent[i]-origin[i])/delta[i];
      if(t0>t1) std::swap(t0,t1);
      lo=std::max(lo,t0); hi=std::min(hi,t1);
      if(lo>hi) return false;
    }
  }
  return true;
}
inline bool blocked(const Pose2 &a, const Pose2 &b, double radius,
                    const std::vector<Box2> &boxes) {
  for(const auto &box: boxes)
    if(segmentHitsBox(a.x,a.y,b.x,b.y,box,radius)) return true;
  return false;
}
}  // namespace experiment6
