#include <cmath>
#include <iostream>
#include <stdexcept>
#include "experiment6_navigation/planar_geometry.hpp"

void require(bool condition,const char *message) {
  if(!condition) throw std::runtime_error(message);
}
int main() {
  using namespace experiment6;
  const double pi=std::acos(-1.);
  auto p=integrate({0,0,pi/2},1,0,0,1);
  require(std::abs(p.x)<1e-10&&std::abs(p.y-1)<1e-10,"Body forward rotates into world Y");
  p=integrate({0,0,0},1,0,1,pi/2);
  require(std::abs(p.x-1)<1e-10&&std::abs(p.y-1)<1e-10,"Exact arc integration");
  p=integrate({0,0,0},0,1,0,2);
  require(std::abs(p.y-2)<1e-10,"Lateral motion");
  const Box2 wall{2.5,0,0,.05,2};
  require(segmentHitsBox(0,0,5,0,wall,.45),"Swept guard must prevent tunnelling");
  require(!segmentHitsBox(0,2.6,5,2.6,wall,.45),"Wall endpoint has a passable side");
  require(segmentHitsBox(0,2.3,5,2.3,wall,.45),"Footprint radius must be included");
  require(!segmentHitsBox(0,0,1,0,wall,.45),"Distant wall must not block initial motion");
  const Box2 rotated{0,0,pi/2,2,.05};
  require(segmentHitsBox(-1,0,1,0,rotated,.2),"Rotated collision geometry");
  std::cout<<"8 planar integration/collision checks passed\n";
}
