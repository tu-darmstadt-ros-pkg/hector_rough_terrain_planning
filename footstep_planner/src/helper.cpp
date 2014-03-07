// SVN $HeadURL: http://alufr-ros-pkg.googlecode.com/svn/trunk/humanoid_stacks/humanoid_navigation/footstep_planner/src/helper.cpp $
// SVN $Id: helper.cpp 4146 2013-05-13 13:57:41Z garimort@informatik.uni-freiburg.de $

/*
 * A footstep planner for humanoid robots
 *
 * Copyright 2010-2011 Johannes Garimort, Armin Hornung, University of Freiburg
 * http://www.ros.org/wiki/footstep_planner
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <footstep_planner/helper.h>

namespace footstep_planner
{
bool
collision_check(double x, double y, double cos_theta, double sin_theta,
                double height, double width,
                int accuracy, const gridmap_2d::GridMap2D &distance_map)
{
  double d = distance_map.distanceMapAt(x, y);
  if (d < 0.0) // if out of bounds => collision
    return true;
  d -= distance_map.getResolution();

  double r_o = 0.5 * sqrt(width*width + height*height);

  if (d >= r_o)
    return false;
  else if (accuracy == 0)
    return false;

  double h_half = 0.5 * height;
  double w_half = 0.5 * width;
  double r_i = std::min(w_half, h_half);

  if (d <= r_i)
    return true;
  else if (accuracy == 1)
    return true;

  double h_new;
  double w_new;
  double delta_x;
  double delta_y;
  if (width < height)
  {
    double h_clear = sqrt(d*d - w_half*w_half);
    h_new = h_half - h_clear;
    w_new = width;
    delta_x = h_clear + 0.5 * h_new;
    delta_y = 0.0;
  }
  else // footWidth >= footHeight
  {
    double w_clear = sqrt(d*d - h_half*h_half);
    h_new = height;
    w_new = w_half - w_clear;
    delta_x = 0.0;
    delta_y = w_clear + 0.5 * w_new;
  }
  double x_shift = cos_theta*delta_x - sin_theta*delta_y;
  double y_shift = sin_theta*delta_x + cos_theta*delta_y;

  return (collision_check(x+x_shift, y+y_shift, cos_theta, sin_theta, h_new, w_new,
                          accuracy, distance_map) ||
          collision_check(x-x_shift, y-y_shift, cos_theta, sin_theta, h_new, w_new,
                          accuracy, distance_map));
}


bool
pointWithinPolygon(int x, int y, const std::vector<std::pair<int, int> >& edges)
{
  int cn = 0;

  // loop through all edges of the polygon
  for(unsigned int i = 0; i < edges.size() - 1; ++i)
  {
    if ((edges[i].second <= y && edges[i + 1].second > y) ||
        (edges[i].second > y && edges[i + 1].second <= y))
    {
      float vt = (float)(y - edges[i].second) /
        (edges[i + 1].second - edges[i].second);
      if (x < edges[i].first + vt * (edges[i + 1].first - edges[i].first))
      {
        ++cn;
      }
    }
  }
  return cn & 1;
}
}
