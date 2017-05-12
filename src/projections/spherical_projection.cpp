// Copyright (C) 2017  I. Bogoslavskyi, C. Stachniss, University of Bonn

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "projections/spherical_projection.h"

#include <vector>
int i_count=0;
int  points_flag[16][870];
namespace depth_clustering {

void SphericalProjection::InitFromPoints(const std::vector<RichPoint>& points) {
  this->CheckCloudAndStorage(points);


   for(int i=0;i<16;++i){
     for(int j=0;j<870;j++){

        points_flag[i][j]=0;

     }
   }

  for (size_t index = 0; index < points.size(); ++index) {

    const auto& point = points[index];
    float dist_to_sensor = point.DistToSensor3D();
    if (dist_to_sensor < 0.01f) {
      continue;
    }


    auto angle_rows = Radians::FromRadians(asin(point.z() / dist_to_sensor));
    auto angle_cols = Radians::FromRadians(atan2(point.y(), point.x()));
    size_t bin_rows = this->_params.RowFromAngle(angle_rows);
    size_t bin_cols = this->_params.ColFromAngle(angle_cols);


    // adding point pointer

    this->at(bin_rows, bin_cols).points().push_back(index);

    points_flag[bin_rows][bin_cols]=index;//在这里记录index　相对应的raw ,col .


    auto& current_written_depth =
        this->_depth_image.template at<float>(bin_rows, bin_cols);
    if (current_written_depth < dist_to_sensor) {
      // write this point to the image only if it is closer
      current_written_depth = dist_to_sensor;
    }
    i_count++;
  }

 // std::cout<<"Now it runs here the value of the i_count is :!!!"<<i_count<<std::endl;
  //i_count=0;
}

typename CloudProjection::Ptr SphericalProjection::Clone() const {
  return typename CloudProjection::Ptr(new SphericalProjection(*this));
}

}  // namespace depth_clustering
