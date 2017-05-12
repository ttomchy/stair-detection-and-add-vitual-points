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

#include "./visualizer.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <limits>
#include <string>
#include <vector>

extern cv::Mat binary_point_flag;
extern int  points_flag[16][870];
extern cv::Mat ratio_flag;
namespace depth_clustering {

using std::string;
using std::vector;
using std::array;
using std::to_string;
int  i_count2=0;

float ratio_i;
using std::map;
using std::mutex;
using std::string;
using std::vector;
using std::thread;
using std::lock_guard;
using std::unordered_map;

static vector<array<int, 3>> COLORS;

Visualizer::Visualizer(QWidget* parent)
    : QGLViewer(parent), AbstractClient<Cloud>(), _updated{false} {
  _cloud_obj_storer.SetUpdateListener(this);
}

void Visualizer::draw() {
  lock_guard<mutex> guard(_cloud_mutex);
  DrawCloud(_cloud);

/*
  for (const auto& kv : _cloud_obj_storer.object_clouds()) {
    const auto& cluster = kv.second;
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    Eigen::Vector3f extent = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_point(std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest(),
                              std::numeric_limits<float>::lowest());
    Eigen::Vector3f min_point(std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max(),
                              std::numeric_limits<float>::max());

    for (const auto& point : cluster.points()) {
      center = center + point.AsEigenVector();
      min_point << std::min(min_point.x(), point.x()),
          std::min(min_point.y(), point.y()),
          std::min(min_point.z(), point.z());
      max_point << std::max(max_point.x(), point.x()),
          std::max(max_point.y(), point.y()),
          std::max(max_point.z(), point.z());
    }
    center /= cluster.size();
    if (min_point.x() < max_point.x()) {
      extent = max_point - min_point;
    }
    DrawCube(center, extent);
  }

*/


    /*
    for (int r = 0; r < 64; ++r) {
        for (int c = 0; c < 870; ++c) {
            binary_self_me_img.at<uchar>(r,c)=0;
        }
    }
     */
}

void Visualizer::init() {
  setSceneRadius(100.0);
  camera()->showEntireScene();
  glDisable(GL_LIGHTING);
}


 void Visualizer::DrawCloud(const Cloud& cloud) {
  glPushMatrix();
  glBegin(GL_POINTS);

   // for (const auto& point : cloud.points()) { //在这里会遍历所有的point 点
    //   std::cout<<"THE VALUE OF THE  cloud.size() IS :"<<cloud.size()<<std::endl;


  int ground_count2=0;

  for (int index = 0; index < cloud.size(); ++index) {


      const auto& point = cloud[index];
      for (int r = 0; r < 16; ++r) {
          for (int c = 0; c < 870; ++c) {


          if(points_flag[r][c]==index ){  //不是地面的点用绿色来表示，地面的点用红色来表示。

              if( binary_point_flag.at<uchar>(r,c)==0) {

                  if(point.x()>0){

                  glColor3f(0.0f, 1.0f, 0.0f);//绿色
                  glVertex3f(point.x(), point.y(), point.z());

                  ratio_i= ratio_flag.at<float>(r,c);
                     if(ratio_i!=0)
                     {

                         glColor3f(0.0f, 0.0f, 1.0f);//蓝色
                         glVertex3f(ratio_i*point.x(), ratio_i*point.y(), ratio_i*point.z());

                     }

                 //std::cout<<" the valur of the point.x() is :"<<point.x()<<std::endl;

                  }
                 //std::cout<<" the valur of the point.y() is :"<<point.y()<<std::endl;
                 //std::cout<<" the valur of the point.z() is :"<<point.z()<<std::endl;
              }
              else{
                  glColor3f(1.0f, 0.0f, 0.0f);
                  glVertex3f(point.x(), point.y(), point.z());
              }
             }
          }
      }
  }
  glEnd();
  glPopMatrix();

}



void Visualizer::DrawCube(const Eigen::Vector3f& center,
                          const Eigen::Vector3f& scale) {
  glPushMatrix();
  glTranslatef(center.x(), center.y(), center.z());
  glScalef(scale.x(), scale.y(), scale.z());
  float volume = scale.x() * scale.y() * scale.z();
  if (volume < 30.0f && scale.x() < 6 && scale.y() < 6 && scale.z() < 6) {
    glColor3f(0.0f, 0.2f, 0.9f);
    glLineWidth(4.0f);
  } else {
    glColor3f(0.3f, 0.3f, 0.3f);
    glLineWidth(1.0f);
  }
  glBegin(GL_LINE_STRIP);

  // Bottom of Box
  glVertex3f(-0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  // Top of Box
  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);
  glVertex3f(0.5, 0.5, -0.5);
  glVertex3f(-0.5, 0.5, -0.5);

  glEnd();

  glBegin(GL_LINES);
  // For the Sides of the Box

  glVertex3f(-0.5, 0.5, -0.5);
  glVertex3f(-0.5, -0.5, -0.5);

  glVertex3f(-0.5, -0.5, 0.5);
  glVertex3f(-0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, 0.5);
  glVertex3f(0.5, 0.5, 0.5);

  glVertex3f(0.5, -0.5, -0.5);
  glVertex3f(0.5, 0.5, -0.5);

  glEnd();
  glPopMatrix();
}

Visualizer::~Visualizer() {}

void Visualizer::OnNewObjectReceived(const Cloud& cloud, const int id) {
  lock_guard<mutex> guard(_cloud_mutex);
  _cloud = cloud;
}

void Visualizer::onUpdate() { this->update(); }

unordered_map<uint16_t, Cloud> ObjectPtrStorer::object_clouds() const {
  lock_guard<mutex> guard(_cluster_mutex);
  return _obj_clouds;
}

void ObjectPtrStorer::OnNewObjectReceived(
    const unordered_map<uint16_t, Cloud>& clouds, const int id) {
  lock_guard<mutex> guard(_cluster_mutex);
  _obj_clouds = clouds;

  if (_update_listener) {
    _update_listener->onUpdate();
  }
}

}  // namespace depth_clustering
