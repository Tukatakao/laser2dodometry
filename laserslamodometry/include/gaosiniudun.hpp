#include <eigen3/Eigen/Dense>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

double diandaoxian(double x, double y, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
Eigen::RowVector3d yakebi(double dtheta,const Eigen::Vector2d &raw_p, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);
bool gaosiniudundiedai(const std::vector<Eigen::Vector2d>& ref_points,
                       const std::vector<Eigen::Vector2d>& curr_points,
                       Eigen::Vector3d& xi);