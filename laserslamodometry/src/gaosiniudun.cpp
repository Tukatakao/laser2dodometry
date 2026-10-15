#include <eigen3/Eigen/Dense>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

// 残差
double diandaoxian(double x, double y, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    Eigen::Vector2d a = p2 - p1;
    Eigen::Vector2d b = Eigen::Vector2d(x, y) - p1;
    return (a(0)*b(1)-a(1)*b(0)) / a.norm();
}
// 雅可比矩阵
Eigen::RowVector3d yakebi(double dtheta,const Eigen::Vector2d &raw_p, const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    Eigen::RowVector3d J;
    double A = p2(1) - p1(1);
    double B = p2(0) - p1(0);
    double norm = std::sqrt(A * A + B * B);
    double cos = std::cos(dtheta);
    double sin = std::sin(dtheta);

    if (norm < 1e-6)
    {
        J.setZero();
        return J;
    }
    // 残差 r = (B*(y-p1y) - A*(x-p1x)) / norm 的雅可比
    J(0) = -A / norm;
    J(1) =  B / norm;
    J(2) = ((B*cos + A*sin)*raw_p(0) + (A*cos - B*sin)*raw_p(1)) / norm;
    return J;
}





bool gaosiniudundiedai(const std::vector<Eigen::Vector2d>& ref_points,
                       const std::vector<Eigen::Vector2d>& curr_points,
                       Eigen::Vector3d& xi)
{   
    Eigen::Matrix3d zuizhongJ;
    Eigen::Vector3d zuizhongB;
    //参考帧构建
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ref_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto& p : ref_points)
    {
        ref_cloud->push_back(pcl::PointXYZ(p(0),p(1),0));   //把points里的二维点 塞进ref_cloud
    }
    kdtree.setInputCloud(ref_cloud);                           //把参考帧点云放进kdtree  
    



    //初始位姿猜测
    
    const int diedaicishu = 20;
    const double eps = 1e-4;


    //高斯牛顿循环
    for(int i = 0;i<diedaicishu;i++)
    {
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();

        double dx = xi(0);
        double dy = xi(1);
        double dtheta = xi(2);
        double cos_t = cos(dtheta);
        double sin_t = sin(dtheta);
        for(auto& raw_p : curr_points)
        {
            double x0 = raw_p(0);
            double y0 = raw_p(1);
            double x = x0 * cos_t - y0 * sin_t + dx;
            double y = x0 * sin_t + y0 * cos_t + dy;


            std::vector<int> idx(2);               //准备给kdtree寻找来的俩点放坐标
            std::vector<float> dist(2);            //准备给kdtree寻找来的俩点放距离平方      
            
            pcl::PointXYZ search_pt(x,y,0);        //要查询的点

            int ret = kdtree.nearestKSearch(search_pt,2,idx,dist);

            if(ret != 2) continue;

            Eigen::Vector2d pp1 = ref_points[idx[0]];
            Eigen::Vector2d pp2 = ref_points[idx[1]];              //把找来的点放进去
            

            //计算残差，雅可比
            double r = diandaoxian(x,y,pp1,pp2);
            Eigen::RowVector3d J = yakebi(dtheta,raw_p,pp1,pp2);


            //累加h和b
            H += J.transpose() * J;
            b += -J.transpose() * r;
        }
            //求解xi
            Eigen::Vector3d dxi = H.ldlt().solve(b);
            //累加
            xi += dxi;
 
            //收敛判断
            if(dxi.norm() < eps)
            {
                zuizhongJ = H;
                zuizhongB = b;
                break;
            }
            if(i == diedaicishu-1)
            {
                zuizhongJ = H;
                zuizhongB = b;
                break;
            }
    }

    
    return true;
}
 