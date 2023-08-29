#ifndef VOXEL_MAP_UTIL_H
#define VOXEL_MAP_UTIL_H

#include "common_lib.h"
#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <cstdio>
#include <string>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <random>

#define HASH_P 116101
#define MAX_N 10000000000

/*** Common Param ***/
static int plane_id = 0;
static int update_size_threshold;
static int max_points_size;
static int sigma_num;
static double planer_threshold;
static double voxel_size;
static double quater_length;

/*** Point to Plane Matching Structure ***/
typedef struct ptpl
{
    V3D point;
    V3D point_world;
    V3D omega;
    double omega_norm = 0;
    double dist = 0;
    M3D plane_cov;
    int main_direction = 0;
} ptpl;

/*** 3D Point with Covariance ***/
typedef struct pointWithCov
{
    V3D point;
    V3D point_world;
    Eigen::Matrix3d cov;
} pointWithCov;

/*** Plane Structure ***/
typedef struct Plane
{
    /*** Update Flag ***/
    bool is_plane = false;
    bool is_init = false;

    /*** Plane Param ***/
    int main_direction = 0; // 0:ax+by+z+d=0;  1:ax+y+bz+d=0;  2:x+ay+bz+d=0;
    M3D plane_cov;
    V3D n_vec;
    bool isRootPlane = true;
    int rgb[3] = {0, 0, 0};

    /*** Incremental Calculation Param ***/
    double xx = 0.0;
    double yy = 0.0;
    double zz = 0.0;
    double xy = 0.0;
    double xz = 0.0;
    double yz = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    V3D center = V3D::Zero();
    Eigen::Matrix3d covariance = M3D::Zero();
    int points_size = 0;

} Plane;
typedef std::shared_ptr<Plane> PlanePtr;
typedef const std::shared_ptr<Plane> PlaneConstPtr;

class VOXEL_LOC
{
public:
    int64_t x, y, z;

    VOXEL_LOC(int64_t vx, int64_t vy, int64_t vz)
        : x(vx), y(vy), z(vz) {}

    bool operator==(const VOXEL_LOC &other) const
    {
        return (x == other.x && y == other.y && z == other.z);
    }
};

// Hash value
namespace std
{
    template <>
    struct hash<VOXEL_LOC>
    {
        int64_t operator()(const VOXEL_LOC &s) const
        {
            using std::hash;
            using std::size_t;
            return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
        }
    };
} // namespace std

class UnionFindNode
{
public:
    std::vector<pointWithCov> temp_points_; // all points in an octo tree
    PlanePtr plane_ptr_;
    double voxel_center_[3]{}; // x, y, z
    int all_points_num_;
    int new_points_num_;

    bool init_node_;
    bool update_enable_;
    bool is_plane;
    int id;
    UnionFindNode *rootNode;

    UnionFindNode();

    /*** Finish ***/
    void InitPlane(const std::vector<pointWithCov> &points, const PlanePtr &plane, UnionFindNode *node) const;

    void InitUnionFindNode();

    void UpdatePlane(const pointWithCov &pv, VOXEL_LOC &position, std::unordered_map<VOXEL_LOC, UnionFindNode *> &feat_map);
};

void MapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);

void BuildVoxelMap(const std::vector<pointWithCov> &input_points, std::unordered_map<VOXEL_LOC, UnionFindNode *> &feat_map);

void UpdateVoxelMap(const std::vector<pointWithCov> &input_points, std::unordered_map<VOXEL_LOC, UnionFindNode *> &feat_map);

void BuildSingleResidual(const pointWithCov &pv, const UnionFindNode *currentNode,
                         bool &is_sucess, ptpl &single_ptpl);

void BuildResidualListOMP(const unordered_map<VOXEL_LOC, UnionFindNode *> &voxel_map,
                          const std::vector<pointWithCov> &pv_list,
                          std::vector<ptpl> &ptpl_list);

/*** Visualization Function ***/
void CalcVectQuaternion(const Plane &single_plane, geometry_msgs::Quaternion &q);

/*** Visualization Function ***/
void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub,
                    const std::string &plane_ns, const Plane &single_plane,
                    const float alpha, const V3D &rgb, int id);

/*** Visualization Function ***/
void pubVoxelMap(const std::unordered_map<VOXEL_LOC, UnionFindNode *> &voxel_map, const ros::Publisher &plane_map_pub);

void calcBodyCov(V3D &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

// void TransformLidar(const StatesGroup &state,
//                     const shared_ptr<ImuProcess> &p_imu,
//                     const PointCloudXYZI::Ptr &input_cloud,
//                     pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud) {
//     trans_cloud->clear();
//     for (size_t i = 0; i < input_cloud->size(); i++) {
//         pcl::PointXYZINormal p_c = input_cloud->points[i];
//         V3D p(p_c.x, p_c.y, p_c.z);
//         p = state.rot_end * p + state.pos_end;
//         pcl::PointXYZI pi;
//         pi.x = static_cast<float>(p(0));
//         pi.y = static_cast<float>(p(1));
//         pi.z = static_cast<float>(p(2));
//         pi.intensity = p_c.intensity;
//         trans_cloud->points.push_back(pi);
//     }
// }

#endif
