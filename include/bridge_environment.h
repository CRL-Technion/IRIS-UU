#ifndef BRIDGE_ENVIRONMENT_H
#define BRIDGE_ENVIRONMENT_H

#include <set>

#include "global_common.h"
#include "nigh/lp_space.hpp"
#include "nigh/kdtree_batch.hpp"
#include <unordered_map>
#include <cstddef>
#include <memory>
#include <limits>

namespace drone
{

    namespace nigh = unc::robotics::nigh;

    // const String kBridgeStructureFilename = "../data/bridge/bridgeTestLarge.obj";
    // const String kBridgeStructureFilename = "../data/bridge/bridge_small.obj";
    // const String kBridgeStructureFilename = "../data/bridge/simpleExample.obj";
    // const String kBridgeStructureFilename = "../data/bridge/bridge.obj";

#if ToyProblem
    const String kBridgeStructureFilename = "../data/bridge/simpleExample.obj";
#else
    // const String kBridgeStructureFilename = "../data/bridge/bridge_small.obj";
    const String kBridgeStructureFilename = "../data/bridge/bridge.obj";
    // const String kBridgeStructureFilename = "../data/bridge/High_Bridge.obj";
#endif

    struct InspectPoint
    {
        SizeType idx;
        Vec3 point;
        InspectPoint(const SizeType i, const Vec3 &p)
            : idx(i), point(p)
        {
        }
    };

    struct InspectPointKey
    {
        const Vec3 &operator()(const InspectPoint &node) const
        {
            return node.point;
        }
    };

    using NodePair = std::pair<InspectPoint, RealNum>;
    struct Cmp
    {
        bool operator()(const NodePair n1, const NodePair n2) const
        {
            // smaller key comes first
            return n1.second < n2.second;
        }
    };
    using NodeSet = std::set<NodePair, Cmp>;

    // #include <Eigen/Core>
    // struct CacheKey
    // {
    //     std::pair<Vec3, Vec3> arg1;

    //     // Implement comparison operator for use in unordered_map
    //     bool operator==(const CacheKey &other) const
    //     {
    //         std::cout << "tsrr" << std::endl;
    //         return (arg1.first - other.arg1.first).norm() <= 5 &&
    //                (arg1.second - other.arg1.second).norm() <= 1;
    //     }
    // };
    // struct PairHash
    // {
    //     std::size_t operator()(const CacheKey &p) const
    //     {
    //         std::size_t seed = 0;
    //         for (int i = 0; i < 3; ++i)
    //         {
    //             seed ^= std::hash<RealNum>()(p.arg1.first(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    //             seed ^= std::hash<RealNum>()(p.arg1.second(i)) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    //         }
    //         return seed;
    //     }
    // };
    // struct Vec3Hash
    // {
    //     std::size_t operator()(const Vec3 &v) const
    //     {
    //         // Compute hash using std::hash for floats
    //         std::size_t h = 0;
    //         for (int i = 0; i < 3; ++i)
    //         {
    //             h = std::hash<float>{}(v(i));
    //         }
    //         return h;
    //     }
    // };
    // struct CacheKeyHash
    // {
    //     std::size_t operator()(const CacheKey &key) const
    //     {
    //         std::size_t h1 = Vec3Hash{}(key.arg1.first);
    //         std::size_t h2 = Vec3Hash{}(key.arg1.second);
    //         return h1 ^ (h2 << 1);
    //     }
    // };

    class AABB
    {
    public:
        Vec3 min, max;

        AABB()
        {
            min = Vec3(std::numeric_limits<RealNum>::max(), std::numeric_limits<RealNum>::max(), std::numeric_limits<RealNum>::max());
            max = Vec3(-std::numeric_limits<RealNum>::max(), -std::numeric_limits<RealNum>::max(), -std::numeric_limits<RealNum>::max());
        }

        AABB(const Vec3 &v)
        {
            min = v;
            max = v;
        }

        void extend(const Vec3 &v)
        {
            min[0] = std::min(min[0], v[0]);
            min[1] = std::min(min[1], v[1]);
            min[2] = std::min(min[2], v[2]);
            max[0] = std::max(max[0], v[0]);
            max[1] = std::max(max[1], v[1]);
            max[2] = std::max(max[2], v[2]);
        }

        void extend(const AABB &other)
        {
            min[0] = std::min(min[0], other.min[0]);
            min[1] = std::min(min[1], other.min[1]);
            min[2] = std::min(min[2], other.min[2]);
            max[0] = std::max(max[0], other.max[0]);
            max[1] = std::max(max[1], other.max[1]);
            max[2] = std::max(max[2], other.max[2]);
        }

        Vec3 center() const
        {
            return (min + max) * 0.5;
        }

        float surfaceArea() const
        {
            Vec3 d = max - min;
            return 2 * (d[0] * d[1] + d[0] * d[2] + d[1] * d[2]);
        }

        bool is_inside(const Vec3 &origin, const Vec3 &target) const
        {

            auto direction = (target - origin).normalized();
            auto diffVector = (target - origin);

      
            if ((target[0] >= min[0]) && (target[0] <= max[0]) &&
                (target[1] >= min[1]) && (target[1] <= max[1]) &&
                (target[2] >= min[2]) && (target[2] <= max[2]))
            {
                // std::cout << "IsPointInsideBox \n"<< std::endl;
                // getchar();
                return true;
            }

            if ((origin[0] >= min[0]) && (origin[0] <= max[0]) &&
                (origin[1] >= min[1]) && (origin[1] <= max[1]) &&
                (origin[2] >= min[2]) && (origin[2] <= max[2]))
            {
                // std::cout << "IsPointInsideBox \n"<< std::endl;
                // getchar();
                return true;
            }

            // Check if the ray intersects the box
            float t_min = std::numeric_limits<float>::lowest();
            float t_max = std::numeric_limits<float>::max();

            for (int i = 0; i < 3; i++)
            {
                if (std::fabs(direction[i]) < std::numeric_limits<float>::epsilon())
                {
                    // The ray is parallel to the slab
                    if (origin[i] < min[i] || origin[i] > max[i])
                    {
                        return false;
                    }
                }
                else
                {
                    float t1 = (min[i] - origin[i]) / direction[i];
                    float t2 = (max[i] - origin[i]) / direction[i];

                    if (t1 > t2)
                    {
                        std::swap(t1, t2);
                    }

                    t_min = std::max(t_min, t1);
                    t_max = std::min(t_max, t2);

                    if (t_min > t_max)
                    {
                        return false;
                    }
                }
            }

            return true;

            // auto direction = (target - origin).normalized();
            // auto diffVector = (target - origin);

            // if ((target[0] >= min[0]) && (target[0] <= max[0]) &&
            //     (target[1] >= min[1]) && (target[1] <= max[1]) &&
            //     (target[2] >= min[2]) && (target[2] <= max[2]))
            // {
            //     // std::cout << "IsPointInsideBox \n"<< std::endl;
            //     // getchar();
            //     return true;
            // }

            // if ((origin[0] >= min[0]) && (origin[0] <= max[0]) &&
            //     (origin[1] >= min[1]) && (origin[1] <= max[1]) &&
            //     (origin[2] >= min[2]) && (origin[2] <= max[2]))
            // {
            //     // std::cout << "IsPointInsideBox \n"<< std::endl;
            //     // getchar();
            //     return true;
            // }

            // for (int i = 0; i < 3; i++)
            // {
            //     float t0 = (min[i] - origin[i]) / direction[i];
            //     float t1 = (max[i] - origin[i]) / direction[i];

            //     if (t0 > 0)
            //     {
            //         if ((diffVector[i]) < (t0))
            //         {
            //             return false;
            //         }
            //     }
            //     else
            //     {
            //         if ((diffVector[i]) < (t1))
            //         {
            //             return false;
            //         }
            //     }

            // }

            // return true;
        }
        //     bool intersect(const Vec3 &origin, const Vec3 &target, float t_near, float t_far) const
        //     {

        //         auto direction = (target-origin).normalized();
        //         for (int i = 0; i < 3; i++)
        //         {
        //             float inv_direction = 1.0f / direction[i];
        //             float t0 = (min[i] - origin[i]) * inv_direction;
        //             float t1 = (max[i] - origin[i]) * inv_direction;
        //             if (inv_direction < 0.0f)
        //             {
        //                 std::swap(t0, t1);
        //             }
        //             t_near = std::max(t_near, t0);
        //             t_far = std::min(t_far, t1);
        //             if (t_near > t_far)
        //             {
        //                 return false;
        //             }
        //             if (t_far < 0.0f)
        //             {
        //                 return false;
        //             }
        //         }
        //         return true;
        //     }
        // };
    };
    struct BVHNode
    {
        AABB box;
        struct BVHNode *left_child = nullptr;
        struct BVHNode *right_child = nullptr;
        std::vector<int> triangles;
        bool is_leaf() const
        {
            return left_child == nullptr && right_child == nullptr;
        }
    };

    class BridgeEnvironment
    {
    public:
        BridgeEnvironment(const Idx seed = 1);
        ~BridgeEnvironment() = default;

        void AddTargetPoint(const Vec3 &p);
        void AddObstacleOnlyPoint(const Vec3 &p);

        std::optional<NodePair> NearestTarget(const Vec3 &p) const;
        NodeSet NearestTargetsInSphere(const Vec3 &p, const RealNum r) const;
        NodeSet NearestObstaclesInShpere(const Vec3 &p, const RealNum r) const;
        bool IsCollisionFree(const Vec3 &p, const RealNum r) const;
        RealNum EnvironmentBoundary(const Idx dim, const bool request_min = true);

        SizeType NumTargets() const;
        std::vector<SizeType> GetVisiblePointIndices(const Vec3 &pos, const Vec3 &tang,
                                                     const RealNum fov_in_rad, const RealNum min_dof, const RealNum max_dof);
        bool IfCorrectDirection(const Vec3 &pos, const Vec3 &tang, const RealNum fov_in_rad,
                                const RealNum dist) const;
        std::vector<Vec3> IndicesToPoints(const std::vector<Idx> &indices) const;
        Idx countVisible;
        Idx countNotVisible;

    private:
        // std::unordered_map<std::pair<Vec3, Vec3>, std::vector<SizeType>, PairHash> cache;
        // std::unordered_map<CacheKey, std::vector<SizeType>, PairHash, std::equal_to<CacheKey>> cache;
        // std::unordered_map<CacheKey, std::vector<SizeType>> cache;
        // std::unordered_map<CacheKey, std::vector<SizeType>, CacheKeyHash, decltype(&CacheKey::operator==)> cache1;
        // std::unordered_map<CacheKey, std::vector<SizeType>, CacheKeyHash, decltype(&CacheKey::operator==)> cache1(nullptr , CacheKeyHash(), &CacheKey::operator==);
        // std::unordered_map<CacheKey, std::vector<SizeType>, CacheKeyHash, std::equal_to<CacheKey>> cache;
        // std::unordered_map<CacheKey, std::vector<SizeType>> cache;

        // std::vector<SizeType> getValue(const Vec3 &pos, const Vec3 &tang, bool &isValueInTheCache)
        // {
        //     auto it = cache.find(std::make_pair(pos, tang));
        //     if (it != cache.end())
        //     {
        //         isValueInTheCache = true;
        //         return it->second;
        //     }
        //     std::vector<SizeType> result(2);
        //     isValueInTheCache = false;
        //     return result;
        //     // std::vector<SizeType> result = calculateFunction(pos, tang);
        //     // cache[std::make_pair(pos, tang)] = result;
        //     // return result;
        // }

        SizeType global_idx_{0};
        SizeType obstacle_idx_{0};
        nigh::Nigh<InspectPoint, nigh::L2Space<RealNum, 3>, InspectPointKey, nigh::Concurrent, nigh::KDTreeBatch<>>
            nn_;
        nigh::Nigh<InspectPoint, nigh::L2Space<RealNum, 3>, InspectPointKey, nigh::Concurrent, nigh::KDTreeBatch<>>
            nn_obstacles_;
        std::vector<Vec3> raw_vertices_;
        std::vector<Vec3> vertices_;
        std::vector<SizeType> vertex_idx_;
        std::vector<IdxPoint> faces_;
        BVHNode *bvh_root;
        RealNum min_x_;
        RealNum max_x_;
        RealNum min_y_;
        RealNum max_y_;
        RealNum min_z_;
        RealNum max_z_;

        void InitializeTargets();
        void InitializeObstaclePointCloud(const RealNum unit_area = 0.15);
        RealNum TriangleArea(const RealNum a, const RealNum b, const RealNum c) const;
        RealNum RandomNum(const RealNum min, const RealNum max) const;
        bool RayTriangleIntersect(const Vec3 &org, const Vec3 &dir, const Vec3 &v0, const Vec3 &v1,
                                  const Vec3 &v2, RealNum *t, RealNum *u, RealNum *v) const;

        // struct CacheKey
        // {
        //     Vec3 pos;
        //     Vec3 tang;
        //     float epsilon; // tolerance value for float comparisons
        //     // Implement comparison operator for use in unordered_map
        //     bool operator==(const CacheKey &other) const
        //     {
        //         return (pos - other.pos).norm() <= epsilon &&
        //                (tang - other.tang).norm() <= epsilon;
        //     }
        // };

        // using CacheValue = std::vector<std::size_t>;
        // std::unordered_map<CacheKey, CacheValue> cacheVisibiltyCheck;

        /// BVH functions
        // Define the BVH node struct

        // Build the BVH
        BVHNode *buildBVH();

        // Recursively build the BVH
        BVHNode *recursiveBuildBVH(const std::vector<int> &sorted_indices, int start_index, int end_index);

        // Compute the bounding box for a group of triangles
        // AABB computeBoundingBox(const std::vector<int> &sorted_indices, int start_index, int end_index);

        // Perform ray-triangle intersection
        bool rayTriangleIntersectBVH(const Vec3 &pos, const Vec3 &tar, const BVHNode *node);

        // Trace a ray through the scene using the BVH
        bool traceRayBVH(const Vec3 &pos, const Vec3 &dir, const BVHNode *node, const std::vector<Vec3> &vertices, const std::vector<int> &indices, float *t, float *u, float *v);
    };
}

#endif // BRIDGE_ENVIRONMENT_H
