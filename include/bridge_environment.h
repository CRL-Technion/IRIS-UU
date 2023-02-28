#ifndef BRIDGE_ENVIRONMENT_H
#define BRIDGE_ENVIRONMENT_H

#include <set>

#include "global_common.h"
#include "nigh/lp_space.hpp"
#include "nigh/kdtree_batch.hpp"
#include <unordered_map>
#include <cstddef>
#include <memory>
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
    const String kBridgeStructureFilename = "../data/bridge/bridge_small.obj";
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
    };

}

#endif // BRIDGE_ENVIRONMENT_H
