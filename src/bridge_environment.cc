#include <iostream>
#include <fstream>
#include <random>

#include "bridge_environment.h"
#include "io_utils.h"

namespace drone
{

    BridgeEnvironment::BridgeEnvironment(const Idx seed)
    {
        srand(seed);

        std::cout << "Loading obj model..." << std::endl;

        io::Vec3s pos;
        io::Vec2s texture;
        io::Vec3s normal;
        io::Indices v_i;
        io::Indices t_i;
        io::Indices n_i;

        io::LoadObjModel(kBridgeStructureFilename, pos, texture, normal, v_i, t_i, n_i);

        // std::cout << "p" << std::endl;

        for (const auto &p : pos)
        {
            raw_vertices_.emplace_back(p[0], p[1], p[2]);
            // std::cout << "vertices" << p[0] << " " << p[1] << " " << p[2] << std::endl;
        }

        // std::cout << "idx" << std::endl;
        for (const auto i : v_i)
        {
            vertex_idx_.emplace_back(i);
            // std::cout << "faces_1 " <<  i << std::endl;
        }

        InitializeTargets();
        // std::cout << "test"  << std::endl;

        InitializeObstaclePointCloud();
        std::cout << "obstacle_idx_" << obstacle_idx_ << std::endl;
        bvh_root = buildBVH();

        // io::WriteJSPtCloud("../data/test.js", obstacles_, 0.5, IdxPoint(255, 255, 255));
    }

    void BridgeEnvironment::AddTargetPoint(const Vec3 &p)
    {
        vertices_.push_back(p);
        nn_.insert(InspectPoint(global_idx_++, p));
        nn_obstacles_.insert(InspectPoint(obstacle_idx_++, p));

        if (global_idx_ == 1)
        {
            min_x_ = p[0];
            max_x_ = p[0];
            min_y_ = p[1];
            max_y_ = p[1];
            min_z_ = p[2];
            max_z_ = p[2];
        }
        else
        {
            min_x_ = fmin(min_x_, p[0]);
            max_x_ = fmax(max_x_, p[0]);
            min_y_ = fmin(min_y_, p[1]);
            max_y_ = fmax(max_y_, p[1]);
            min_z_ = fmin(min_z_, p[2]);
            max_z_ = fmax(max_z_, p[2]);
        }
    }

    void BridgeEnvironment::AddObstacleOnlyPoint(const Vec3 &p)
    {
        nn_obstacles_.insert(InspectPoint(obstacle_idx_++, p));
    }

    std::optional<NodePair> BridgeEnvironment::NearestTarget(const Vec3 &p) const
    {
        return nn_.nearest(p);
    }

    NodeSet BridgeEnvironment::NearestTargetsInSphere(const Vec3 &p, const RealNum r) const
    {
        std::vector<NodePair> nbh;
        nn_.nearest(nbh, p, MAX_COVERAGE_SIZE, r);

        NodeSet nodes(nbh.begin(), nbh.end());
        return nodes;
    }

    NodeSet BridgeEnvironment::NearestObstaclesInShpere(const Vec3 &p, const RealNum r) const
    {
        std::vector<NodePair> nbh;
        nn_obstacles_.nearest(nbh, p, MAX_COVERAGE_SIZE, r);

        NodeSet nodes(nbh.begin(), nbh.end());
        return nodes;
    }

    bool BridgeEnvironment::IsCollisionFree(const Vec3 &p, const RealNum r) const
    {
        std::vector<NodePair> nbh;
        nn_obstacles_.nearest(nbh, p, obstacle_idx_, r);

        return (nbh.size() == 0);
    }

    void BridgeEnvironment::InitializeTargets()
    {
#if ToyProblem
        {
            Vec3 v;
            Idx i = 0;
            Idx j = 0;
            RealNum offset = 0.2;
            v[0] = -12 - offset * 2;
            v[1] = 4.0;
            v[2] = 0;
            for (i = 0; i < MAX_COVERAGE_SIZE / 3.0; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    AddTargetPoint(v);
                    std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
                    v[0] += offset;
                }
                v[0] += 3 - offset * 3;
            }

            std::cout << "Number of targets: " << global_idx_ << std::endl;
            return;
        }
#else
        std::ofstream fout;
        fout.open("targetsPoints");

        if (!fout.is_open())
        {
            std::cerr << "targetsPoints file cannot be opened!" << std::endl;
            exit(1);
        }

        for (const auto &v : raw_vertices_)
        {
            // Avoid duplicating target points.
            const auto neartest = nn_.nearest(v)->first.point;

            if ((neartest - v).norm() < 5.5)
            {
                continue;
            }

            // std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
            fout << v[0] << " " << v[1] << " " << v[2] << std::endl;

            AddTargetPoint(v);
        }
        fout.close();
        std::cout << "targetsPoints saved!" << std::endl;
#endif
        std::cout << "Number of targets: " << global_idx_ << std::endl;
    }

    void BridgeEnvironment::InitializeObstaclePointCloud(const RealNum unit_area)
    {
        for (auto i = 0; i < vertex_idx_.size(); i += 3)
        {

            auto p0 = raw_vertices_[vertex_idx_[i]];
            auto p1 = raw_vertices_[vertex_idx_[i + 1]];
            auto p2 = raw_vertices_[vertex_idx_[i + 2]];

            faces_.emplace_back(vertex_idx_[i], vertex_idx_[i + 1], vertex_idx_[i + 2]);
            // std::cout << "faces_" << vertex_idx_[i] << " " << vertex_idx_[i+1] << " " << vertex_idx_[i+2] << std::endl;

            auto area = TriangleArea((p1 - p0).norm(), (p2 - p1).norm(), (p0 - p2).norm());

            // std::cout << "a: " << (p1 - p0).norm()<< "b: " << (p2 - p1).norm()<< "b: " << (p0 - p2).norm()<< "TriangleArea: " << area << std::endl;

            if (area < unit_area)
            {
                continue;
            }

            // Idx num_points = std::floor(area / unit_area) * std::floor(area / unit_area) / 2.0;
            Idx num_points = 2*std::floor(area / unit_area) ;
            // std::cout << "std::floor(area / unit_area): " << std::floor(area / unit_area)<< std::endl;

            for (auto j = 0; j < num_points; ++j)
            {
                auto alpha = RandomNum(0, 1);
                auto beta = RandomNum(0, 1 - alpha);
                auto gamma = 1 - alpha - beta;
                // std::cout << "gamma" << alpha << " " << beta << " " << gamma << std::endl;

                auto new_point = alpha * p0 + beta * p1 + gamma * p2;
                // std::cout << vertex_idx_.size()<< " " << num_points<< " "<< i << " " << j << std::endl;

                AddObstacleOnlyPoint(new_point);
            }
        }

        std::cout << "Number of obstacles: " << obstacle_idx_ << std::endl;
    }

    RealNum BridgeEnvironment::TriangleArea(const RealNum a, const RealNum b, const RealNum c) const
    {
        auto s = 0.5 * (a + b + c);
        return sqrt(s * (s - a) * (s - b) * (s - c));
    }

    RealNum BridgeEnvironment::RandomNum(const RealNum min, const RealNum max) const
    {
        return min + ((RealNum)rand() / RAND_MAX) * (max - min);
    }

    RealNum BridgeEnvironment::EnvironmentBoundary(const Idx dim, const bool request_min)
    {
        if (dim == 0)
        {
            if (request_min)
            {
                return min_x_;
            }

            return max_x_;
        }
        else if (dim == 1)
        {
            if (request_min)
            {
                return min_y_;
            }

            return max_y_;
        }

        if (request_min)
        {
            return min_z_;
        }

        return max_z_;
    }

    SizeType BridgeEnvironment::NumTargets() const
    {
        return global_idx_;
    }

    // std::vector<SizeType> BridgeEnvironment::GetVisiblePointIndices(const Vec3& pos, const Vec3& tang,
    //         const RealNum fov_in_rad, const RealNum min_dof, const RealNum max_dof) const {
    //     std::vector<SizeType> visible_points;

    //     // Compute points in valid range.
    //     auto large_set = NearestTargetsInSphere(pos, max_dof);
    //     auto small_set = NearestTargetsInSphere(pos, min_dof);

    //     for (auto& p : small_set) {
    //         large_set.erase(p);
    //     }

    //     auto obstacle_set = NearestObstaclesInShpere(pos, max_dof);

    //     std::vector<Vec3> rays;

    //     for (auto& node : obstacle_set) {
    //         auto p = node.first.point;
    //         auto camera_to_point = p - pos;
    //         auto angle = std::acos(camera_to_point.normalized().dot(tang.normalized()));

    //         if (angle > 0.5*fov_in_rad) {
    //             continue;
    //         }

    //         bool visible = true;

    //         for (auto& other : rays) {
    //             auto angle_diff = std::acos(camera_to_point.normalized().dot(other.normalized()));

    //             if (fabs(angle_diff) < 2.0/180.0 * M_PI) {
    //                 visible = false;
    //                 break;
    //             }
    //         }

    //         if (visible && node.first.idx < global_idx_) {
    //             visible_points.push_back(node.first.idx);
    //         }

    //         rays.push_back(camera_to_point);
    //     }

    //     return visible_points;
    // }

    std::vector<SizeType> BridgeEnvironment::GetVisiblePointIndices(const Vec3 &pos, const Vec3 &tang,
                                                                    const RealNum fov_in_rad, const RealNum min_dof, const RealNum max_dof)
    {
        std::vector<SizeType> visible_points;

        // std::pair<Vec3, Vec3> input_args = std::make_pair(pos, tang);
        // CacheKey cache_key = {input_args};
        //     // std::cout << (cache_key.arg1.first).norm() << std::endl;

        // auto cache_iter = cache1.find(cache_key);
        // if (cache_iter != cache1.end())
        // {
        //     std::cout << "use cache" << std::endl;
        //     // Cache hit: use the cached value
        //     return cache1[cache_key];
        // }

        // CacheValue addMemberToCache(float arg1, float arg2, float arg3, float epsilon, int newMember)
        // {
        //     CacheKey key = {arg1, arg2, arg3, epsilon};
        //     auto it = cache.find(key);
        //     if (it != cache.end())
        //     {
        //         CacheValue cachedResult = it->second;
        //         // Modify the cached result to include the new member
        //         cachedResult.members.push_back(newMember);
        //         cache[key] = cachedResult;
        //         return cachedResult;
        //     }
        //     CacheValue result = // Calculate the result of the function and add the new member
        //         result.members.push_back(newMember);
        //     cache[key] = result;
        //     return result;
        // }

        // Compute points in valid range.
        auto large_set = NearestTargetsInSphere(pos, max_dof);
        auto small_set = NearestTargetsInSphere(pos, min_dof);

        for (const auto &p : small_set)
        {
            large_set.erase(p);
        }

        // Ray shooting to every point
        for (const auto &node : large_set)
        {
            const auto p = node.first.point;
            const auto camera_to_point = (p - pos);
            const auto camera_to_point_normalized = camera_to_point.normalized();
            const auto angle = std::acos(camera_to_point_normalized.dot(tang.normalized()));

            if (angle > 0.5 * fov_in_rad)
            {
                continue;
            }

            bool visible = true;
            const auto dist = camera_to_point.norm();
            RealNum t, u, v;
            Idx count = 0;

            if (rayTriangleIntersectBVH(pos, p, bvh_root))
            {
                visible = false;
            }

            // for (const auto &f : faces_)
            // {
            //     const auto &v0 = raw_vertices_[f[0]];
            //     const auto &v1 = raw_vertices_[f[1]];
            //     const auto &v2 = raw_vertices_[f[2]];

            //     if (RayTriangleIntersect(pos, camera_to_point_normalized, v0, v1, v2, &t, &u, &v) && t < dist - EPS)
            //     {
            //         visible = false;
            //         break;
            //     }
            // }

            if (visible && node.first.idx < global_idx_)
            {
                visible_points.push_back(node.first.idx);
            }
        }

        // cache[cache_key] = visible_points;
        // cache1.insert(std::make_pair(cache_key,visible_points));
        return visible_points;
    }

    bool BridgeEnvironment::RayTriangleIntersect(const Vec3 &org, const Vec3 &dir, const Vec3 &v0,
                                                 const Vec3 &v1, const Vec3 &v2, RealNum *t, RealNum *u, RealNum *v) const
    {
        const auto v0v1 = v1 - v0;
        const auto v0v2 = v2 - v0;
        const auto pvec = dir.cross(v0v2);
        RealNum det = v0v1.dot(pvec);

        if (std::fabs(det) < EPS)
        {
            return false;
        }

        RealNum inv_det = 1.0 / det;

        const auto tvec = org - v0;
        *u = tvec.dot(pvec) * inv_det;

        if (*u < 0 || *u > 1)
        {
            return false;
        }

        const auto qvec = tvec.cross(v0v1);
        *v = dir.dot(qvec) * inv_det;

        if (*v < 0 || *u + *v > 1)
        {
            return false;
        }

        *t = v0v2.dot(qvec) * inv_det;
        return true;
    }

    bool BridgeEnvironment::IfCorrectDirection(const Vec3 &pos, const Vec3 &tang,
                                               const RealNum fov_in_rad, const RealNum dist) const
    {
        auto obstacle_set = NearestObstaclesInShpere(pos, dist);

        for (auto &node : obstacle_set)
        {
            auto p = node.first.point;
            auto camera_to_point = p - pos;
            auto angle = std::acos(camera_to_point.normalized().dot(tang.normalized()));

            if (angle < 0.5 * fov_in_rad && node.first.idx < global_idx_)
            {
                return true;
            }
        }

        return false;
    }

    std::vector<Vec3> BridgeEnvironment::IndicesToPoints(const std::vector<Idx> &indices) const
    {
        std::vector<Vec3> points;

        for (auto &i : indices)
        {
            points.push_back(vertices_[i]);
        }

        return points;
    }

    BVHNode *BridgeEnvironment::buildBVH()
    {
        std::vector<RealNum> triangle_centers;
        triangle_centers.reserve(faces_.size());
        for (size_t i = 0; i < faces_.size(); i++)
        {

            const Vec3 &v1 = raw_vertices_[faces_[i][0]];
            const Vec3 &v2 = raw_vertices_[faces_[i][1]];
            const Vec3 &v3 = raw_vertices_[faces_[i][2]];
            Vec3 center = (v1 + v2 + v3) / 3.0;
            // triangle_centers.push_back(center.norm());
            triangle_centers.push_back(center[1]);
        }

        // Create a vector of indices corresponding to the sorted order of the triangles
        std::vector<int> sorted_indices(triangle_centers.size());
        std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
        std::sort(sorted_indices.begin(), sorted_indices.end(),
                  [&](const int &i1, const int &i2)
                  { return triangle_centers[i1] < triangle_centers[i2]; });

        return recursiveBuildBVH(sorted_indices, 0, sorted_indices.size());
    }

    BVHNode *BridgeEnvironment::recursiveBuildBVH(const std::vector<int> &sorted_indices, int start_index, int end_index)
    {
        BVHNode *node = new BVHNode();

        for (int i = start_index; i < end_index; i++)
        {
            int triangle_index = sorted_indices[i];
            const auto &v0 = raw_vertices_[faces_[triangle_index][0]];
            const auto &v1 = raw_vertices_[faces_[triangle_index][1]];
            const auto &v2 = raw_vertices_[faces_[triangle_index][2]];

            node->box.extend(v0);
            node->box.extend(v1);
            node->box.extend(v2);
            node->triangles.push_back(triangle_index);
        }
        if (end_index - start_index <= sqrt(sorted_indices.size()))
        {
            return node;
        }

        int best_split_index = (start_index + end_index) / 2;

        node->left_child = recursiveBuildBVH(sorted_indices, start_index, best_split_index);

        node->right_child = recursiveBuildBVH(sorted_indices, best_split_index, end_index);

        return node;
    }

    // AABB BridgeEnvironment::computeBoundingBox(const std::vector<int> &sorted_indices, int start_index, int end_index)
    // {
    //     AABB box;
    //     for (const auto &triangle_index : sorted_indices)
    //     {
    //         // int triangle_index = sorted_indices[i];
    //         const auto &v0 = raw_vertices_[faces_[triangle_index][0]];
    //         const auto &v1 = raw_vertices_[faces_[triangle_index][1]];
    //         const auto &v2 = raw_vertices_[faces_[triangle_index][2]];
    //         // const auto &v0 = raw_vertices_[vertex_idx_[triangle_index * 3]];
    //         // const auto &v1 = raw_vertices_[vertex_idx_[triangle_index * 3 + 1]];
    //         // const auto &v2 = raw_vertices_[vertex_idx_[triangle_index * 3 + 2]];
    //         box.extend(v0);
    //         box.extend(v1);
    //         box.extend(v2);
    //     }
    //     return box;
    // }

    bool BridgeEnvironment::rayTriangleIntersectBVH(const Vec3 &pos, const Vec3 &tar, const BVHNode *node)
    {
        if (!node->box.is_inside(pos, tar))
        {
            return false;
        }

        // If this node is a leaf, test for intersection with each triangle
        if (node->is_leaf())//node->left_child == nullptr && node->right_child == nullptr)
        {
            const auto dist = (tar - pos).norm();

            for (const auto &triangle_index : node->triangles)
            {
                const auto &v0 = raw_vertices_[faces_[triangle_index][0]];
                const auto &v1 = raw_vertices_[faces_[triangle_index][1]];
                const auto &v2 = raw_vertices_[faces_[triangle_index][2]];

                RealNum t, u, v;
                if (RayTriangleIntersect(pos, (tar - pos).normalized(), v0, v1, v2, &t, &u, &v) && t < (dist - EPS))
                {
                    return true;
                }
            }

            return false;
        }

        // Otherwise, recursively test for intersection with child nodes
        if (rayTriangleIntersectBVH(pos, tar, node->left_child))
        {
            return true;
        }

        return rayTriangleIntersectBVH(pos, tar, node->right_child);
    }
}
