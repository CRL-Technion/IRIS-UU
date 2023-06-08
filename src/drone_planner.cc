#include "drone_planner.h"

extern bool extern_validate_sample;

namespace drone
{

    DronePlanner::DronePlanner(const RobotPtr &robot, const EnvPtr &env, const Idx seed)
        : robot_(robot), env_(env), seed_(seed)
    {
        rng_.seed(seed_);
        uni_ = RealUniformDist(0, 1);

#if REJECT_SAMPLING
        global_vis_set_.Clear();
#endif

        // ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
    }

    void DronePlanner::SampleStartConfig(const Idx max_iter, const Idx seed)
    {
        Rand rng;
        rng.seed(seed);
        RealUniformDist uni(0, 1);

        Vec3 pos;
        RealNum yaw, camera_angle;

        for (auto i = 0; i < max_iter; ++i)
        {
            for (auto j = 0; j < 3; ++j)
            {
                auto lo = (j == 2) ? env_->EnvironmentBoundary(j) : env_->EnvironmentBoundary(j) - validation_distance_;
                auto hi = env_->EnvironmentBoundary(j, false) + validation_distance_;
                // if (j == 0)
                // {
                //     lo = -8;
                //     hi = -6;
                // }

                // if (j == 1)
                // {
                //     lo = 1;
                //     hi = 2;
                // }

                pos[j] = uni(rng) * (hi - lo) + lo;
                // if (j == 2)
                // {
                //     pos[j] = 0.0;
                // }
            }

            yaw = uni(rng) * (kMaxYaw - kMinYaw) + kMinYaw;
            camera_angle = uni(rng) * (kMaxCameraAngle - kMinCameraAngle) + kMinCameraAngle;

            // std::cout << "pos:" << std::endl;
            // std::cout << pos << std::endl;
#if ToyProblem
            pos[0] = 12;
            pos[1] = 0;
            pos[2] = 0;

            yaw = M_PI / 2.0;
            camera_angle = 0;
            // pos[0] = -102;
            // pos[1] = -11;
            // yaw = 0.0;
            // camera_angle = 0;
#endif

            robot_->SetConfig(pos, yaw, camera_angle);
            robot_->ComputeShape();

            if (env_->IsCollisionFree(robot_->Config()->Position(), robot_->SphereRadius()) && env_->IfCorrectDirection(robot_->CameraPos(), robot_->CameraTangent(), robot_->FOV(),
                                                                                                                        validation_distance_))
            {
                robot_->SaveStartConfig();
                std::cout << "Found at " << i << std::endl;
                std::cout << "Start config: ";
                robot_->Config()->Print(std::cout);
                return;
            }
            robot_->SaveStartConfig();
            std::cout << "Found at " << i << std::endl;
            std::cout << "Start config: ";
            robot_->Config()->Print(std::cout);
            return;
        }

        std::cout << "Fail to find valid start config." << std::endl;
        exit(1);
    }

    void DronePlanner::SetParams(const RealNum step_size, const bool if_k_nearest)
    {
        step_size_ = step_size;
        k_nearest_ = if_k_nearest;
    }

    ob::SpaceInformationPtr DronePlanner::Define_space_info_()
    {
        auto state_space_ = ob::StateSpacePtr(new DroneStateSpace());
        ob::RealVectorBounds bounds(5);

        for (auto i = 0; i < 3; ++i)
        {
            if (i < 2)
            {
                bounds.setLow(i, env_->EnvironmentBoundary(i) - validation_distance_);
            }
            else
            {
                bounds.setLow(i, env_->EnvironmentBoundary(i));
            }

            bounds.setHigh(i, env_->EnvironmentBoundary(i, false) + validation_distance_);
        }

        bounds.setLow(3, kMinYaw);
        bounds.setHigh(3, kMaxYaw);
        bounds.setLow(4, kMinCameraAngle);
        bounds.setHigh(4, kMaxCameraAngle);
        state_space_->as<DroneStateSpace>()->setBounds(bounds);
        space_info_.reset(new ob::SpaceInformation(state_space_));
        using namespace std::placeholders;
        space_info_->setStateValidityChecker(std::bind(&DronePlanner::StateValid, this, _1));
        space_info_->setStateValidityCheckingResolution(validity_res_);
        space_info_->setup();
        return space_info_;
    }

    void DronePlanner::BuildAndSaveInspectionGraph(const String file_name, const Idx target_size)
    {
        std::cout << "Prepare to build an inspection graph of size: " << target_size << std::endl;

        // State space.
        ompl::RNG::setSeed(seed_);
        num_targets_ = env_->NumTargets();
        auto state_space_ = ob::StateSpacePtr(new DroneStateSpace());

        ob::RealVectorBounds bounds(5);

        for (auto i = 0; i < 3; ++i)
        {
            if (i < 2)
            {
                bounds.setLow(i, env_->EnvironmentBoundary(i) - validation_distance_);
            }
            else
            {
                bounds.setLow(i, env_->EnvironmentBoundary(i));
            }

            bounds.setHigh(i, env_->EnvironmentBoundary(i, false) + validation_distance_);
        }

        bounds.setLow(3, kMinYaw);
        bounds.setHigh(3, kMaxYaw);
        bounds.setLow(4, kMinCameraAngle);
        bounds.setHigh(4, kMaxCameraAngle);
        state_space_->as<DroneStateSpace>()->setBounds(bounds);

        // Space info.
        space_info_.reset(new ob::SpaceInformation(state_space_));
        using namespace std::placeholders;
        space_info_->setStateValidityChecker(std::bind(&DronePlanner::StateValid, this, _1));
        space_info_->setStateValidityCheckingResolution(validity_res_);
        space_info_->setup();

        // Problem definition.
        auto problem_def = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(space_info_));
        ob::State *start = space_info_->allocState();
        state_space_->as<DroneStateSpace>()->StateFromConfiguration(start, robot_->StartConfig());

        problem_def->addStartState(start);
        auto goal = ob::GoalPtr(new InfiniteGoal(space_info_));
        problem_def->setGoal(goal);
        auto obj = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(space_info_));
        problem_def->setOptimizationObjective(obj);

        // Planner.
        auto planner = ob::PlannerPtr(new og::RRG(space_info_));
        planner->as<og::RRG>()->setRange(10 * step_size_);
        planner->as<og::RRG>()->setGoalBias(0.0);
        planner->as<og::RRG>()->setKNearest(k_nearest_);
        planner->setProblemDefinition(problem_def);
        planner->setup();

        // Build graph incrementally.
        Inspection::Graph *graph = new Inspection::Graph();

        ob::PlannerData tree_data(space_info_);
        ob::PlannerData graph_data(space_info_);

#if ToyProblem
        InsertGraphPointToyProblem(graph);
        // InsertGraphPointOutSideToyProblem(graph);
        // InsertIntermediatePointsToGraph(graph);
        // MarkEdgeRiskZone(graph);
        std::cout << "Number of vertices : " << graph->NumVertices() << "\n";
        std::cout << "Number of edges : " << graph->NumEdges() << "\n";
        std::cout << "Covered targets: " << graph->NumTargetsCovered()
                  << ", " << graph->NumTargetsCovered() * (RealNum)100 / num_targets_ << "%" << std::endl;

#else
        // InsertGraphPointToyProblem(graph);
        for (Idx j = 0; j < MAX_COVERAGE_SIZE; j++)
        {
            global_vis_set_.bitset_[j] = 0.0;
        }
        while (graph->NumVertices() < target_size)
        {
            BuildRRGIncrementally(graph, planner, tree_data, graph_data);
            std::cout << "Covered targets: " << graph->NumTargetsCovered()
                      << ", " << graph->NumTargetsCovered() * (RealNum)100 / num_targets_ << "%" << std::endl;
        }
        std::cout << "Number of vertices : " << graph->NumVertices() << "\n";
        std::cout << "Number of edges : " << graph->NumEdges() << "\n";
        std::cout << "Covered targets: " << graph->NumTargetsCovered()
                  << ", " << graph->NumTargetsCovered() * (RealNum)100 / num_targets_ << "%" << std::endl;

        std::ofstream fout;
        fout.open("targetsBuildIndex");

        if (!fout.is_open())
        {
            std::cerr << "targetsBuildIndex file cannot be opened!" << std::endl;
            exit(1);
        }
        for (const auto &v : graph->GlobalVisibility().bitset_)
        {
            fout << v << " " << std::endl;
        }

        // Vec3 pos;
        // pos[0] = 0;
        // pos[1] = 0;
        // pos[2] = 0;
        // auto nodetemp = env_->NearestTargetsInSphere(pos, 10000000000000);
        // for (const auto &node : nodetemp)
        // {
        //     const auto p = node.first.point;
        //     const auto idx = node.first.idx;
        //     std::cout << "node.first.idx " << idx << std::endl;
        //     std::cout << "node.first.p " <<p[0] << " " << p[1] << " " << p[2]  << std::endl;

        // }
        fout.close();
        std::cout << "targetsBuildIndex saved!" << std::endl;
// InsertIntermediatePointsToGraph(graph);
// MarkEdgeRiskZone(graph);
#endif

        graph->Save(file_name, true);

        delete graph;
    }
    void DronePlanner::InsertGraphPointToyProblem(Inspection::Graph *graph)
    {
        Vec3 pos;
        pos[0] = 12; // robot_->Config()->Position()[0];
        pos[1] = 0;
        pos[2] = 0;
        auto cameraAngle = 0;
        auto yaw = M_PI / 2;
        Idx i = 0;
        Idx j = 0;
        // add vertices Under the bridge
        auto numberOfVertices = 100;
        for (i = 0; i < numberOfVertices; i++)
        {
            yaw = M_PI / 2;
            // if (i == 0)
            // {
            //     pos[0] = 20; // robot_->Config()->Position()[0];
            //     pos[1] = 0;
            //     pos[2] = 0;
            //     yaw = -M_PI / 2;
            // }
            if (i == 0)
            {
                // std::cout << "i == 0" << std::endl;
                pos[0] = 12; // robot_->Config()->Position()[0];
                pos[1] = 0;
                pos[2] = 0;
            }

            if (pos[0] < -13 && i < 10)
            {
                // std::cout << "i == 1" << std::endl;
                pos[0] = 12.0;
                pos[1] = -2;
            }
            if (pos[0] < -13 && i < 19)
            {
                // std::cout << "i == 2" << std::endl;
                // break;
                pos[0] = 12.0;
                pos[1] = -4;
            }

            if (pos[0] < -13 && i < 28)
            {
                // std::cout << "i == 3" << std::endl;
                break;
                pos[0] = 12.0;
                pos[1] = -4;
            }
            if (pos[0] < -13 && i < 36)
            {
                // break;
                pos[0] = 12.0;
                pos[1] = -6;
            }
            if (pos[0] < -13 && i < 45)
            {
                // break;
                pos[0] = 12.0;
                pos[1] = -6.5;
            }
            if (pos[0] < -13 && i < 100)
            {
                break;
            }
            auto numVertices = graph->NumVertices();
            graph->AddVertex(numVertices);
            auto vertex = graph->Vertex(numVertices);
            vertex->state = space_info_->allocState();

            // space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
            vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
            std::cout << "pos: " << i << " " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;

            vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(cameraAngle);
            vertex->state->as<DroneStateSpace::StateType>()->SetYaw(yaw);

            const TimePoint start = Clock::now();
            ComputeVisibilitySet(vertex);
            vertex->time_vis = RelativeTime(start);
            vertex->time_build = 0; // todo calculate time_build of IntermediatePoint

            graph->UpdateGlobalVisibility(vertex->vis);
            pos[0] -= 3;
        }
        std::cout << "graph->NumVertices(): " << graph->NumVertices() << std::endl;
        const int groupSize = 9; // group size
        const int numGroups = graph->NumVertices() / groupSize;

        for (i = 0; i < graph->NumVertices(); i++)
        {
            for (j = 0; j < graph->NumVertices(); j++)
            {
                if (i >= j)
                {
                    continue;
                }
                bool tempGroups = (i / groupSize == j / groupSize);
                // std::cout << "i j: " << i << " " << j << " " << tempGroups << std::endl;
                const ob::State *source = graph->Vertex(i)->state;
                const ob::State *target = graph->Vertex(j)->state;
                Inspection::EPtr edge1(new Inspection::Edge(i, j));
                edge1->cost = space_info_->distance(source, target);
                // std::cout << "edge1->cost : " << edge1->cost << tempGroups << std::endl;

                if (tempGroups && abs(i - j) > 1.5)
                {
                    continue;
                }
                else
                {

                    if (edge1->cost > 3.7)
                    {
                        continue;
                    }
                }

                if (abs(i / groupSize - j / groupSize) > 1)
                {
                    continue;
                }

                // std::cout << "i j: " << i << " " << j << std::endl;

                // if (validate_all_edges_)
                // {
                const TimePoint start = Clock::now();
                if (!CheckEdge(source, target))
                {
                    continue;
                }
                bool valid = CheckEdge(source, target);
                edge1->checked = true;
                edge1->valid = valid;

                edge1->time_forward_kinematics = RelativeTime(start);
                // }

                graph->AddEdge(edge1);
            }
        }
        std::cout << "debug: " << graph->NumVertices() << std::endl;

        // Vec3 pos;
        // pos[0] = -100; // robot_->Config()->Position()[0];
        // pos[1] = robot_->Config()->Position()[1];
        // pos[2] = robot_->Config()->Position()[2];
        // auto cameraAngle = robot_->Config()->CameraAngle();
        // auto yaw = robot_->Config()->Yaw();
        // Idx i = 0;
        // // add vertices Under the bridge
        // auto numberOfVerticesUnderTheBridge = 50;
        // for (i = 0; i < numberOfVerticesUnderTheBridge; i++)
        // {

        //     auto numVertices = graph->NumVertices();
        //     graph->AddVertex(numVertices);
        //     auto vertex = graph->Vertex(numVertices);
        //     vertex->state = space_info_->allocState();

        //     // space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
        //     vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
        //     // std::cout << "pos: " << pos[0] << "pos: " << pos[1]<< "pos: " << pos[2] << std::endl;

        //     vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(cameraAngle);
        //     vertex->state->as<DroneStateSpace::StateType>()->SetYaw(yaw);

        //     const TimePoint start = Clock::now();
        //     ComputeVisibilitySet(vertex);
        //     vertex->time_vis = RelativeTime(start);
        //     vertex->time_build = 0; // todo calculate time_build of IntermediatePoint

        //     graph->UpdateGlobalVisibility(vertex->vis);
        //     pos[0] += (UpperBordersXYZ[0] - LowerBordersXYZ[0]) / numberOfVerticesUnderTheBridge;
        //     // if (pos[0]> -53 && pos[0] <-7)
        //     // {
        //     //     pos[1] = -10;
        //     // }
        //     // else{
        //     //     pos[1] = -18;
        //     // }
        //     // std::cout << "InsertGraphPointToyProblem: " << i << std::endl;
        // }
        // std::cout << "graph->NumVertices(): " << graph->NumVertices() << std::endl;
        // auto numVerticesUnderTheBridge = graph->NumVertices();
        // for (i = 0; i < graph->NumVertices() - 1; i++)
        // {
        //     auto numEdges = graph->NumEdges();
        //     const ob::State *source = graph->Vertex(i)->state;
        //     const ob::State *target = graph->Vertex(i + 1)->state;
        //     Inspection::EPtr edge1(new Inspection::Edge(i, i + 1));
        //     edge1->cost = space_info_->distance(source, target);
        //     if (validate_all_edges_)
        //     {
        //         const TimePoint start = Clock::now();
        //         bool valid = this->CheckEdge(source,target);
        //         edge1->checked = true;
        //         edge1->valid = valid;
        //         edge1->time_forward_kinematics = RelativeTime(start);
        //     }

        //     graph->AddEdge(edge1);
        // }
    }

    Inspection::EPtr DronePlanner::InsertIntermediatePointToGraph(Inspection::Graph *graph, Inspection::EPtr edge, Vec3 InsertIntermediatePosition)
    {
        auto sourceIndex = edge->source;
        auto targetIndex = edge->target;
        edge->valid = false;
        auto numVertices = graph->NumVertices();
        auto numEdges = graph->NumEdges();

        graph->AddVertex(numVertices);
        auto vertex = graph->Vertex(numVertices);
        vertex->state = space_info_->allocState();
        // std::cout << "InsertIntermediatePosition " << InsertIntermediatePosition << std::endl;
        // space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
        vertex->state->as<DroneStateSpace::StateType>()->SetPosition(InsertIntermediatePosition);

        auto SumCameraAngle = (graph->Vertex(sourceIndex)->state->as<DroneStateSpace::StateType>()->CameraAngle() + graph->Vertex(targetIndex)->state->as<DroneStateSpace::StateType>()->CameraAngle());
        auto SumYaw = (graph->Vertex(sourceIndex)->state->as<DroneStateSpace::StateType>()->Yaw() + graph->Vertex(targetIndex)->state->as<DroneStateSpace::StateType>()->Yaw());
        vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(SumCameraAngle / 2.0);
        vertex->state->as<DroneStateSpace::StateType>()->SetYaw(SumYaw / 2.0);

        const TimePoint start = Clock::now();
        // ComputeVisibilitySet(vertex);
        vertex->time_vis = RelativeTime(start);
        vertex->time_build = 0; // todo calculate time_build of IntermediatePoint

        // graph->UpdateGlobalVisibility(vertex->vis);

        const ob::State *source = graph->Vertex(sourceIndex)->state;
        const ob::State *intermediate = graph->Vertex(numVertices)->state;
        const ob::State *target = graph->Vertex(targetIndex)->state;

        // add the first edge
        Inspection::EPtr edge1(new Inspection::Edge(sourceIndex, numVertices));
        edge1->cost = space_info_->distance(source, intermediate);

        if (validate_all_edges_)
        {
            const TimePoint start = Clock::now();
            // bool valid = this->CheckEdge(source,intermediate);
            edge->checked = true;
            edge1->valid = true;
            edge1->time_forward_kinematics = RelativeTime(start);
        }

        graph->AddEdge(edge1);

        // add the second edge
        Inspection::EPtr edge2(new Inspection::Edge(numVertices, targetIndex));
        edge2->cost = space_info_->distance(intermediate, target);

        if (validate_all_edges_)
        {
            const TimePoint start = Clock::now();
            // bool valid = this->CheckEdge(intermediate,target);
            edge->checked = true;
            edge2->valid = true;
            edge2->time_forward_kinematics = RelativeTime(start);
        }

        graph->AddEdge(edge2);
        return edge2;
    }

    // bool DronePlanner::FindInsertPointRiskZone(const Vec3 &s, const Vec3 &t, Vec3 &intersection)
    // {
    //     double dir[3];
    //     for (int i = 0; i < 3; i++)
    //     {
    //         dir[i] = t[i] - s[i];
    //     }

    //     double tmin = -INFINITY, tmax = INFINITY;
    //     for (int i = 0; i < 3; i++)
    //     {
    //         double t1 = (LowerBordersXYZ[i] - s[i]) / dir[i];
    //         double t2 = (UpperBordersXYZ[i] - s[i]) / dir[i];
    //         if (dir[i] > 0)
    //         {
    //             tmin = std::max(tmin, std::min(t1, t2));
    //             tmax = std::min(tmax, std::max(t1, t2));
    //         }
    //         else if (dir[i] < 0)
    //         {
    //             tmin = std::max(tmin, std::min(t1, t2));
    //             tmax = std::min(tmax, std::max(t1, t2));
    //         }
    //         else
    //         {
    //             // the ray is parallel to the plane of the box
    //             if (s[i] < LowerBordersXYZ[i] || s[i] > UpperBordersXYZ[i])
    //             {
    //                 // the ray is outside the box
    //                 return false;
    //             }
    //         }
    //     }

    //     if (tmax < tmin)
    //     {
    //         // the ray is pointing away from the box
    //         return false;
    //     }

    //     // intersection point
    //     for (int i = 0; i < 3; i++)
    //     {
    //         intersection[i] = s[i] + tmin * dir[i];
    //     }

    //     // check if the intersection point lies within the line segment
    //     for (int i = 0; i < 3; i++)
    //     {
    //         double lambda = (intersection[i] - s[i]) / dir[i];
    //         if (lambda < 0 || lambda > 1)
    //         {
    //             // the intersection point is outside the line segment
    //             return false;
    //         }
    //     }

    //     return true;
    // }

    // bool DronePlanner::FindInsertPointRiskZone(const Vec3 &source, const Vec3 &target, Vec3 &insertPoint)
    // {
    //     Vec3 insertPointTemp;
    //     Vec3 direction = (target - source);
    //     auto normVec = direction.norm();
    //     direction.normalize();
    //     std::vector<double> t;
    //     Idx i = 0;
    //     Idx counterDirection = 0;
    //     for (i = 0; i < 3; i++)
    //     {
    //         if (abs(direction[i] < 1e-4))
    //         {
    //             counterDirection++;

    //             if (counterDirection > 2)
    //             {
    //                 insertPoint = source;
    //                 return false;
    //             }
    //             continue;
    //         }

    //         t.push_back((LowerBordersXYZ[i] - source[i]) / direction[i]);
    //         t.push_back((UpperBordersXYZ[i] - source[i]) / direction[i]);
    //     }
    //     sort(t.begin(), t.end());
    //     // std::cout << "------------" << std::endl;
    //     RealNum tmin = 1000000;
    //     bool found = false;
    //     for (i = 0; i < t.size(); i++)
    //     {

    //         // std::cout << "t[i]: " << t[i] << std::endl;
    //         if (t[i] > -normVec && t[i] < normVec)
    //         {
    //             insertPointTemp[0] = source[0] + (t[i] + 1e-2) * direction[0];
    //             insertPointTemp[1] = source[1] + (t[i] + 1e-2) * direction[1];
    //             insertPointTemp[2] = source[2] + (t[i] + 1e-2) * direction[2];

    //             if (IsPointInsideBox(insertPointTemp))
    //             {
    //                 if (tmin > t[i])
    //                 {
    //                     tmin = t[i];
    //                 }
    //                 found = true;
    //             }
    //         }
    //     }
    //     if (found)
    //     {
    //         insertPointTemp[0] = source[0] + (tmin + 1e-2) * direction[0];
    //         insertPointTemp[1] = source[1] + (tmin + 1e-2) * direction[1];
    //         insertPointTemp[2] = source[2] + (tmin + 1e-2) * direction[2];
    //         return true;
    //     }
    //     return false;
    // }

    bool DronePlanner::FindInsertPointRiskZone(const Vec3 &source, const Vec3 &target, Vec3 &insert_point)
    {
        const double epsilon = 1e-4;
        const double box_epsilon = 1e-2;

        Vec3 direction = target - source;
        double norm = direction.norm();
        direction.normalize();

        std::vector<double> intersections;
        Idx counter_direction = 0;

        for (Idx i = 0; i < 3; ++i)
        {
            if (std::abs(direction[i]) < epsilon)
            {
                ++counter_direction;
                if (counter_direction > 2)
                {
                    insert_point = source;
                    return false;
                }
            }
            else
            {
                intersections.push_back((LowerBordersXYZ[i] - source[i]) / direction[i]);
                intersections.push_back((UpperBordersXYZ[i] - source[i]) / direction[i]);
            }
        }

        std::sort(intersections.begin(), intersections.end());

        double tmin = std::numeric_limits<double>::max();
        bool found = false;

        for (const auto &t : intersections)
        {
            if (t > -norm && t < norm)
            {
                Vec3 temp_point = source + (t + box_epsilon) * direction;

                if (IsPointInsideBox(temp_point))
                {
                    if (t < tmin)
                    {
                        tmin = t;
                        found = true;
                    }
                }
            }
        }

        if (found)
        {
            insert_point = source + (tmin + box_epsilon) * direction;
            return true;
        }

        return false;
    }

    bool DronePlanner::FindExitPointRiskZone(const Vec3 &source, const Vec3 &target, Vec3 &exitPoint)
    {
        Vec3 direction = (target - source);
        auto normVec = direction.norm();
        direction.normalize();
        std::vector<double> t;
        Idx i = 0;
        for (i = 0; i < 3; i++)
        {
            t.push_back((LowerBordersXYZ[i] - source[i]) / direction[i]);
            t.push_back((UpperBordersXYZ[i] - source[i]) / direction[i]);
        }
        sort(t.begin(), t.end());
        for (i = 0; i < t.size(); i++)
        {
            if (t[i] > 0 && t[i] < normVec)
            {
                exitPoint[0] = source[0] + (t[i] + 1e-2) * direction[0];
                exitPoint[1] = source[1] + (t[i] + 1e-2) * direction[1];
                exitPoint[2] = source[2] + (t[i] + 1e-2) * direction[2];

                if (!IsPointInsideBox(exitPoint))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // void DronePlanner::RandomNoiseGPS(Vec3 &TotalLocationError)
    //     {
    //         RealNormalDist NormR(0, 1);
    //             RealNormalDist NormAngle(0, 2 * M_PI);
    //             auto r = NormR(rng);
    //             auto azimuth = NormAngle(rng);
    //             auto elevation = NormAngle(rng);

    //             TotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
    //             TotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
    //             TotalLocationError[i][2] = -r * sin(elevation);
    //     }

    // double DronePlanner::ComputeCost(const Vec3 TotalLocationError,const Vec3 TotalLocationError) const
    // {

    // }
    void DronePlanner::InsertIntermediatePointsToGraph(Inspection::Graph *graph)
    {
        Idx indexWhile = 0;
        Inspection::Graph *tempgraph = new Inspection::Graph();
        Idx counterAddEdges = 0;
        Idx counterAddVertex = 0;

        auto isPlusEpsInside = false;
        auto isMinusEpsInside = true;
        while (indexWhile < graph->NumEdges() - counterAddEdges)
        {
            auto edge = graph->Edge(indexWhile);
            if (!edge->valid)
            {
                indexWhile++;
                continue;
            }
            Vec3 source = graph->Vertex(edge->source)->state->as<DroneStateSpace::StateType>()->Position();
            Vec3 target = graph->Vertex(edge->target)->state->as<DroneStateSpace::StateType>()->Position();

            Vec3 direction = (target - source);
            auto normVec = direction.norm();
            direction.normalize();

            bool isSourceInside = IsPointInsideBox(source);
            bool istargetInside = IsPointInsideBox(target);

            if (isSourceInside && istargetInside) // convex Risk zone
            {
                indexWhile++;
                continue;
            }

            // std::cout << "p: " << source[0] << " " << source[1] << " " << source[2] << std::endl;

            std::vector<double> t;
            Idx i = 0;
            for (i = 0; i < 3; i++)
            {
                t.push_back((LowerBordersXYZ[i] - source[i]) / direction[i]);
                t.push_back((UpperBordersXYZ[i] - source[i]) / direction[i]);
            }
            sort(t.begin(), t.end());
            RealNum eps = 1E-2;
            Idx counterAddVertexPerEdge = 0;
            Idx indexEdge = indexWhile;
            bool isInside;
            bool isEnterAlready = false;
            Inspection::EPtr edgeTemp;
            for (i = 0; i < t.size(); i++)
            {
                if (counterAddVertexPerEdge >= 2)
                {
                    break;
                }
                if (t[i] > 0 && t[i] < normVec)
                {
                    // std::cout << t[i] << std::endl;
                    Vec3 intersectPointPlusEps = source + (t[i] + eps) * direction;
                    Vec3 intersectPointMinusEps = source + (t[i] - eps) * direction;

                    if (!isSourceInside)
                    {
                        isPlusEpsInside = IsPointInsideBox(intersectPointPlusEps);
                        isMinusEpsInside = IsPointInsideBox(intersectPointMinusEps);
                        if ((isPlusEpsInside || isMinusEpsInside) && !isEnterAlready)
                        {
                            counterAddVertexPerEdge++;
                            auto edgeToDevide = graph->Edge(indexEdge);
                            edgeTemp = InsertIntermediatePointToGraph(graph, edgeToDevide, intersectPointPlusEps);
                            indexEdge = graph->NumEdges() - 1;
                            // edge = graph->Edge(graph->NumEdges()-1);
                            counterAddEdges += 2;
                            counterAddVertex++;
                            isEnterAlready = true;
                        }
                    }
                    else
                    {
                        isPlusEpsInside = IsPointInsideBox(intersectPointPlusEps);
                        isMinusEpsInside = IsPointInsideBox(intersectPointMinusEps);
                        if (!(isPlusEpsInside && isMinusEpsInside))
                        {

                            auto edgeToDevide = graph->Edge(indexEdge);
                            if (isEnterAlready)
                            {
                                edgeToDevide = edgeTemp;
                            }

                            counterAddVertexPerEdge++;
                            edgeTemp = InsertIntermediatePointToGraph(graph, edgeToDevide, intersectPointPlusEps);
                            indexEdge = graph->NumEdges() - 1;
                            counterAddEdges += 2;
                            counterAddVertex++;

                            break;
                        }
                    }
                }
            }
            indexWhile++;
        }
        std::cout << "Add Intermediate Vertices : " << counterAddVertex << "\n";
        std::cout << "Add Intermediate Edges : " << counterAddEdges << "\n";
    }

    void DronePlanner::MarkEdgeRiskZone(Inspection::Graph *graph)
    {
        RealNum eps = 1E-3;

        for (Idx i = 0; i < graph->NumEdges(); i++)
        {
            auto edge = graph->Edge(i);
            Vec3 source = graph->Vertex(edge->source)->state->as<DroneStateSpace::StateType>()->Position();
            Vec3 target = graph->Vertex(edge->target)->state->as<DroneStateSpace::StateType>()->Position();

            Vec3 direction = (target - source);
            auto normVec = direction.norm();
            direction.normalize();

            auto middlePoint = source + normVec / 2 * direction;

            auto middlePointInRiskZone = IsPointInsideBox(middlePoint);
            edge->IsEdgeBelongToRiskZone = middlePointInRiskZone;
        }
    }

    bool DronePlanner::IsPointInsideBox(const Vec3 &point)
    {
        if ((point[0] >= LowerBordersXYZ[0]) && (point[0] <= UpperBordersXYZ[0]) &&
            (point[1] >= LowerBordersXYZ[1]) && (point[1] <= UpperBordersXYZ[1]) &&
            (point[2] >= LowerBordersXYZ[2]) && (point[2] <= UpperBordersXYZ[2]))
        {
            // std::cout << "IsPointInsideBox \n"<< std::endl;
            // getchar();
            return true;
        }
        return false;
    }

    void DronePlanner::InsertGraphPointOutSideToyProblem(Inspection::Graph *graph)
    {
        auto NumVerticesOutside_1 = graph->NumVertices();
        auto NumVerticesOutside_2 = 0;
        Vec3 pos;
        pos[0] = LowerBordersXYZ[0] - 5; // robot_->Config()->Position()[0];
        pos[1] = UpperBordersXYZ[1] + 3;
        pos[2] = robot_->Config()->Position()[2];
        auto cameraAngle = robot_->Config()->CameraAngle();
        auto yaw = robot_->Config()->Yaw();
        auto FirstNumVertices = graph->NumVertices();
        // auto MinusNumVertices = 0;
        // auto PlusNumVertices = 0;
        Idx i = 0;
        // add vertices Under the bridge
        Idx j = 0;

        for (j = 0; j < 2; j++)
        {
            // if (j==0)
            // {
            //     cameraAngle = -0.7854;
            // }
            // else
            // {
            //     cameraAngle = 0.7854;
            // }
            auto numberOfOutsidePoints = 50;
            if (j > 0)
            {
                NumVerticesOutside_2 = graph->NumVertices();
            }
            for (i = 0; i < numberOfOutsidePoints / 2; i++)
            {
                auto numVertices = graph->NumVertices();
                graph->AddVertex(numVertices);
                auto vertex = graph->Vertex(numVertices);
                vertex->state = space_info_->allocState();

                // space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
                // std::cout << "pos: " << pos[0] << "pos: " << pos[1]<< "pos: " << pos[2] << std::endl;

                vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(cameraAngle);
                vertex->state->as<DroneStateSpace::StateType>()->SetYaw(yaw);

                const TimePoint start = Clock::now();
                // ComputeVisibilitySet(vertex);
                vertex->time_vis = 0;   // RelativeTime(start);
                vertex->time_build = 0; // todo calculate time_build of IntermediatePoint

                // graph->UpdateGlobalVisibility(vertex->vis);
                pos[0] += (UpperBordersXYZ[0] - LowerBordersXYZ[0]) / (numberOfOutsidePoints / 2);
            }
            // MinusNumVertices = graph->NumVertices();
            pos[0] = LowerBordersXYZ[0] - 5;
            pos[1] = LowerBordersXYZ[1] - 3;
        }

        for (i = NumVerticesOutside_1; i < graph->NumVertices(); i++)
        {
            // std::cout << "NumVerticesOutside_1: " << NumVerticesOutside_1 << "\n" ;
            // std::cout << "NumVerticesOutside_2: " << NumVerticesOutside_2 << "\n" ;
            for (j = 0; j < NumVerticesOutside_1; j++)
            {
                // if (i==j || (i<=NumVerticesOutside_1 && j<=NumVerticesOutside_1))
                if (i == j)
                {
                    continue;
                }
                const ob::State *source = graph->Vertex(i)->state;
                const ob::State *target = graph->Vertex(j)->state;
                auto cost = space_info_->distance(source, target);
                // if (cost>(UpperBordersXYZ[1]-LowerBordersXYZ[1])/2+10)
                if (cost > (UpperBordersXYZ[1] - LowerBordersXYZ[1]) / 2 + 6)
                {
                    continue;
                }
                auto numEdges = graph->NumEdges();

                Inspection::EPtr edge1(new Inspection::Edge(i, j));
                edge1->cost = cost;
                if (validate_all_edges_)
                {
                    const TimePoint start = Clock::now();
                    bool valid = this->CheckEdge(source, target);
                    edge1->checked = valid;
                    // todo david
                    edge1->valid = true; // this->CheckEdge(source,target);
                    edge1->time_forward_kinematics = RelativeTime(start);
                }

                graph->AddEdge(edge1);
            }
        }
        // auto PlusNumVertices = graph->NumVertices();
        // auto MinusNumVertices=(PlusNumVertices-FirstNumVertices)/2+FirstNumVertices-1;
        // for (i = FirstNumVertices+1; i < MinusNumVertices-1; i++)
        // {
        //     auto numEdges = graph->NumEdges();
        //     const ob::State *source = graph->Vertex(i)->state;
        //     const ob::State *target = graph->Vertex(i+1)->state;
        //     Inspection::EPtr edge1(new Inspection::Edge(i, i+1));
        //     edge1->cost = space_info_->distance(source,target);
        //     if (validate_all_edges_)
        //     {
        //         const TimePoint start = Clock::now();
        //         //bool valid = this->CheckEdge(source,intermediate);
        //         //edge->checked = valid;
        //         edge1->valid = true;
        //         edge1->time_forward_kinematics = RelativeTime(start);
        //     }

        //     graph->AddEdge(edge1);
        // }

        // for (i = MinusNumVertices+1; i < PlusNumVertices-1; i++)
        // {
        //     auto numEdges = graph->NumEdges();
        //     const ob::State *source = graph->Vertex(i)->state;
        //     const ob::State *target = graph->Vertex(i+1)->state;
        //     Inspection::EPtr edge1(new Inspection::Edge(i, i+1));
        //     edge1->cost = space_info_->distance(source,target);
        //     if (validate_all_edges_)
        //     {
        //         const TimePoint start = Clock::now();
        //         //bool valid = this->CheckEdge(source,intermediate);
        //         //edge->checked = valid;
        //         edge1->valid = true;
        //         edge1->time_forward_kinematics = RelativeTime(start);
        //     }

        //     graph->AddEdge(edge1);
        // }

        // for (i = FirstNumVertices+1; i < PlusNumVertices; i++)
        // {
        //     for (j=0;j<FirstNumVertices;j++)
        //     {
        //     auto numEdges = graph->NumEdges();
        //     const ob::State *source = graph->Vertex(i)->state;
        //     const ob::State *target = graph->Vertex(j)->state;
        //     Inspection::EPtr edge1(new Inspection::Edge(i, j));
        //     edge1->cost = space_info_->distance(source,target);
        //     if (validate_all_edges_)
        //     {
        //         const TimePoint start = Clock::now();
        //         //bool valid = this->CheckEdge(source,intermediate);
        //         //edge->checked = valid;
        //         edge1->valid = true;
        //         edge1->time_forward_kinematics = RelativeTime(start);
        //     }

        //     graph->AddEdge(edge1);
        //     }
        // }
    }

    void DronePlanner::BuildRRGIncrementally(Inspection::Graph *graph,
                                             ob::PlannerPtr &planner,
                                             ob::PlannerData &tree_data,
                                             ob::PlannerData &graph_data)
    {
        // use ompl planner
        const TimePoint start = Clock::now();
        ob::PlannerStatus solved;
        ob::IterationTerminationCondition ptc(incremental_step_);
        solved = planner->solve(ptc);
        planner->as<og::RRG>()->getPlannerData(tree_data);
        planner->as<og::RRG>()->getUncheckedEdges(graph_data);
        SizeType total_time_build = RelativeTime(start);

        SizeType avg_time_build = 0;
        Idx prev_size = graph->NumVertices();
        Idx current_size = tree_data.numVertices();

        if (current_size > prev_size)
        {
            avg_time_build = SizeType(total_time_build / (current_size - prev_size));
        }
        else
        {
            return;
        }

        // update inspection graph
        for (Idx i = prev_size; i < current_size; ++i)
        {
            std::cout << i << " " << std::flush;
            graph->AddVertex(i);
            auto vertex = graph->Vertex(i);
            vertex->state = space_info_->allocState();
            space_info_->copyState(vertex->state, tree_data.getVertex(i).getState());

            const TimePoint start = Clock::now();
            ComputeVisibilitySet(vertex);
            // Check if the number of covered targets in the graph is greater than 20
            // std::cout << "sss" << std ::endl;
            // std::cout << "graph->NumTargetsCovered() " << graph->NumTargetsCovered() << std ::endl;

            {
                if (graph->NumTargetsCovered() >= 15)
                    // Iterate over the bitset of the global visited set
                    for (Idx j = 0; j < MAX_COVERAGE_SIZE; j++)
                    {
                        // If the j-th bit is not set (i.e., less than 0.5), reset it in the current vertex's visited set
                        if (global_vis_set_.bitset_[j] < 0.5)
                        {

                            vertex->vis.bitset_[j] = 0;
                        }
                    }
            }
            vertex->time_vis = RelativeTime(start);
            vertex->time_build = avg_time_build;

            graph->UpdateGlobalVisibility(vertex->vis);
#if REJECT_SAMPLING
            global_vis_set_.Insert(graph->GlobalVisibility());
#endif

            // tree edges
            std::vector<unsigned> edges;
            auto num_parent = tree_data.getIncomingEdges(i, edges);

            if (num_parent == 1)
            {
                Idx p = edges[0];
                Inspection::EPtr edge(new Inspection::Edge(p, i));
                edge->checked = true;
                edge->valid = true;
                const ob::State *source = tree_data.getVertex(p).getState();
                const ob::State *target = tree_data.getVertex(i).getState();
                bool valid = this->CheckEdge(source, target);
                edge->checked = true;
                edge->valid = valid;
                edge->cost = space_info_->distance(tree_data.getVertex(p).getState(),
                                                   tree_data.getVertex(i).getState());
                graph->AddEdge(edge);
            }
            else if (num_parent != 0)
            {
                std::cerr << "More than one parent! Error!" << std::endl;
                exit(1);
            }

            // graph edges
            edges.clear();
            graph_data.getEdges(i, edges);

            for (auto &&e : edges)
            {
                Inspection::EPtr edge(new Inspection::Edge(i, e));
                const ob::State *source = tree_data.getVertex(i).getState();
                const ob::State *target = tree_data.getVertex(e).getState();
                edge->cost = space_info_->distance(source, target);

                if (validate_all_edges_)
                {
                    const TimePoint start = Clock::now();
                    bool valid = this->CheckEdge(source, target);
                    edge->checked = true;
                    edge->valid = valid;
                    edge->time_forward_kinematics = RelativeTime(start);
                }

                graph->AddEdge(edge);
            }
        }
    }

    void DronePlanner::ComputeRobotVisibilitySet(VisibilitySet &vis_set) const
    {
        auto visible_points = env_->GetVisiblePointIndices(robot_->CameraPos(),
                                                           robot_->CameraTangent(),
                                                           robot_->FOV(),
                                                           robot_->MinDOF(),
                                                           robot_->MaxDOF());
        vis_set.Clear();

        for (auto p : visible_points)
        {
            vis_set.Insert(p);
        }
    }

    void DronePlanner::ComputeVisibilitySet(Inspection::VPtr vertex) const
    {
        // const Idx index = vertex->index;
        // VertexVisCache::const_iterator it = vertex_vis_cache_.find(index);
        // if (it != vertex_vis_cache_.end())
        // {
        //     // The visibility set for this vertex has already been computed and cached.
        //     vertex->vis = it->second;
        //     return;
        // }
        const auto &s = vertex->state->as<DroneStateSpace::StateType>();
        robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
        robot_->ComputeShape();
        this->ComputeRobotVisibilitySet(vertex->vis);
        // vertex_vis_cache_[index] = vertex->vis;
    }

    bool DronePlanner::StateValid(const ob::State *state)
    {
        const auto &s = state->as<DroneStateSpace::StateType>();

        auto collision_free = env_->IsCollisionFree(s->Position(), robot_->SphereRadius());

        if (!collision_free || !extern_validate_sample)
        {
            return collision_free;
        }

        extern_validate_sample = false;
        robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
        robot_->ComputeShape();

        bool valid_direction = env_->IfCorrectDirection(robot_->CameraPos(), robot_->CameraTangent(),
                                                        robot_->FOV(),
                                                        validation_distance_);

        if (!valid_direction && RandomRealNumber(0, 1) < reject_ratio_)
        {
            // Camera is not facing the correct direction.
            return false;
        }

#if REJECT_SAMPLING
        bool valid = true;

        RealNum coverage = global_vis_set_.Size() / (RealNum)num_targets_;

        if (coverage > REJECT_START_COVERAGE)
        {
            if (RandomRealNumber(0, 1) < reject_check_ratio_)
            {
                VisibilitySet vis_set;
                this->ComputeRobotVisibilitySet(vis_set);
                // todo david
                if (global_vis_set_.Size() >= 15)
                {
                    // Iterate over the bitset of the global visited set
                    for (Idx j = 0; j < MAX_COVERAGE_SIZE; j++)
                    {
                        // If the j-th bit is not set (i.e., less than 0.5), reset it in the current vertex's visited set
                        if (global_vis_set_.bitset_[j] < 0.5)
                        {

                            vis_set.bitset_[j] = 0;
                        }
                    }
                }
                vis_set.Insert(global_vis_set_);
                RealNum extend_ratio = (vis_set.Size() - global_vis_set_.Size()) / (RealNum)num_targets_;
                valid = (extend_ratio > coverage_min_extend_);
                // std::cout << "extend_ratio  " << extend_ratio << std::endl;
                // std::cout << "coverage_min_extend_  " << coverage_min_extend_ << std::endl;

                if (!valid)
                {
                    invalid_states_counter_++;
                    // std::cout << "invalid_states_counter_  " << invalid_states_counter_ << std::endl;
                }
                else
                {
                    invalid_states_counter_ = 0;
                    reject_check_ratio_ *= 1.05;
                    reject_check_ratio_ = std::fmin(reject_check_ratio_, MAX_REJECT_CHECK_RATIO);

                    // coverage_min_extend_ *= 1.5;
                    // coverage_min_extend_ = std::fmin(coverage_min_extend_, COVERAGE_MIN_EXTEND);

                    global_vis_set_.Insert(vis_set);
                }
            }

            if (invalid_states_counter_ == REJECT_THRESHOLD)
            {
                // Perform less rejection.
                reject_check_ratio_ *= 0.95;
                reject_check_ratio_ = std::fmax(reject_check_ratio_, MIN_REJECT_CHECK_RATIO);

                // Set a lower bar for accepting a state.
                coverage_min_extend_ *= 0.5;

                // Reset counter
                invalid_states_counter_ = 0;
            }
        }

        return valid;
#endif

        return true;
    }

    bool DronePlanner::CheckEdge(const ob::State *source, const ob::State *target) const
    {
        const auto &s0 = source->as<DroneStateSpace::StateType>();
        const auto &s1 = target->as<DroneStateSpace::StateType>();
        Vec3 p0 = s0->Position();
        Vec3 p1 = s1->Position();

        // Idx num_steps = std::ceil((p0 - p1).norm() / 0.5);//todo david
        Idx num_steps = std::ceil((p0 - p1).norm() / (robot_->SphereRadius() * 0.5));

        // std::cout << "num_steps" << num_steps << " " << robot_->SphereRadius()  << " " << (p0 - p1).norm()<< std::endl;

        for (Idx i = 1; i < num_steps; ++i)
        {
            RealNum p = i / (RealNum)num_steps;
            Vec3 config_mid = p0 * p + p1 * (1 - p);

            if (!env_->IsCollisionFree(config_mid, robot_->SphereRadius()))
            {
                return false;
            }
        }

        return true;
    }

    RealNum DronePlanner::RandomRealNumber(const RealNum lower_bound, const RealNum higher_bound)
    {
        return uni_(rng_) * (higher_bound - lower_bound) + lower_bound;
    }

    SizeType DronePlanner::RelativeTime(const TimePoint start) const
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
    }
}
