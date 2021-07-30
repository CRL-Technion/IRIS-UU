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
                pos[j] = uni(rng) * (hi - lo) + lo;
            }


            yaw = uni(rng) * (kMaxYaw - kMinYaw) + kMinYaw;
            camera_angle = uni(rng) * (kMaxCameraAngle - kMinCameraAngle) + kMinCameraAngle;

#if UAV_NAVIGATION_ERROR
            pos[0] = -102;
            pos[1] = -11;
            yaw = 0.0;
            camera_angle = 0;
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
        }

        std::cout << "Fail to find valid start config." << std::endl;
        exit(1);
    }

    void DronePlanner::SetParams(const RealNum step_size, const bool if_k_nearest)
    {
        step_size_ = step_size;
        k_nearest_ = if_k_nearest;
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
        planner->as<og::RRG>()->setRange(step_size_);
        planner->as<og::RRG>()->setGoalBias(0.0);
        planner->as<og::RRG>()->setKNearest(k_nearest_);
        planner->setProblemDefinition(problem_def);
        planner->setup();

        // Build graph incrementally.
        Inspection::Graph *graph = new Inspection::Graph();

        ob::PlannerData tree_data(space_info_);
        ob::PlannerData graph_data(space_info_);
        auto ToleranceBorder = 2;
        Vec3 LowerBordersXYZ{-100 - ToleranceBorder, -22 - ToleranceBorder, -20 - ToleranceBorder};
        Vec3 UpperBordersXYZ{100 + ToleranceBorder, 2 + ToleranceBorder, 0 + ToleranceBorder};
#if UAV_NAVIGATION_ERROR
        InsertGraphPointToyProblem(graph, LowerBordersXYZ, UpperBordersXYZ);
        InsertGraphPointOutSideToyProblem(graph, LowerBordersXYZ, UpperBordersXYZ);
        InsertIntermediatePointsToGraph(graph, LowerBordersXYZ, UpperBordersXYZ);
        MarkEdgeRiskZone(graph, LowerBordersXYZ, UpperBordersXYZ);

        std::cout << "Covered targets: " << graph->NumTargetsCovered()
                  << ", " << graph->NumTargetsCovered() * (RealNum)100 / num_targets_ << "%" << std::endl;

#else
        while (graph->NumVertices() < target_size)
        {
            BuildRRGIncrementally(graph, planner, tree_data, graph_data);
            std::cout << "Covered targets: " << graph->NumTargetsCovered()
                      << ", " << graph->NumTargetsCovered() * (RealNum)100 / num_targets_ << "%" << std::endl;
        }
#endif

        graph->Save(file_name, true);

        delete graph;
    }
    void DronePlanner::InsertGraphPointToyProblem(Inspection::Graph *graph, Vec3 LowerBordersXYZ, Vec3 UpperBordersXYZ)
    {
        Vec3 pos;
        pos[0] = -100; //robot_->Config()->Position()[0];
        pos[1] = robot_->Config()->Position()[1];
        pos[2] = robot_->Config()->Position()[2];
        auto cameraAngle = robot_->Config()->CameraAngle();
        auto yaw = robot_->Config()->Yaw();
        Idx i = 0;
        //add vertices Under the bridge
        auto numberOfVerticesUnderTheBridge = 100;
        for (i = 0; i < numberOfVerticesUnderTheBridge; i++)
        {

            auto numVertices = graph->NumVertices();
            graph->AddVertex(numVertices);
            auto vertex = graph->Vertex(numVertices);
            vertex->state = space_info_->allocState();

            //space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
            vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
            // std::cout << "pos: " << pos[0] << "pos: " << pos[1]<< "pos: " << pos[2] << std::endl;

            vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(cameraAngle);
            vertex->state->as<DroneStateSpace::StateType>()->SetYaw(yaw);

            const TimePoint start = Clock::now();
            ComputeVisibilitySet(vertex);
            vertex->time_vis = RelativeTime(start);
            vertex->time_build = 0; //todo calculate time_build of IntermediatePoint

            graph->UpdateGlobalVisibility(vertex->vis);
            pos[0] += (UpperBordersXYZ[0] - LowerBordersXYZ[0]) / numberOfVerticesUnderTheBridge;
            // if (pos[0]> -53 && pos[0] <-7)
            // {
            //     pos[1] = -10;
            // }
            // else{
            //     pos[1] = -18;
            // }
            //std::cout << "InsertGraphPointToyProblem: " << i << std::endl;
        }
        std::cout << "graph->NumVertices(): " << graph->NumVertices() << std::endl;
        auto numVerticesUnderTheBridge = graph->NumVertices();
        for (i = 0; i < graph->NumVertices() - 1; i++)
        {
            auto numEdges = graph->NumEdges();
            const ob::State *source = graph->Vertex(i)->state;
            const ob::State *target = graph->Vertex(i + 1)->state;
            Inspection::EPtr edge1(new Inspection::Edge(i, i + 1));
            edge1->cost = space_info_->distance(source, target);
            if (validate_all_edges_)
            {
                const TimePoint start = Clock::now();
                //bool valid = this->CheckEdge(source,intermediate);
                edge1->checked = true;
                edge1->valid = true;
                edge1->time_forward_kinematics = RelativeTime(start);
            }

            graph->AddEdge(edge1);
        }
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
        //space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
        vertex->state->as<DroneStateSpace::StateType>()->SetPosition(InsertIntermediatePosition);

        auto SumCameraAngle = (graph->Vertex(sourceIndex)->state->as<DroneStateSpace::StateType>()->CameraAngle() + graph->Vertex(targetIndex)->state->as<DroneStateSpace::StateType>()->CameraAngle());
        auto SumYaw = (graph->Vertex(sourceIndex)->state->as<DroneStateSpace::StateType>()->Yaw() + graph->Vertex(targetIndex)->state->as<DroneStateSpace::StateType>()->Yaw());
        vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(SumCameraAngle / 2.0);
        vertex->state->as<DroneStateSpace::StateType>()->SetYaw(SumYaw / 2.0);

        const TimePoint start = Clock::now();
        //ComputeVisibilitySet(vertex);
        vertex->time_vis = RelativeTime(start);
        vertex->time_build = 0; //todo calculate time_build of IntermediatePoint

        //graph->UpdateGlobalVisibility(vertex->vis);

        const ob::State *source = graph->Vertex(sourceIndex)->state;
        const ob::State *intermediate = graph->Vertex(numVertices)->state;
        const ob::State *target = graph->Vertex(targetIndex)->state;

        //add the first edge
        Inspection::EPtr edge1(new Inspection::Edge(sourceIndex, numVertices));
        edge1->cost = space_info_->distance(source, intermediate);

        if (validate_all_edges_)
        {
            const TimePoint start = Clock::now();
            //bool valid = this->CheckEdge(source,intermediate);
            edge->checked = true;
            edge1->valid = true;
            edge1->time_forward_kinematics = RelativeTime(start);
        }

        graph->AddEdge(edge1);

        //add the second edge
        Inspection::EPtr edge2(new Inspection::Edge(numVertices, targetIndex));
        edge2->cost = space_info_->distance(intermediate, target);

        if (validate_all_edges_)
        {
            const TimePoint start = Clock::now();
            //bool valid = this->CheckEdge(intermediate,target);
            edge->checked = true;
            edge2->valid = true;
            edge2->time_forward_kinematics = RelativeTime(start);
        }

        graph->AddEdge(edge2);
        return edge2;
    }

    void DronePlanner::InsertIntermediatePointsToGraph(Inspection::Graph *graph, Vec3 LowerBordersXYZ, Vec3 UpperBordersXYZ)
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

            bool isSourceInside = IsPointInsideBox(source, LowerBordersXYZ, UpperBordersXYZ);
            bool istargetInside = IsPointInsideBox(target, LowerBordersXYZ, UpperBordersXYZ);

            if (isSourceInside && istargetInside) //convex Risk zone
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
                    //std::cout << t[i] << std::endl;
                    Vec3 intersectPointPlusEps = source + (t[i] + eps) * direction;
                    Vec3 intersectPointMinusEps = source + (t[i] - eps) * direction;

                    if (!isSourceInside)
                    {
                        isPlusEpsInside = IsPointInsideBox(intersectPointPlusEps, LowerBordersXYZ, UpperBordersXYZ);
                        isMinusEpsInside = IsPointInsideBox(intersectPointMinusEps, LowerBordersXYZ, UpperBordersXYZ);
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
                        isPlusEpsInside = IsPointInsideBox(intersectPointPlusEps, LowerBordersXYZ, UpperBordersXYZ);
                        isMinusEpsInside = IsPointInsideBox(intersectPointMinusEps, LowerBordersXYZ, UpperBordersXYZ);
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

    void DronePlanner::MarkEdgeRiskZone(Inspection::Graph *graph, Vec3 LowerBordersXYZ, Vec3 UpperBordersXYZ)
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
            // auto sourceInRiskZone = IsPointInsideBox(SourcePoint,LowerBordersXYZ,UpperBordersXYZ);
            // auto TargetPoint = target - eps*direction;
            // auto targetInRiskZone = IsPointInsideBox(TargetPoint,LowerBordersXYZ,UpperBordersXYZ);
            // edge->IsBelongToRiskZone = sourceInRiskZone && targetInRiskZone;

            auto middlePointInRiskZone = IsPointInsideBox(middlePoint, LowerBordersXYZ, UpperBordersXYZ);
            edge->IsEdgeBelongToRiskZone = middlePointInRiskZone;

            // if (ToyProblem)
            // {
            //     if (edge->source <2*62 && edge->target<2*62)
            //     {
            //        edge->IsBelongToRiskZone=1;

            //     }
            //     continue;
            //     // auto temp =IsPointInsideBox(source,LowerBordersXYZ,UpperBordersXYZ);
            //     // if (temp)
            //     // {
            //     //     edge->IsBelongToRiskZone = IsPointInsideBox(target,LowerBordersXYZ,UpperBordersXYZ);;
            //     // }

            //     // continue;
            // }
            // edge->IsBelongToRiskZone = IsPointInsideBox(intersectPoint,LowerBordersXYZ,UpperBordersXYZ);
        }
    }

    bool DronePlanner::IsPointInsideBox(const Vec3 &point, Vec3 LowerBordersXYZ, Vec3 UpperBordersXYZ)
    {
        if ((point[0] > LowerBordersXYZ[0]) && (point[0] < UpperBordersXYZ[0]) && (point[1] > LowerBordersXYZ[1]) && (point[1] < UpperBordersXYZ[1]) && (point[2] > LowerBordersXYZ[2]) && (point[2] < UpperBordersXYZ[2]))
        {
            // std::cout << "IsPointInsideBox \n"<< std::endl;
            // getchar();
            return true;
        }
        return false;
    }

    void DronePlanner::InsertGraphPointOutSideToyProblem(Inspection::Graph *graph, Vec3 LowerBordersXYZ, Vec3 UpperBordersXYZ)
    {
        auto NumVerticesOutside_1 = graph->NumVertices();
        auto NumVerticesOutside_2 = 0;
        Vec3 pos;
        pos[0] = LowerBordersXYZ[0] - 5; //robot_->Config()->Position()[0];
        pos[1] = UpperBordersXYZ[1] + 3;
        pos[2] = robot_->Config()->Position()[2];
        auto cameraAngle = robot_->Config()->CameraAngle();
        auto yaw = robot_->Config()->Yaw();
        auto FirstNumVertices = graph->NumVertices();
        // auto MinusNumVertices = 0;
        // auto PlusNumVertices = 0;
        Idx i = 0;
        //add vertices Under the bridge
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
            auto numberOfOutsidePoints = 36;
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

                //space_info_->copyState(vertex->state, graph->Vertex(targetIndex)->state);
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
                // std::cout << "pos: " << pos[0] << "pos: " << pos[1]<< "pos: " << pos[2] << std::endl;

                vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(cameraAngle);
                vertex->state->as<DroneStateSpace::StateType>()->SetYaw(yaw);

                const TimePoint start = Clock::now();
                // ComputeVisibilitySet(vertex);
                vertex->time_vis = 0;   //RelativeTime(start);
                vertex->time_build = 0; //todo calculate time_build of IntermediatePoint

                //graph->UpdateGlobalVisibility(vertex->vis);
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
                //if (cost>(UpperBordersXYZ[1]-LowerBordersXYZ[1])/2+10)
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
                    //todo david
                    edge1->valid = true; //this->CheckEdge(source,target);
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
                    // edge->checked = true;
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
        const auto &s = vertex->state->as<DroneStateSpace::StateType>();
        robot_->SetConfig(s->Position(), s->Yaw(), s->CameraAngle());
        robot_->ComputeShape();
        this->ComputeRobotVisibilitySet(vertex->vis);
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
                vis_set.Insert(global_vis_set_);
                RealNum extend_ratio = (vis_set.Size() - global_vis_set_.Size()) / (RealNum)num_targets_;
                valid = (extend_ratio > coverage_min_extend_);

                if (!valid)
                {
                    invalid_states_counter_++;
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

        Idx num_steps = std::ceil((p0 - p1).norm() / 0.5);

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
