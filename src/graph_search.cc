#include "graph_search.h"

Inspection::VPtr vertex(new Inspection::Vertex(0));
Inspection::VPtr parentVertex(new Inspection::Vertex(0));
Rand rng;
GraphSearch::GraphSearch(const Inspection::GPtr graph) : graph_(graph)
{
    virtual_graph_coverage_.Clear();
    open_sets_.clear();
    closed_sets_.clear();

    auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, -0.049);
    robot->SetCameraParameters(94.0 / 180 * M_PI, 0.2, 10.0);
    robot->Initialize();

    // Environment setup.
    auto env = std::make_shared<drone::BridgeEnvironment>();

    // Planner
    planner = std::make_shared<drone::DronePlanner>(robot, env, 1);

    // space_info_ = ob::StateSpacePtr(new DroneStateSpace());
    // auto state_space_ = ob::StateSpacePtr(new DroneStateSpace());

    // space_info_.reset(new ob::SpaceInformation(state_space_));
    // using namespace std::placeholders;
    // space_info_->setStateValidityChecker(std::bind(&DronePlanner::StateValid, this, _1));
    // space_info_->setStateValidityCheckingResolution(0.1);
    // space_info_->setup();
    // VPtr v(new Inspection::Vertex(0));
    // Inspection::VPtr vertex(new Inspection::Vertex(0));
    // vertex(new Inspection::Vertex(0));
    space_info_ = planner->Define_space_info_();
    vertex->state = space_info_->allocState();
    parentVertex->state = space_info_->allocState();
}

void GraphSearch::ReadLocationErrorParameters(const String Location_Error_file_name)
{
    std::ifstream fin;
    //String fileLocationError ="LocationErrorParameterFile";
    fin.open(Location_Error_file_name);
    if (!fin.is_open())
    {
        std::cerr << "LocationErrorParameters file cannot be opened!" << std::endl;
        exit(1);
    }

    String line;
    Idx i = 0;
    Idx MonteCarloNumber;
    while (getline(fin, line))
    {
        std::istringstream sin(line);
        String field;

        while (getline(sin, field, ' '))
        {
            sin >> b_a_milli_g;
            sin >> b_g_degPerHr;
            sin >> avarageVelocity;
            sin >> minTimeAllowInRistZone;
            sin >> maxTimeAllowInRistZone;
            sin >> multipleCostFunction;
            sin >> MonteCarloNumber;
            // = std::stoi(field);
            // b_g_degPerHr = std::stoi(field);
            // avarageVelocity = std::stoi(field);
            // minTimeAllowInRistZone = std::stoi(field);
            // multipleCostFunction = std::stoi(field);

            break;
        }
    }
    fin.close();
    std::cout << "LocationErrorParameters read!" << std::endl;
    std::cout << b_a_milli_g << " " << b_g_degPerHr << " " << avarageVelocity << " " << minTimeAllowInRistZone << " " << maxTimeAllowInRistZone << " " << multipleCostFunction << " " << MonteCarloNumber << std::endl;

    //rund monteCarloParameter

    rng.seed(1);
    //Idx MonteCarloNumber = 5;
    auto milli_g2mpss = 9.81 / 1000.0;                 //   Conversion from [mili g ] to [m/s^2]
    auto degPerHr2radPerSec = (3.14 / 180.0) / 3600.0; //  Conversion from [deg/hr] to [rad/s]
    auto b_a = b_a_milli_g * milli_g2mpss;
    auto b_g = b_g_degPerHr * degPerHr2radPerSec;
    //auto ba_input = -b_a/ 5.0 ;
    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        RealNormalDist Norm1(0, b_a / 5.0);
        ba_x.push_back(Norm1(rng));
        ba_y.push_back(Norm1(rng));
        ba_z.push_back(Norm1(rng));
        //ba_input+=2*(b_a/5.0)/MonteCarloNumber-1;
        //RealNormalDist Norm1(0,  +1e-10);
        // ba_x.push_back(Norm1(rng)+ba_input);
        // ba_y.push_back(Norm1(rng)+ba_input);
        // ba_z.push_back(Norm1(rng)+ba_input);
        std::cout << "b_a = " << ba_x[i] << "," << ba_y[i] << "," << ba_z[i] << "," << std::endl;
    }
    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        RealNormalDist Norm2(0, b_g / 5.0);

        bg_x.push_back(Norm2(rng));
        bg_y.push_back(Norm2(rng));
        bg_z.push_back(Norm2(rng));
        std::cout << "b_g = " << bg_x[i] << "," << bg_y[i] << "," << bg_z[i] << "," << std::endl;
    }

    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        Vec3 temp{0.0, 0.0, 0.0};
        totalLocationErrorDefault.push_back(temp);
        exitRiskZoneDefault.push_back(false);
        costToComeRiskZoneDefault.push_back(0.0);
    }
}

void GraphSearch::SetLazinessMode(const Idx mode_id)
{
    if (mode_id >= kLazinessMap.size())
    {
        std::cerr << "Invalid laziness mode! Keep using old mode "
                  << kLazinessMap.at(laziness_mode_)
                  << std::endl;
        return;
    }

#if KEEP_SUBSUMING_HISTORY == 0

    if (kLazinessMap.at(laziness_mode_) == "LazyA*")
    {
        std::cerr << "LazyA* cannot work without subsuming history"
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;
        return;
    }

#endif

    laziness_mode_ = mode_id;
    std::cout << "Set laziness mode to "
              << kLazinessMap.at(mode_id)
              << std::endl;
}

void GraphSearch::SetSuccessorMode(const Idx mode_id)
{
    if (mode_id >= kSuccessorMap.size())
    {
        std::cerr << "Invalid successor mode! Keep using old mode "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;
        return;
    }

#if USE_NODE_REUSE

    if (kSuccessorMap.at(mode_id) != "direct")
    {
        std::cerr << "Node reusing does not support successor mode "
                  << kSuccessorMap.at(mode_id)
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;

        return;
    }

#endif

    if (mode_id > 0 && laziness_mode_ > 1)
    {
        std::cerr << kLazinessMap.at(laziness_mode_)
                  << " does not support successor mode "
                  << kSuccessorMap.at(mode_id)
                  << ", keep using "
                  << kSuccessorMap.at(successor_mode_)
                  << std::endl;

        return;
    }

    successor_mode_ = mode_id;
    std::cout << "Set successor mode to "
              << kSuccessorMap.at(mode_id)
              << std::endl;
}

void GraphSearch::SetSourceIndex(const Idx source)
{
    source_idx_ = source;
}

void GraphSearch::UpdateApproximationParameters(const RealNum eps, const RealNum p)
{
#if USE_GHOST_DATA
    eps_ = eps < 0.0 ? 0.0 : eps;
    p_ = p > 1.0 ? 1.0 : p;
#endif
}

SizeType GraphSearch::ExpandVirtualGraph(SizeType new_size)
{
    if (virtual_graph_size_ == graph_->NumVertices())
    {
        std::cout << "Pre-computed graph with size " << virtual_graph_size_ << " is exhausted!" << std::endl;
        return virtual_graph_size_;
    }

    if (new_size > graph_->NumVertices())
    {
        new_size = graph_->NumVertices();
    }

    for (SizeType i = virtual_graph_size_; i < new_size; ++i)
    {
        Inspection::VPtr v = graph_->Vertex(i);
        time_vis_ += v->time_vis;
        time_build_ += v->time_build;
        virtual_graph_coverage_.Insert(v->vis);
    }

    for (SizeType i = 0; i < graph_->NumEdges(); ++i)
    {
        Inspection::EPtr e = graph_->Edge(i);

        if (e->source >= new_size)
        {
            break;
        }

        if ((!e->in_virtual_graph) && e->target < new_size)
        {
            e->in_virtual_graph = true;
            virtual_graph_num_edges_++;

            if (kLazinessMap.at(laziness_mode_) == "no lazy" || e->checked)
            {
                time_valid_ += e->time_forward_kinematics;
                time_valid_ += e->time_collision_detection;
                num_validated_++;
                e->virtual_checked = true;
            }
        }
    }

    virtual_graph_size_ = new_size;
    return virtual_graph_size_;
}

std::vector<Idx> GraphSearch::SearchVirtualGraph()
{
    const TimePoint start = Clock::now();

    NodePtr result_node = nullptr;
    bool valid_result_found = false;
    SizeType num_replan = 0;
    num_nodes_extended_ = 0;
    num_nodes_generated_ = 0;
    num_nodes_remained_ = 0;

    while (!valid_result_found)
    {
        // Reset result node.
        result_node = nullptr;

        InitDataStructures();

        // Search starts.
        bool found = false;

        while (!queue_->empty())
        {
            NodePtr n = PopFromPriorityQueue();

            // Not a valid node.
            if (n == nullptr)
            {
                continue;
            }

            // Updage result.
            if (result_node == nullptr || n->BetterThan(result_node))
            {
                std::cout << "virtual_graph_coverage_.Size(): " << virtual_graph_coverage_.Size() << std::endl;
                result_node = n;
                std::cout << "this->CoverageSize(): " << result_node->CoverageSize() << std::endl;
                for (size_t i = 0; i < MAX_COVERAGE_SIZE; i++)
                {
                    if (i == MAX_COVERAGE_SIZE - 1)
                    {
                        std::cout << " " << result_node->VisSet().bitset_[i] << std::endl;
                    }
                    else
                    {
                        std::cout << " " << result_node->VisSet().bitset_[i];
                    }
                }
                std::vector<Idx> path;
                auto tag = result_node;
                path.push_back(tag->Index());

                while (tag)
                {
                    auto super_edge = tag->LocalPath();

                    for (auto i = 1; i < super_edge.size(); ++i)
                    {
                        path.push_back(super_edge[i]);
                    }

                    tag = tag->Parent();
                }

                std::reverse(path.begin(), path.end());
                for (auto &p : path)
                {
                    std::cout << " " << p;
                }

                std::cout << std::endl;
            }

            // Check for termination.
            if (InGoalSet(n))
            {
                found = true;
                break;
            }

            open_sets_[n->Index()].erase(n);

            // Extend a node.
            num_nodes_extended_++;
            Extend(n);

#if USE_NODE_REUSE

            if (n->ReusedFromClosedSet())
            {
                // Node is already in closed set.
                continue;
            }

#endif
            n->SetSearchID(search_id_);
            closed_sets_[n->Index()].insert(n);
        }

        if (!found)
        {
            std::cerr << "[ERROR] Search terminated without finding a valid result!" << std::endl;
            exit(1);
        }

        valid_result_found = true;

        if (kLazinessMap.at(laziness_mode_) == "LazySP")
        {
            // If using completely lazy, should check result plan.
            NodePtr tag = result_node;

            while (tag != nullptr)
            {
                bool local_path_valid = ValidPath(tag->LocalPath());

                if (!local_path_valid)
                {
                    valid_result_found = false;
                    break;
                }

                tag = tag->Parent();
            }

            if (!valid_result_found)
            {
                std::cout << "\rReplan: " << ++num_replan << std::flush;
            }
            else
            {
                std::cout << std::endl;
            }
        }
    }

    prev_graph_size_ = virtual_graph_size_;
    search_id_++;

    // Update global result.
    if (result_ == nullptr)
    {
        result_.reset(new Node(result_node));
    }
    else if (result_node->BetterThan(result_))
    {
        result_->DeepCopy(result_node);
    }

    // Retrieve full path.
    std::vector<Idx> path;
    auto tag = result_node;
    path.push_back(tag->Index());

    while (tag)
    {
        auto super_edge = tag->LocalPath();

        for (auto i = 1; i < super_edge.size(); ++i)
        {
            path.push_back(super_edge[i]);
        }

        tag = tag->Parent();
    }

    std::reverse(path.begin(), path.end());

    time_search_ += RelativeTime(start);

    return path;
}

void GraphSearch::InitDataStructures()
{
    // Prepare traceback map.
    map_.reset(new TracebackMap(virtual_graph_size_));

    for (SizeType i = 0; i < graph_->NumEdges(); ++i)
    {
        Inspection::EPtr e = graph_->Edge(i);

        // An edge should be in virtual graph and it should not be checked as invalid.
        if (e->in_virtual_graph && !(e->virtual_checked && !e->valid))
        {
            map_->AddDirectEdge(e->source, e->target, e->cost, e->IsEdgeBelongToRiskZone);
        }
    }

    // Reset data structures.
    queue_.reset(new PriorityQueue);
    open_sets_.resize(virtual_graph_size_);
    closed_sets_.resize(virtual_graph_size_);

#if USE_NODE_REUSE
    this->UpdateUnboundedNodes();
#else

    for (SizeType i = 0; i < virtual_graph_size_; ++i)
    {
        open_sets_[i].clear();
        closed_sets_[i].clear();
    }

#endif

    if (queue_->empty())
    {
        // Prepare source node.
        NodePtr source_node(new Node(graph_->Vertex(source_idx_)->index));
        source_node->SetVisSet(graph_->Vertex(source_idx_)->vis);
        source_node->SetChecked(true);
#if USE_GHOST_DATA
        source_node->SetGhostVisSet(graph_->Vertex(source_idx_)->vis);
#endif
        source_node->SetTotalLocationError(totalLocationErrorDefault);
        source_node->SetExitRiskZone(exitRiskZoneDefault);
        source_node->SetCostToComeRiskZone(costToComeRiskZoneDefault);

        queue_->push(source_node);
        open_sets_[source_idx_].insert(source_node);
    }
}

NodePtr GraphSearch::PopFromPriorityQueue()
{
    // Pop from priority queue;
    auto node_to_pop = queue_->top();
    queue_->pop();

#if USE_NODE_REUSE

    if (node_to_pop->ReusedFromClosedSet())
    {
        // A reused node must be latest and not subsumed.
        return node_to_pop;
    }

#endif

    if (!node_to_pop->Latest())
    {
        // This is not the latest version of the node.
        return nullptr;
    }

    if (node_to_pop->IsSubsumed())
    {
        // This node is subsumed by another node.
        return nullptr;
    }

    if (kLazinessMap.at(laziness_mode_) == "LazyA* modified")
    {
        if (kSuccessorMap.at(successor_mode_) == "direct")
        {
            if (!node_to_pop->IsChecked())
            {
                if (!Valid(node_to_pop))
                {
                    // Invalid node is discarded.
                    open_sets_[node_to_pop->Index()].erase(node_to_pop);
                    return nullptr;
                }
            }
        }
        else
        {
            std::cerr << "[ERROR] Not implemented!" << std::endl;
            exit(1);
        }
    }
    else if (kLazinessMap.at(laziness_mode_) == "LazyA*")
    {
        if (!Valid(node_to_pop))
        {
            // Invalid node is discarded.
            open_sets_[node_to_pop->Index()].erase(node_to_pop);
#if SAVE_PREDECESSOR
            Idx index = node_to_pop->Index();
#endif

            for (auto n : node_to_pop->SubsumedNodes())
            {
#if SAVE_PREDECESSOR

                if (!map_->EdgeExists(index, n->Index()) || n->SearchID() < search_id_)
                {
                    continue;
                }

                NodePtr new_node(new Node());
                new_node->CopyAsChild(n);
                new_node->Extend(index, map_->EdgeCost(index, n->Index()), graph_->Vertex(index)->vis);
                new_node->SetLocalPath(std::vector<Idx>{index, n->Index()});

                AddNode(new_node);
#else
                n->SetSubsumed(false);
                RecursivelyAddNode(n);
#endif
            }

            node_to_pop->ClearSubsumeHistory();
            return nullptr;
        }
    }

    return node_to_pop;
}

void GraphSearch::Extend(NodePtr n)
{
    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "direct")
    {
        successors = map_->NeighboringSuccessors(n->Index());
    }
    else if (kSuccessorMap.at(successor_mode_) == "expanded")
    {
        successors = map_->Successors(n->Index(), n->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet")
    {
        successors = map_->FirstMeetSuccessors(n->Index(), n->VisSet(), graph_);
    }

    for (const auto &s : successors)
    {
#if USE_NODE_REUSE

        if (n->ReusedFromClosedSet() && !NewNodesInvolved(n, s))
        {
            continue;
        }

#endif

        Idx v = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(n);
        auto cost = map_->Cost(v, n->Index());
#if UAV_NAVIGATION_ERROR
        vis.Clear();
        // if (v < 99 && n->Index() < 99)

        // auto parentPosition = graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Position();
        // auto candidateVertexPos = graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Position();
        // Vec3 direction = (candidateVertexPos - parentPosition);
        // auto normVec = direction.norm();
        // direction.normalize();
        // Vec3 temp_originalPos;
        // RealNum epsilon = 0.1;
        // temp_originalPos[0] = candidateVertexPos[0] + direction[0] * epsilon;
        // temp_originalPos[1] = candidateVertexPos[1] + direction[1] * epsilon;
        // temp_originalPos[2] = candidateVertexPos[2] + direction[2] * epsilon;
        // // bool IsInRiskZone = map_->IsTargetEdgeInRiskZone(n->Index(), v);
        // bool IsInRiskZone = planner->IsPointInsideBox(temp_originalPos, LowerBordersXYZ, UpperBordersXYZ);

        if (1) //(IsInRiskZone)
        {

            if (!ReCalculateVisibilitySetMC(n, v, new_node, cost)) //larger than max time in risk zone
            {
                continue;
            }
            // std::cout << "cost" << cost << std::endl;
        }
        // else
        // {
        //     vis.Insert(graph_->Vertex(v)->vis);
        //     // new_node->SetTotalLocationError(totalLocationErrorDefault);

        //     RealNormalDist NormR(0, 1);
        //     RealNormalDist NormAngle(0, 2 * M_PI);

        //     auto previousTotalLocationError = new_node->GetTotalLocationError();
        //     for (size_t i = 0; i < ba_x.size(); i++)
        //     {
        //         auto r = NormR(rng);
        //         auto azimuth = NormAngle(rng);
        //         auto elevation = NormAngle(rng);
        //         previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
        //         previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
        //         previousTotalLocationError[i][2] = r * sin(elevation);
        //         pos[0] = candidateVertexPos[0] + previousTotalLocationError[i][0];
        //         pos[1] = candidateVertexPos[1] + previousTotalLocationError[i][1];
        //         pos[2] = candidateVertexPos[2] + previousTotalLocationError[i][2];
        //     }
        //     new_node->SetTotalLocationError(previousTotalLocationError);

        //     new_node->SetExitRiskZone(exitRiskZoneDefault);
        //     new_node->SetCostToComeRiskZone(costToComeRiskZoneDefault);
        // }

#endif
        // std::cout << "v: " << graph_->Vertex(v)->vis.Size() << std::endl;
        // new_node->Extend(v, cost, graph_->Vertex(v)->vis);
        new_node->Extend(v, cost, vis);

        new_node->SetLocalPath(s);
        num_nodes_generated_++;

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(v, new_node->VisSet(), graph_,
                                                      virtual_graph_coverage_));
#endif

        if (AddNode(new_node))
        {
            num_nodes_remained_++;
        }
    }

    n->SetExtendedGraphSize(virtual_graph_size_);

    //     if (kLazinessMap.at(laziness_mode_) == "LazyA* modified") {
    // #if KEEP_SUBSUMING_HISTORY
    //                 ComputeAndAddSuccessorsCompleteLazy(n);
    // #else
    //                 ComputeAndAddSuccessors(n);
    // #endif
    //             }
    //             else {
    //                 ComputeAndAddSuccessorsCompleteLazy(n);
    //             }
}
bool GraphSearch::ReCalculateVisibilitySetMC(NodePtr n, Idx v, NodePtr new_node, RealNum &cost)
{
    //planner->ComputeVisibilitySet(graph_->Vertex(v));
    // auto currentTimeRiskZone = cost; //map_->CostRiskZone(m, parent->Index());
    // auto perviousTimeRiskZone = n->GetCostToComeRiskZone();
    // new_node->SetCostToComeRiskZone(currentTimeRiskZone + perviousTimeRiskZone);

    Idx MonteCarloNum = ba_x.size();

    if (MonteCarloNum < 0.5)
    {
        vis.Insert(graph_->Vertex(v)->vis);
        // new_node->SetCostToComeRiskZone(0.0);
        // new_node->SetTotalLocationError(totalLocationErrorDefault);
        // source_node->SetExitRiskZone(exitRiskZoneDefault);

        return true;
    }
    // std::cout << "aaaaaaaaaaaaa" << std::endl;
    // if (v > 99) ///todo to not see POI outside of risk zone
    // {
    //     vis.Insert(graph_->Vertex(v)->vis);
    //     new_node->SetCostToComeRiskZone(0.0);
    //     new_node->SetTotalLocationError(totalLocationErrorDefault);
    //     source_node->SetExitRiskZone(exitRiskZoneDefault);

    //     return true;
    // }
    // auto parentPosition = graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Position();
    // auto candidateVertexPos = graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Position();
    // Vec3 direction = (candidateVertexPos - parentPosition);
    // auto normVec = direction.norm();
    // direction.normalize();
    // Vec3 temp_originalPos;
    // RealNum epsilon = 0.1;
    // temp_originalPos[0] = candidateVertexPos[0] + direction[0] * epsilon;
    // temp_originalPos[1] = candidateVertexPos[1] + direction[1] * epsilon;
    // temp_originalPos[2] = candidateVertexPos[2] + direction[2] * epsilon;

    // if (!planner->IsPointInsideBox(temp_originalPos, LowerBordersXYZ, UpperBordersXYZ))

    // {
    //     vis.Insert(graph_->Vertex(v)->vis);
    //     new_node->SetCostToComeRiskZone(0.0);
    //     new_node->SetTotalLocationError(totalLocationErrorDefault);
    // source_node->SetExitRiskZone(exitRiskZoneDefault);

    //     return true;
    // }

    // auto exitRiskZone = false;
    RealNum x_error = 0, y_error = 0, z_error = 0, totalCost = 0, currentTimeRiskZone = 0, perviousTimeRiskZone = 0, currentCost = 0;

    Vec3 pos;
    auto parentPosition = graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Position();
    auto candidateVertexPos = graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Position();
    //     const ob::State *sourcede = graph_->Vertex(n->Index())->state;
    //     const ob::State *targetede = graph_->Vertex(v)->state;

    // auto testdebug = space_info_->distance(sourcede, targetede);
    //     std::cout << "dddddd " << testdebug << std::endl;
    // Vec3 direction = (candidateVertexPos - parentPosition);
    // auto normVec = direction.norm();
    // direction.normalize();
    // Vec3 temp_originalPos;
    // RealNum epsilon = 0.1;
    // temp_originalPos[0] = parentPosition[0] + direction[0] * normVec / 2;
    // temp_originalPos[1] = parentPosition[1] + direction[1] * normVec / 2;
    // temp_originalPos[2] = parentPosition[2] + direction[2] * normVec / 2;
    // bool IsInRiskZone = map_->IsTargetEdgeInRiskZone(n->Index(), v);

    auto previousTotalLocationError = new_node->GetTotalLocationError();
    auto perviousExitRiskZone = new_node->GetExitRiskZone();
    auto perviousCostRiskZone = new_node->GetCostToComeRiskZone();
    // space_info_->copyState(vertex->state, graph_->Vertex(v)->state);
    // space_info_->copyState(parentVertex->state, graph_->Vertex(n->Index())->state);

    vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Yaw());
    vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->CameraAngle());
    parentVertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Yaw());
    parentVertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->CameraAngle());

    // RealNum theta = graph_->Vertex(m)->state->as<DroneStateSpace::StateType>()->CameraAngle();
    // RealNum psi = graph_->Vertex(m)->state->as<DroneStateSpace::StateType>()->Yaw();

    // auto x = (candidateVertexPos[0]) - parentPosition[0];
    // auto y = (candidateVertexPos[1]) - parentPosition[1];
    // auto z = (candidateVertexPos[2]) - parentPosition[2];
    // //todo psi and theta
    // RealNum psi = atan2(y, x);
    // RealNum theta = atan2(z, sqrt(x * x + y * y));
    // auto counterExitRiskZone = 0;

    for (size_t i = 0; i < ba_x.size(); i++)
    {

        ///for cost calculation
        parentPosition[0] += previousTotalLocationError[i][0];
        parentPosition[1] += previousTotalLocationError[i][1];
        parentPosition[2] += previousTotalLocationError[i][2];
        bool IsParentInRiskZone = planner->IsPointInsideBox(parentPosition);

        if (perviousExitRiskZone[i]) //case that parent is outside riskZone only because drift of sensors
        {
            //todo assuming that the gps will fix the position outside of the risk zone
            RealNormalDist NormR(0, 1);
            RealNormalDist NormAngle(0, 2 * M_PI);
            auto r = NormR(rng);
            auto azimuth = NormAngle(rng);
            auto elevation = NormAngle(rng);

            previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
            previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
            previousTotalLocationError[i][2] = r * sin(elevation);
            perviousCostRiskZone[i] = 0.0;
            currentTimeRiskZone = 0.0;
        }
        //todo this is approximation of the candidateVertexPos
        candidateVertexPos[0] += previousTotalLocationError[i][0];
        candidateVertexPos[1] += previousTotalLocationError[i][1];
        candidateVertexPos[2] += previousTotalLocationError[i][2];

        //start calculate currentTimeRiskZone
        bool IsCandidateInRiskZone = planner->IsPointInsideBox(candidateVertexPos);

        // if (perviousExitRiskZone[i] || (!IsParentInRiskZone))
        // {
        //     perviousExitRiskZone[i] = false;

        //     RealNormalDist NormR(0, 1);
        //     RealNormalDist NormAngle(0, 2 * M_PI);
        //     auto r = NormR(rng);
        //     auto azimuth = NormAngle(rng);
        //     auto elevation = NormAngle(rng);

        //     previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
        //     previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
        //     previousTotalLocationError[i][2] = r * sin(elevation);
        //     perviousCostRiskZone[i] = 0.0;
        // }
        Vec3 InsertPoint;
        Vec3 VertexPointInRiskZone;
        auto needToConsiderRiskZone = false;
        auto needToUpdateOnlyTheCost = false;
        if (IsCandidateInRiskZone)
        {
            if (IsParentInRiskZone) //type 5
            {
                InsertPoint[0] = parentPosition[0];
                InsertPoint[1] = parentPosition[1];
                InsertPoint[2] = parentPosition[2];
                VertexPointInRiskZone[0] = candidateVertexPos[0];
                VertexPointInRiskZone[1] = candidateVertexPos[1];
                VertexPointInRiskZone[2] = candidateVertexPos[2];
                needToConsiderRiskZone = true;
            }
            else //find the entry point
            {
                //type 4
                auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPosition, candidateVertexPos, InsertPoint);

                VertexPointInRiskZone[0] = candidateVertexPos[0];
                VertexPointInRiskZone[1] = candidateVertexPos[1];
                VertexPointInRiskZone[2] = candidateVertexPos[2];
                needToConsiderRiskZone = true;
            }
        }
        else // todo:: assuming that the GNSS fix the position but we need to update only the cost (but maybe there is a case that because of the error the candidate is in riskZone??)
        {
            // needToUpdateOnlyTheCost = true;

            // if (IsParentInRiskZone) //type 3 - find the exit Point
            // {
            //     auto IsExitToRiskZone = planner->FindInsertPointRiskZone(parentPosition, candidateVertexPos, VertexPointInRiskZone);
            // }
            // else
            // {
            //     auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPosition, candidateVertexPos, InsertPoint);
            //     if (!IsInsertToRiskZone) //type 1
            //     {
            //     }
            //     else
            //     {
            //     }
            // }

            RealNormalDist NormR(0, 1);
            RealNormalDist NormAngle(0, 2 * M_PI);
            auto r = NormR(rng);
            auto azimuth = NormAngle(rng);
            auto elevation = NormAngle(rng);

            previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
            previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
            previousTotalLocationError[i][2] = r * sin(elevation);
            perviousCostRiskZone[i] = 0.0;
            currentTimeRiskZone = 0.0;
        }

        const ob::State *target = vertex->state;
        if (needToConsiderRiskZone)
        {
            parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(InsertPoint);
            const ob::State *source = parentVertex->state;

            vertex->state->as<DroneStateSpace::StateType>()->SetPosition(VertexPointInRiskZone);
            const ob::State *targetInitial = vertex->state;

            currentTimeRiskZone = space_info_->distance(source, targetInitial);
            perviousTimeRiskZone = perviousCostRiskZone[i];

            if ((currentTimeRiskZone + perviousTimeRiskZone) > maxTimeAllowInRistZone)
            {
                // std::cout << "accumulatedCostRiskZone " << CostRiskZone+accumulatedCostRiskZone;
                // std::cout << ","<< new_node->CostToComeRiskZone()  << std::endl;
                //vis.Insert(graph_->Vertex(v)->vis);
                return false;
            }

            // if ((currentTimeRiskZone + perviousTimeRiskZone) > minTimeAllowInRistZone)
            // {
            //     auto milli_g2mpss = 9.81 / 1e3;                    //   Conversion from [mili g ] to [m/s^2]
            //     auto degPerHr2radPerSec = (3.14 / 180.0) / 3600.0; //  Conversion from [deg/hr] to [rad/s]
            //     auto b_a = b_a_milli_g * milli_g2mpss;
            //     auto b_g = b_g_degPerHr * degPerHr2radPerSec;
            //     auto costRiskZone = LocationErrorFunc(b_a, b_g, new_node->CostToComeRiskZone());
            //     auto MinTimeCostRiskZone = LocationErrorFunc(b_a, b_g, minTimeAllowInRistZone);
            //     if (costRiskZone > MinTimeCostRiskZone)
            //     {
            //         cost = cost + multipleCostFunction * (costRiskZone - MinTimeCostRiskZone); // 1000000;
            //         // std::cout << "cost" << cost << std::endl;
            //     }
            // }
            auto x = (VertexPointInRiskZone[0] + previousTotalLocationError[i][0]) - InsertPoint[0];
            auto y = (VertexPointInRiskZone[1] + previousTotalLocationError[i][1]) - InsertPoint[1];
            auto z = (VertexPointInRiskZone[2] + previousTotalLocationError[i][2]) - InsertPoint[2];
            //todo psi and theta
            RealNum psi = atan2(y, x);
            RealNum theta = atan2(z, sqrt(x * x + y * y));
            auto temp_x_error = ba_x[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
            temp_x_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_z[i] + bg_y[i]);

            auto temp_y_error = ba_y[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
            temp_y_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (bg_z[i] - bg_x[i]);

            auto temp_z_error = ba_z[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
            temp_z_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_y[i] + bg_x[i]);

            previousTotalLocationError[i][0] += cos(theta) * cos(psi) * temp_x_error - sin(psi) * temp_y_error + sin(theta) * cos(psi) * temp_z_error;
            previousTotalLocationError[i][1] += cos(theta) * sin(psi) * temp_x_error + cos(psi) * temp_y_error + sin(theta) * sin(psi) * temp_z_error;
            previousTotalLocationError[i][2] += -sin(theta) * temp_x_error + cos(theta) * temp_z_error;

            x_error += previousTotalLocationError[i][0];
            y_error += previousTotalLocationError[i][1];
            z_error += previousTotalLocationError[i][2];

            pos[0] = VertexPointInRiskZone[0] + previousTotalLocationError[i][0];
            pos[1] = VertexPointInRiskZone[1] + previousTotalLocationError[i][1];
            pos[2] = VertexPointInRiskZone[2] + previousTotalLocationError[i][2];
            if (!planner->IsPointInsideBox(pos))
            {
                perviousExitRiskZone[i] = true;
            }
            vertex->state->as<DroneStateSpace::StateType>()->SetPosition(pos);
            //recalculate cost

            currentCost = space_info_->distance(source, target);
            perviousCostRiskZone[i] += currentCost;

            totalCost += currentCost;
        }
        else
        {
            vertex->state->as<DroneStateSpace::StateType>()->SetPosition(candidateVertexPos);
            totalCost += cost;
        }

        //recalculate POI
        planner->ComputeVisibilitySet(vertex);

        for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
        {
            //todo make bitset_ private
            if (virtual_graph_coverage_.bitset_[j] > 0.5 && vertex->vis.bitset_[j] > 0.5)
            {
                vis.bitset_[j] += 1;

                // std::cout << " " << j;
            }
        }
        // std::cout << " " << std::endl;
    }
    // std::cout << "aaa " << currentCost << std::endl;

    // std::cout << "000 " << cost << std::endl;
    cost = 1.0 * totalCost / MonteCarloNum;

    // std::cout << "bbb " << cost << std::endl;
    // if (exitRiskZone)
    // {
    //     new_node->SetCostToComeRiskZone(0.0);
    //     new_node->SetTotalLocationError(totalLocationErrorDefault);
    //     new_node->SetExitRiskZone(exitRiskZoneDefault);s
    // }
    // else
    // {

    //     new_node->SetTotalLocationError(previousTotalLocationError);
    //     new_node->SetExitRiskZone(perviousExitRiskZone);
    // }
    new_node->SetTotalLocationError(previousTotalLocationError);
    new_node->SetExitRiskZone(perviousExitRiskZone);
    new_node->SetCostToComeRiskZone(perviousCostRiskZone);

    for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
    {
        // if (virtual_graph_coverage_.visPOI[j] > 0.5 && graph_->Vertex(m)->vis.visPOI[j] > 0.5)
        // {
        //     x_error /= MonteCarloNum;
        //     y_error /= MonteCarloNum;
        //     z_error /= MonteCarloNum;
        //     auto TotalError = sqrt(x_error * x_error + y_error * y_error + z_error * z_error);
        //     auto p = 1 - TotalError * 0.1;
        //     vis.visPOI[j] = 1 - (1 - p) * (1 - new_node->VisSet().visPOI[j]);

        //     if (vis.visPOI[j] > 0.99)
        //     {
        //         vis.visPOI[j] = 1;
        //     }
        //     if (vis.visPOI[j] < 0)
        //     {
        //         vis.visPOI[j] = 0;
        //     }
        // }
        if (vis.bitset_[j] > 0.5)
        {
            RealNum p = 0.0;
            if (MonteCarloNum > 0.5)
            {
                p = 1.0 * vis.bitset_[j] / MonteCarloNum;
            }
            else
            {
                p = 1;
            }

            vis.bitset_[j] = 1 - (1 - p) * (1 - new_node->VisSet().bitset_[j]);
            if (vis.bitset_[j] > 0.99)
            {
                vis.bitset_[j] = 1;
            }
            if (vis.bitset_[j] < 0)
            {
                vis.bitset_[j] = 0;
            }
        }
    }

    return true;
}
RealNum GraphSearch::LocationErrorFunc(const RealNum b_a, const RealNum b_g, const RealNum timeRiskZone) const
{
    auto g = 9.81;
    auto LocationError = ((1.0 / 2.0) * b_a * timeRiskZone * timeRiskZone + (1.0 / 6.0) * g * b_g * timeRiskZone * timeRiskZone * timeRiskZone);
    return LocationError;
}
bool GraphSearch::AddNode(NodePtr n, const bool skip_queue_operations)
{
    if (DominatedByClosedState(n))
    {
        return false;
    }

    if (SubsumedByOpenState(n, skip_queue_operations))
    {
        return false;
    }

    open_sets_[n->Index()].insert(n);

    if (!skip_queue_operations)
    {
        queue_->push(n);
    }

    return true;
}

void GraphSearch::RecursivelyAddNode(NodePtr n, const bool skip_queue_operations)
{
    if (!map_->EdgeExists(n->Parent()->Index(), n->Index()) || n->Parent()->SearchID() < search_id_)
    {
        for (auto s : n->SubsumedNodes())
        {
            s->SetSubsumed(false);
            RecursivelyAddNode(s, skip_queue_operations);
        }
    }
    else
    {
        AddNode(n, skip_queue_operations);
    }
}

bool GraphSearch::SubsumedByOpenState(NodePtr node, const bool skip_queue_operations)
{
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();)
    {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet()))
        {
            if (!s->IsChecked())
            {
                if (!Valid(s))
                {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            return true;
        }

#endif

        if (Dominates(s, node))
        {
            if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && !s->IsChecked())
            {
                if (!Valid(s))
                {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            bool need_update_queue = false;

            if (!skip_queue_operations)
            {
#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY
                //todo Contains
                // if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet()))
                if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Size() > (node->GhostVisSet().Size()))

                {
                    need_update_queue = true;
                }

#else

                if (!s->GhostVisSet().Contains(node->GhostVisSet()))
                {
                    need_update_queue = true;
                }

#endif

#endif
            }

            if (need_update_queue)
            {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else
            {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet()))
        {
            if (!node->IsChecked())
            {
                if (!Valid(node))
                {
                    return true;
                }
            }

            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s))
        {
            if (kLazinessMap.at(laziness_mode_) == "LazyA* modified" && !node->IsChecked())
            {
                if (!Valid(node))
                {
                    return true;
                }
            }

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::Valid(NodePtr n)
{
    bool local_path_valid = ValidPath(n->LocalPath());
    n->SetChecked(true);
    n->SetValid(local_path_valid);

    return local_path_valid;
}

NodePtr GraphSearch::PopFromQueue()
{
    // pop a parent node
    auto node_to_pop = queue_->top();
    queue_->pop();

#if DEBUG_MODE
    std::cout << "Node popped: ";
    PrintNodeStatus(node_to_pop);
    std::cout << std::endl;
    getchar();
#endif

    if (!node_to_pop->Latest())
    {
        // This node has been updated to a separate node.
        return nullptr;
    }

    // if this node is not subsumed by other nodes
    if (!node_to_pop->IsSubsumed())
    {
        if (IsUpToDate(node_to_pop))
        {
            if (!node_to_pop->IsChecked())
            {
                bool local_path_valid = ValidPath(node_to_pop->LocalPath());
                node_to_pop->SetChecked(true);
                node_to_pop->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    ComputeAndAddSuccessors(node_to_pop->Parent());
                    return nullptr;
                }
            }

            // reset to 0, later match to its successors
            node_to_pop->ResetSuccessorStatusID();
            return node_to_pop;
        }
    }

    return nullptr;
}

NodePtr GraphSearch::PopFromQueueCompleteLazy()
{
    // pop a parent node
    auto node_to_pop = queue_->top();
    queue_->pop();

#if DEBUG_MODE
    std::cout << "Node popped: ";
    PrintNodeStatus(node_to_pop);
    std::cout << std::endl;
    getchar();
#endif

#if USE_NODE_REUSE

    if (node_to_pop->ReusedFromClosedSet())
    {
        return node_to_pop;
    }

#endif

    if (!node_to_pop->Latest())
    {
        // This node has been updated to a separate node.
        return nullptr;
    }

    // if this node is not subsumed by other nodes
    if (!node_to_pop->IsSubsumed())
    {
        return node_to_pop;
    }

    return nullptr;
}

bool GraphSearch::IsUpToDate(const NodePtr p) const
{
    if (p->Parent() == nullptr)
    {
        return true;
    }

    if (p->IsChecked() && p->IsValid())
    {
        return true;
    }

    auto parent_id = p->Parent()->SuccessorStatusID();
    auto self_id = p->SuccessorStatusID();

    return (self_id == parent_id);
}

void GraphSearch::ComputeAndAddSuccessors(const NodePtr parent)
{
#if DEBUG_MODE
    std::cout << "Pre: ";
    PrintOpenSets();
    Idx i = CheckOpenSets();

    if (i > 0)
    {
        std::cout << "Pre: Open set is invalid at " << i << std::endl;
        getchar();
    }

    getchar();
#endif

    parent->IncreaseSuccessorStatusID();
    Idx id = parent->SuccessorStatusID();

    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "expanded")
    {
        successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet")
    {
        successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "direct")
    {
        successors = map_->NeighboringSuccessors(parent->Index());
    }

    for (const auto &s : successors)
    {
        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);
        new_node->SetSuccessorStaturID(id);
        num_nodes_generated_++;

        if (DominatedByClosedState(new_node))
        {
            continue;
        }

        if (kSuccessorMap.at(successor_mode_) != "direct")
        {
            if (DominatedByOpenState(new_node))
            {
                if (parent->SuccessorStatusID() != id)
                {
                    break;
                }

                continue;
            }

            if (parent->SuccessorStatusID() != id)
            {
                break;
            }
        }
        else
        {
            // Do not need to recompute successors.
            if (DominatedByOpenState2(new_node))
            {
                continue;
            }
        }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_,
                                                      virtual_graph_coverage_));
#endif

        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
        num_nodes_remained_++;
    }

#if DEBUG_MODE
    std::cout << "Post: ";
    PrintOpenSets();
    i = CheckOpenSets();

    if (i > 0)
    {
        std::cout << "Post: Open set is invalid at " << i << std::endl;
        getchar();
    }

    getchar();
#endif
}

bool GraphSearch::NewNodesInvolved(const NodePtr parent, const std::vector<Idx> &successor) const
{
    for (const auto &i : successor)
    {
        if (i >= parent->ExtendedGraphSize())
        {
            return true;
        }
    }

    return false;
}

void GraphSearch::ComputeAndAddSuccessorsCompleteLazy(const NodePtr parent)
{
#if DEBUG_MODE
    std::cout << "Pre: ";
    PrintOpenSets();
    getchar();
#endif

    std::vector<std::vector<Idx>> successors;

    if (kSuccessorMap.at(successor_mode_) == "expanded")
    {
        successors = map_->Successors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "first-meet")
    {
        successors = map_->FirstMeetSuccessors(parent->Index(), parent->VisSet(), graph_);
    }
    else if (kSuccessorMap.at(successor_mode_) == "direct")
    {
        successors = map_->NeighboringSuccessors(parent->Index());
    }

    // std::cout << parent->Index() << ", " << parent->ReusedFromClosedSet() << std::endl;
    // for (const auto& s : successors) {
    //     std::cout << s[0] << " ";
    // }
    // std::cout << std::endl;
    // getchar();

    // PrintClosedSets();
    // PrintOpenSets();
    // getchar();

#if USE_NODE_REUSE
    const bool &add_only_new_successors = parent->ReusedFromClosedSet();
#endif

    for (const auto &s : successors)
    {
#if USE_NODE_REUSE

        if (add_only_new_successors && !NewNodesInvolved(parent, s))
        {
            continue;
        }

#endif

        Idx m = s[0];
        NodePtr new_node(new Node());
        new_node->CopyAsChild(parent);
        new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
        new_node->SetLocalPath(s);
        num_nodes_generated_++;

        if (DominatedByClosedState(new_node))
        {
            // std::cout << "Dominated by closed states" << std::endl;
            continue;
        }

        if (DominatedByOpenStateCompleteLazy(new_node))
        {
            // std::cout << "Dominated by open states" << std::endl;
            continue;
        }

#if USE_HEURISTIC
        new_node->SetHeuristic(map_->ComputeHeuristic(m, new_node->VisSet(), graph_,
                                                      virtual_graph_coverage_));
#endif
        // std::cout << "Added to open set" << std::endl;

        open_sets_[new_node->Index()].insert(new_node);
        queue_->push(new_node);
        num_nodes_remained_++;
    }

    parent->SetExtendedGraphSize(virtual_graph_size_);

#if DEBUG_MODE
    std::cout << "Post: ";
    PrintOpenSets();
    getchar();
#endif
}

NodePtr GraphSearch::ComputeNearestSuccessor(const NodePtr parent)
{
    auto successor = map_->NearestSuccessor(parent->Index(), parent->VisSet(), graph_);

    Idx m = successor[0];
    NodePtr new_node(new Node());
    new_node->CopyAsChild(parent);
    new_node->Extend(m, map_->Cost(m, parent->Index()), graph_->Vertex(m)->vis);
    new_node->SetLocalPath(successor);

    return new_node;
}

void GraphSearch::PrintClosedSets() const
{
    for (auto i = 0; i < closed_sets_.size(); ++i)
    {
        std::cout << "Closed set " << i << " :" << std::endl;
        ClosedSet closed_set = *(closed_sets_.begin() + i);

        for (auto node : closed_set)
        {
            PrintNodeStatus(node);
        }

        std::cout << std::endl;
    }
}

void GraphSearch::PrintOpenSets() const
{
    unsigned i = 0;

    for (auto open_set : open_sets_)
    {
        std::cout << "Open set " << i << " :" << std::endl;

        for (auto node : open_set)
        {
            PrintNodeStatus(node);
        }

        std::cout << std::endl;
        ++i;
    }
}

void GraphSearch::PrintNodeStatus(const NodePtr node, std::ostream &out) const
{
    if (node == nullptr)
    {
        out << "Node is null.\t";
        return;
    }

    out << node->Index() << ", ";

    if (node->Parent())
    {
        out << node->Parent()->Index() << ", ";
    }
    else
    {
        out << "nullptr, ";
    }

    out << node->CostToCome() << ", "
        << node->CoverageSize() << ", "
        << node->GhostCost() << ", "
        << node->GhostCoverageSize() << ", "
        << node->IsSubsumed() << ", "
        << node->IsChecked() << ", "
        << node->IsValid() << ", "
        << node->Latest() << ", "
        << node->ReusedFromClosedSet() << ", "
        << node->SearchID() << ", "
        << IsUpToDate(node) << ", "
        << node->NumberOfSubsumed() << "; \t";
}

Idx GraphSearch::CheckOpenSets() const
{
    for (Idx it = 0; it < open_sets_.size(); ++it)
    {
        OpenSet set = *(open_sets_.begin() + it);

        for (auto i = set.begin(); i != set.end(); ++i)
        {
            for (auto j = i; j != set.end(); ++j)
            {
                if (i != j && Dominates(*i, *j))
                {
                    return it;
                }
            }

            NodePtr n = *i;

            if (n->NumberOfSubsumed() > 0)
            {
                if (!n->IsChecked())
                {
                    std::cout << "not checked" << std::endl;
                    getchar();
                    return it;
                }

                if (!n->IsValid())
                {
                    std::cout << "not valid" << std::endl;
                    getchar();
                    return it;
                }
            }
        }
    }

    return 0;
}

bool GraphSearch::InGoalSet(const NodePtr n) const
{
#if USE_GHOST_DATA
    // return (n->VisSet().Size() >= p_*virtual_graph_coverage_.Size());
    // return (n->GhostVisSet() == virtual_graph_coverage_);
    auto GhostSize = n->GhostVisSet().Size();
    auto virtualSize = virtual_graph_coverage_.Size();
    return (fabs(GhostSize - virtualSize) < 1e-3 || GhostSize > virtualSize);
#endif

    return (n->VisSet() == virtual_graph_coverage_);
}

bool GraphSearch::StronglyDominates(const RealNum &l1, const VisibilitySet &s1, const RealNum &l2,
                                    const VisibilitySet &s2) const
{
    if (l1 > l2)
    {
        return false;
    }

    if (!s1.Contains(s2))
    {
        return false;
    }

    return true;
}

bool GraphSearch::DominatedByClosedState(const NodePtr node) const
{
    auto states = closed_sets_[node->Index()];

    for (const auto s : states)
    {
#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet()))
        {
            // New state is completely dominated.
            return true;
        }

        if (this->StronglyDominates(s->GhostCost(), s->GhostVisSet(), node->GhostCost(),
                                    node->GhostVisSet()))
        {
            s->Subsume(node, true);
            return true;
        }

#else

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->CostToCome(), node->VisSet()))
        {
            return true;
        }

#endif
    }

    return false;
}

bool GraphSearch::DominatedByOpenState(const NodePtr node)
{
    // this is a copy
    Idx index = node->Index();
    auto states = open_sets_[index];

    // for (auto s : states) {
    for (auto it = states.begin(); it != states.end();)
    {
        NodePtr s = *it;

        // early return for duplicates
        if (node->Parent() == s->Parent() && fabs(node->CostToCome() - s->CostToCome()) < EPS)
        {
            s->SetSuccessorStaturID(node->SuccessorStatusID());
            return true;
        }

        // remove out of date nodes
        if (!IsUpToDate(s))
        {
            open_sets_[index].erase(it++);
            continue;
        }

        if (Dominates(s, node))
        {
            if (!s->IsChecked())
            {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    open_sets_[index].erase(it++);

                    ComputeAndAddSuccessors(s->Parent());
                    return DominatedByOpenState(node);
                }
            }

            bool need_update_queue = false;

#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY
            //todo Contains
            if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet()))
            // if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Size() > (node->GhostVisSet().Size()))

            {
                need_update_queue = true;
            }

#else

            if (!s->GhostVisSet().Contains(node->GhostVisSet()))
            {
                need_update_queue = true;
            }

#endif

#endif

            if (need_update_queue)
            {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else
            {
                s->Subsume(node);
            }

            return true;
        }

        if (Dominates(node, s))
        {
            if (!node->IsChecked())
            {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    ComputeAndAddSuccessors(node->Parent());
                    return true;
                }
            }

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::DominatedByOpenState2(const NodePtr node)
{
    // this is a copy
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();)
    {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet()))
        {
            if (!s->IsChecked())
            {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            return true;
        }

#endif

        if (Dominates(s, node))
        {
            if (!s->IsChecked())
            {
                bool local_path_valid = ValidPath(s->LocalPath());
                s->SetChecked(true);
                s->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    open_sets_[index].erase(it++);
                    continue;
                }
            }

            bool need_update_queue = false;

#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY
            //todo Contains
            if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet()))
            // if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Size() > (node->GhostVisSet().Size()))

            {
                need_update_queue = true;
            }

#else

            if (!s->GhostVisSet().Contains(node->GhostVisSet()))
            {
                need_update_queue = true;
            }

#endif

#endif

            if (need_update_queue)
            {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else
            {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet()))
        {
            if (!node->IsChecked())
            {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    return true;
                }
            }

            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s))
        {
            if (!node->IsChecked())
            {
                bool local_path_valid = ValidPath(node->LocalPath());
                node->SetChecked(true);
                node->SetValid(local_path_valid);

                if (!local_path_valid)
                {
                    return true;
                }
            }

            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::DominatedByOpenStateCompleteLazy(const NodePtr node,
                                                   const bool skip_queue_operations)
{
    // this is a copy
    auto index = node->Index();

    for (auto it = open_sets_[index].begin(); it != open_sets_[index].end();)
    {
        NodePtr s = *it;

#if USE_GHOST_DATA

        if (this->StronglyDominates(s->CostToCome(), s->VisSet(), node->GhostCost(), node->GhostVisSet()))
        {
            return true;
        }

#endif

        if (Dominates(s, node))
        {
            bool need_update_queue = false;

            if (!skip_queue_operations)
            {
#if USE_GHOST_DATA

#if USE_GHOST_COST_AS_KEY
                //todo Contains
                // if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet()))
                if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Size() > (node->GhostVisSet().Size()))
                {
                    need_update_queue = true;
                }

#else

                if (!s->GhostVisSet().Contains(node->GhostVisSet()))
                {
                    need_update_queue = true;
                }

#endif

#endif
            }

            if (need_update_queue)
            {
                NodePtr updated(new Node(s));
                s->SetLatest(false);
                updated->Subsume(node);
                open_sets_[index].erase(s);
                open_sets_[index].insert(updated);
                queue_->push(updated);
            }
            else
            {
                s->Subsume(node);
            }

            return true;
        }

#if USE_GHOST_DATA

        if (this->StronglyDominates(node->CostToCome(), node->VisSet(), s->GhostCost(), s->GhostVisSet()))
        {
            s->SetSubsumed(true);
            open_sets_[index].erase(it++);
            continue;
        }

#endif

        if (Dominates(node, s))
        {
            node->Subsume(s);
            open_sets_[index].erase(it++);
            continue;
        }

        ++it;
    }

    return false;
}

bool GraphSearch::Dominates(const NodePtr n1, const NodePtr n2) const
{
#if USE_GHOST_DATA
    VisibilitySet union_set = n2->GhostVisSet();
    union_set.Insert(n1->GhostVisSet());

    if ((1 + eps_) * (n2->GhostCost()) < n1->CostToCome())
    {
        return false;
    }
    //todo smaller with epsilon
    if (union_set.Size() * p_ > n1->CoverageSize())
    {
        return false;
    }

    return true;
#else
    return this->StronglyDominates(n1->CostToCome(), n1->VisSet(), n2->CostToCome(), n2->VisSet());
#endif
}

bool GraphSearch::ValidPath(const std::vector<Idx> &path)
{
    SizeType len = path.size();

    if (len > 0)
    {
        for (auto i = len - 1; i > 0; --i)
        {
            Inspection::EPtr edge = graph_->FindEdge(path[i - 1], path[i]);

            // for an up to date successor, there is no invalid but checked edge
            if (!edge->virtual_checked)
            {
                time_valid_ += edge->time_forward_kinematics;
                time_valid_ += edge->time_collision_detection;
                num_validated_++;
                edge->virtual_checked = true;

                if (!edge->valid)
                {
                    map_->RemoveDirectEdge(edge->source, edge->target);
                    return false;
                }
            }
        }
    }

    return true;
}

SizeType GraphSearch::VirtualGraphCoverageSize() const
{
    return virtual_graph_coverage_.Size();
}

const VisibilitySet &GraphSearch::VirtualGraphCoverage() const
{
    return virtual_graph_coverage_;
}

SizeType GraphSearch::ResultCoverageSize() const
{
    if (result_ == nullptr)
    {
        return 0;
    }

    return result_->VisSet().Size();
}

RealNum GraphSearch::ResultCost() const
{
    return result_->CostToCome();
}

void GraphSearch::PrintTitle(std::ostream &out) const
{
    out << "GraphSize "
        << "GraphCoverage "
        << "P "
        << "EPS "
        << "Coverage "
        << "Cost "
        << "CoverRatio "
        << "VisTime "
        << "BuildTime "
        << "ValidationTime "
        << "SearchTime "
        << "TotalTime "
        << "NumEdgeValidated "
        << "NumEdgesOnGraph "
        << "NumNodesExtended "
        << "NumNodesGenerated "
        << "NumNodesRemained "
        << std::endl;
}

void GraphSearch::PrintResult(std::ostream &out) const
{
    out << virtual_graph_size_ << " "
        << virtual_graph_coverage_.Size() << " "
        << p_ << " "
        << eps_ << " "
        << ResultCoverageSize() << " "
        << ResultCost() << " "
        << ResultCoverageSize() / (RealNum)MAX_COVERAGE_SIZE << " "
        << time_vis_ << " "
        << time_build_ << " "
        << time_valid_ << " "
        << time_search_ << " "
        << time_vis_ + time_build_ + time_valid_ + time_search_ << " "
        << num_validated_ << " "
        << virtual_graph_num_edges_ << " "
        << num_nodes_extended_ << " "
        << num_nodes_generated_ << " "
        << num_nodes_remained_ << " "
        << std::endl;
}

SizeType GraphSearch::RelativeTime(const TimePoint start) const
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

SizeType GraphSearch::TotalTime() const
{
    return time_vis_ + time_build_ + time_valid_ + time_search_;
}

void GraphSearch::SetMaxTimeAllowed(const SizeType &time)
{
    max_time_allowed_ = time;
}

bool GraphSearch::CheckTermination() const
{
    if (TotalTime() > max_time_allowed_)
    {
        std::cout << "Exceeding maximum time allowed!" << std::endl;
        PrintResult(std::cout);
        return true;
    }

    return false;
}

SizeType GraphSearch::VirtualGraphNumEdges() const
{
    return virtual_graph_num_edges_;
}

void GraphSearch::ReconstructNode(const NodePtr node) const
{
    auto idx = node->Index();
    node->CopyAsChild(node->Parent());
    node->Extend(idx, node->LocalPathCost(), graph_->Vertex(idx)->vis);
}

void GraphSearch::TraceFirstUnboundedNode(const NodePtr node, std::queue<NodePtr> &recycle_bin)
{
    NodePtr tag = node;
    std::stack<NodePtr> stack;

    while (tag != nullptr && tag->SearchID() < search_id_)
    {
        stack.push(tag);
        tag = tag->Parent();
    }

    bool found_first_unbounded = true;

    if (tag == nullptr || tag->ReusedFromClosedSet())
    {
        found_first_unbounded = false;
    }

    while (!stack.empty())
    {
        tag = stack.top();
        stack.pop();

        tag->SetSearchID(search_id_);

        if (!found_first_unbounded)
        {
            if (!tag->IsBounded(p_, eps_))
            {
                found_first_unbounded = true;
            }
        }

        if (found_first_unbounded)
        {
            tag->SetReuseFromClosedSet(false);
        }
        else
        {
            tag->SetReuseFromClosedSet(true);
        }
    }
}

void GraphSearch::RecycleSubsumedNodes(NodePtr node, std::queue<NodePtr> &recycle_bin,
                                       const bool force_recycle_all)
{
    auto tmp_history = node->SubsumedNodes();
    node->ClearSubsumeHistory();

#if SAVE_PREDECESSOR
    Idx index = node->Index();
#endif

    for (auto n : tmp_history)
    {
#if SAVE_PREDECESSOR

        if (!map_->EdgeExists(index, n->Index()) || n->SearchID() < search_id_ || !n->ReusedFromClosedSet())
        {
            continue;
        }

        NodePtr new_node(new Node());
        new_node->CopyAsChild(n);
        new_node->Extend(index, map_->EdgeCost(index, n->Index()), graph_->Vertex(index)->vis);
        new_node->SetLocalPath(std::vector<Idx>{index, n->Index()});

        if (!force_recycle_all && Dominates(node, new_node))
        {
            node->Subsume(new_node);
        }
        else
        {
            new_node->SetSubsumed(false);
            recycle_bin.push(new_node);
        }

#else

        if (!force_recycle_all && n->Parent()->SearchID() == search_id_ && n->Parent()->ReusedFromClosedSet() && map_->EdgeExists(n->Parent()->Index(), n->Index()) && Dominates(node, n))
        {
            node->Subsume(n);
        }
        else
        {
            n->SetSubsumed(false);
            recycle_bin.push(n);
        }

#endif
    }
}

void GraphSearch::UpdateUnboundedNodes()
{
    std::queue<NodePtr> recycle_bin;

    // Parse closed set.
    for (SizeType i = 0; i < virtual_graph_size_; ++i)
    {
        auto &closed_set = closed_sets_[i];

        for (auto it = closed_set.begin(); it != closed_set.end(); ++it)
        {
            auto node = *it;
            this->TraceFirstUnboundedNode(node, recycle_bin);
        }
    }

    std::cout << "Parsed closed set" << std::flush;

    // Find boundary nodes.
    std::unordered_set<NodePtr> reopened_nodes;
    // std::set<NodePtr, CoverageCmp> reopened_nodes;
    for (SizeType i = 0; i < virtual_graph_size_; ++i)
    {
        auto &closed_set = closed_sets_[i];

        for (auto it = closed_set.begin(); it != closed_set.end();)
        {
            auto node = *it;

            if (node->ReusedFromClosedSet())
            {
                ++it;
                continue;
            }

            if (node->Parent() && node->Parent()->ReusedFromClosedSet())
            {
                // Boundary node.
                this->ReconstructNode(node);
                this->RecycleSubsumedNodes(node, recycle_bin, true);
                reopened_nodes.insert(node);
            }
            else
            {
                // Non-reusable node.
                this->RecycleSubsumedNodes(node, recycle_bin, true);
            }

            closed_set.erase(it++);
        }
    }

    std::cout << "\rFound boundary nodes" << std::flush;

    // Update open sets.
    for (SizeType i = 0; i < virtual_graph_size_; ++i)
    {
        auto &open_set = open_sets_[i];

        for (auto it = open_set.begin(); it != open_set.end();)
        {
            auto node = *it;

            if (node->Parent() == nullptr)
            {
                ++it;
                continue;
            }

            if (node->Parent()->ReusedFromClosedSet() && map_->EdgeExists(node->Parent()->Index(), node->Index()))
            {
                if (!node->IsBounded(p_, eps_))
                {
                    // Boundary node.
                    this->ReconstructNode(node);
                    this->RecycleSubsumedNodes(node, recycle_bin);
                }

                ++it;
                continue;
            }

            // Non-reusable node.
            this->RecycleSubsumedNodes(node, recycle_bin, true);
            open_set.erase(it++);
        }
    }

    std::cout << "\rUpdated open set" << std::flush;

    for (auto node : reopened_nodes)
    {
        open_sets_[node->Index()].insert(node);
    }

    std::cout << "\rMoved reopened nodes to open set" << std::flush;

    std::vector<NodePtr> nodes_to_add;

    while (!recycle_bin.empty())
    {
        auto node = recycle_bin.front();
        recycle_bin.pop();

        if (node->Parent()->ReusedFromClosedSet() && node->Parent()->SearchID() == search_id_)
        {
            if (!node->IsBounded(p_, eps_))
            {
                // Boundary node.
                this->ReconstructNode(node);
                this->RecycleSubsumedNodes(node, recycle_bin, true);
            }

            nodes_to_add.push_back(node);
            continue;
        }

        // Non-reusable node.
        this->RecycleSubsumedNodes(node, recycle_bin, true);
    }

    for (auto node : nodes_to_add)
    {
        AddNode(node, true);
    }

    std::cout << "\rInserted recycle bin" << std::flush;

    for (SizeType i = 0; i < virtual_graph_size_; ++i)
    {
        auto &closed_set = closed_sets_[i];

        for (auto node : closed_set)
        {
            queue_->push(node);
        }

        auto &open_set = open_sets_[i];

        for (auto node : open_set)
        {
            queue_->push(node);
        }
    }

    std::cout << "\rUpdated queue" << std::flush;
}
