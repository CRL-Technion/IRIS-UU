#include "graph_search.h"
#include <algorithm>
#include <execution>
#include <omp.h>

#include <functional>
#include <boost/functional/hash.hpp>

// Idx GraphSearch::hashPOI(const std::vector<Idx> &listIndexes) const
// {
//     // Using boost::hash_range for better distribution
//     return static_cast<Idx>(boost::hash_range(listIndexes.begin(), listIndexes.end()));
// }
// Idx GraphSearch::hashPOI(const std::vector<Idx> listIndexes) const
// {
//     std::hash<Idx> h;
//     size_t ret = 0;

//     for (const auto &i : listIndexes)
//     {
//         ret ^= h(i) + 0x9e3779b9 + (ret << 6) + (ret >> 2);
//     }

//     return ret;
// }

Inspection::VPtr vertex(new Inspection::Vertex(0));
Inspection::VPtr parentVertex(new Inspection::Vertex(0));
Rand rng;
bool use_smart_cache;
GraphSearch::GraphSearch(const Inspection::GPtr graph) : graph_(graph)
{
    virtual_graph_coverage_.Clear();
    open_sets_.clear();
    closed_sets_.clear();

    // auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, -0.049);
    auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, 0.0);

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
    counterBetterThan = 0;
    previousUpdate = 3;
}

void GraphSearch::ReadLocationErrorParameters(const String Location_Error_file_name, const Idx seed, const Idx MC, const Idx _use_smart_cache)
{
    use_smart_cache = _use_smart_cache;
    std::ifstream fin;
    // String fileLocationError ="LocationErrorParameterFile";
    fin.open(Location_Error_file_name);
    if (!fin.is_open())
    {
        std::cerr << "LocationErrorParameters file cannot be opened!" << std::endl;
        exit(1);
    }

    String line;
    Idx i = 0;
    Idx MonteCarloNumber;

    while (!fin.eof())
    {
        fin >> line;
        b_a_milli_g = std::stod(line);
        fin >> line;
        b_g_degPerHr = std::stod(line);
        fin >> line;
        avarageVelocity = std::stod(line);
        fin >> line;
        minTimeAllowInRistZone = std::stod(line);
        fin >> line;
        maxTimeAllowInRistZone = std::stod(line);
        fin >> line;
        multipleCostFunction = std::stod(line);
        fin >> line;
        MonteCarloNumber = std::stod(line);
        fin >> line;
        Threshold_p_coll = std::stod(line);
        break;
    }
    if (MC > 0)
    {
        MonteCarloNumber = MC;
    }
    // while (getline(fin, line))
    // {
    //     std::istringstream sin(line);
    //     String field;

    //     while (getline(sin, field, ' '))
    //     {
    //         sin >> b_a_milli_g;
    //         sin >> b_g_degPerHr;
    //         sin >> avarageVelocity;
    //         sin >> minTimeAllowInRistZone;
    //         sin >> maxTimeAllowInRistZone;
    //         sin >> multipleCostFunction;
    //         sin >> MonteCarloNumber;
    //         sin >> Threshold_p_coll;
    //         // = std::stoi(field);
    //         // b_g_degPerHr = std::stoi(field);
    //         // avarageVelocity = std::stoi(field);
    //         // minTimeAllowInRistZone = std::stoi(field);
    //         // multipleCostFunction = std::stoi(field);

    //         break;
    //     }
    // }
    // std::cout << "b_a_milli_g = " << b_a_milli_g<< std::endl;
    // std::cout << "b_g_degPerHr = " << b_g_degPerHr<< std::endl;
    // std::cout << "avarageVelocity = " << avarageVelocity<< std::endl;
    // std::cout << "minTimeAllowInRistZone = " << minTimeAllowInRistZone<< std::endl;
    // std::cout << "maxTimeAllowInRistZone = " << maxTimeAllowInRistZone<< std::endl;
    // std::cout << "multipleCostFunction = " << multipleCostFunction<< std::endl;
    // std::cout << "MonteCarloNumber = " << MonteCarloNumber<< std::endl;
    // std::cout << "Threshold_p_coll = " << Threshold_p_coll<< std::endl;

    // // todo: insert Threshold_p_coll to LocationErrorParameterFile
    // Threshold_p_coll = 0.05;
    fin.close();
    std::cout << "LocationErrorParameters read!" << std::endl;
    std::cout << b_a_milli_g << " " << b_g_degPerHr << " " << avarageVelocity << " " << minTimeAllowInRistZone << " " << maxTimeAllowInRistZone << " " << multipleCostFunction << " " << MonteCarloNumber << " " << Threshold_p_coll << std::endl;

    // rund monteCarloParameter

    rng.seed(seed);
    std::cout << "seed" << seed << std::endl;
    // Idx MonteCarloNumber = 5;
    auto milli_g2mpss = 9.81 / 1000.0;                 //   Conversion from [mili g ] to [m/s^2]
    auto degPerHr2radPerSec = (3.14 / 180.0) / 3600.0; //  Conversion from [deg/hr] to [rad/s]
    auto b_a = b_a_milli_g * milli_g2mpss;
    auto b_g = b_g_degPerHr * degPerHr2radPerSec;
    // auto ba_input = -b_a/ 5.0 ;
    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        // RealNormalDist Norm1(0, sqrt(b_a) / 3);
        RealNormalDist Norm1(0, sqrt(b_a) / 4);
        // RealNormalDist Norm1(0, (b_a) / 2);
        // RealNormalDist Norm1(0, (b_a) / 4.5);
        ba_x.push_back(Norm1(rng));
        ba_y.push_back(Norm1(rng));
        // ba_z.push_back(Norm1(rng));
        ba_z.push_back(Norm1(rng)); // todo simple test only
        // ba_input+=2*(b_a/5.0)/MonteCarloNumber-1;
        // RealNormalDist Norm1(0,  +1e-10);
        //  ba_x.push_back(Norm1(rng)+ba_input);
        //  ba_y.push_back(Norm1(rng)+ba_input);
        //  ba_z.push_back(Norm1(rng)+ba_input);
        std::cout << "b_a = " << ba_x[i] << "," << ba_y[i] << "," << ba_z[i] << "," << std::endl;
    }
    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        // RealNormalDist Norm2(0, sqrt(b_g) / 3);
        RealNormalDist Norm2(0, sqrt(b_g) / 4);
        // RealNormalDist Norm2(0, (b_g) / 2);
        // RealNormalDist Norm2(0, (b_g) / 4.5);

        bg_x.push_back(Norm2(rng));
        bg_y.push_back(Norm2(rng));
        bg_z.push_back(Norm2(rng));
        std::cout << "b_g = " << bg_x[i] << "," << bg_y[i] << "," << bg_z[i] << "," << std::endl;
    }

    for (size_t i = 0; i < MonteCarloNumber; i++)
    {
        Vec3 temp{0.0, 0.0, 0.0};
        totalLocationErrorDefault.push_back(temp);
        CostToComeMcDefault.push_back(0.0);
        costToComeRiskZoneDefault.push_back(0.0);
    }
    if (MonteCarloNumber < 0.5) // for case of cost and stop
    {
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
                // todo david e->virtual_checked = true;
            }
        }
    }

    virtual_graph_size_ = new_size;
    return virtual_graph_size_;
}

std::vector<Idx> GraphSearch::SearchVirtualGraph(NodePtr &result_node)
{
    const TimePoint start = Clock::now();

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

            counterBetterThan++;
            // std::cout << "counterBetterThan" << counterBetterThan << std::endl;
            if (counterBetterThan > 9000000 && previousUpdate < 0.01)
            {
                found = true; // todo
                break;
            }
            // Updage result.
            if (result_node == nullptr || n->BetterThan(result_node))
            {
                if (result_node != nullptr)
                {
                    previousUpdate = n->CoverageSize() - result_node->CoverageSize();
                }
                counterBetterThan = 0;
                std::cout << "virtual_graph_coverage_.Size(): " << virtual_graph_coverage_.Size() << std::endl;
                result_node = n;
                std::cout << "this->CoverageSize(): " << result_node->CoverageSize() << std::endl;

                // std::cout <<  "countVisible = " <<  planner->env_->countVisible << ",countNotVisible = "<< planner->env_->countNotVisible << "ratio = "<<  100.0*planner->env_->countNotVisible/(planner->env_->countVisible+planner->env_->countNotVisible) << std::endl;
                // for (size_t i = 0; i < MAX_COVERAGE_SIZE; i++)
                // {
                //     if (i == MAX_COVERAGE_SIZE - 1)
                //     {
                //         std::cout << " " << result_node->VisSet().bitset_[i] << std::endl;
                //     }
                //     else
                //     {
                //         std::cout << " " << result_node->VisSet().bitset_[i];
                //     }
                // }
                // std::vector<Idx> path;
                // auto tag = result_node;
                // path.push_back(tag->Index());

                // while (tag)
                // {
                //     auto super_edge = tag->LocalPath();

                //     for (auto i = 1; i < super_edge.size(); ++i)
                //     {
                //         path.push_back(super_edge[i]);
                //     }

                //     tag = tag->Parent();
                // }

                // std::reverse(path.begin(), path.end());
                // for (auto &p : path)
                // {
                //     std::cout << " " << p;
                // }

                // std::cout << std::endl;
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
            // exit(1);todo
        }

        valid_result_found = true;

        // todo david
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
                std::cout << "\rReplan: " << ++num_replan << std::endl;
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
        Idx MonteCarloNum = ba_x.size();
        NodePtr source_node(new Node(graph_->Vertex(source_idx_)->index));

        Inspection::EPtr edge;
        if (false) //(MonteCarloNum > 0.5)
        {
            vis.Clear();
            RealNum cost0 = 0.0;
            _isfirstTime = true;
            ReCalculateIPVCostMC(source_node, graph_->Vertex(source_idx_)->index, source_node, cost0, edge, 1);
            source_node->SetVisSet(vis);
            source_node->SetChecked(true);
#if USE_GHOST_DATA
            source_node->SetGhostVisSet(vis);
#endif
        }
        else
        {
            source_node->SetVisSet(graph_->Vertex(source_idx_)->vis);
            source_node->SetChecked(true);
#if USE_GHOST_DATA
            source_node->SetGhostVisSet(graph_->Vertex(source_idx_)->vis);
#endif
        }
        _isfirstTime = false;
        source_node->SetTotalLocationError(totalLocationErrorDefault);
        source_node->SetCostToComeMc(CostToComeMcDefault);
        source_node->SetCostToComeRiskZone(costToComeRiskZoneDefault);
        source_node->SetCountVertexInRiskZone(0);

        source_node->SetCollisionProbability(0);

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
    // std::cout << "successors" << successors.size() << std::endl;

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

#if UAV_NAVIGATION_ERROR
        auto cost = map_->Cost(v, n->Index());

        Idx MonteCarloNum = ba_x.size();
        bool isNodeValidateForRiskZone = false;

        Inspection::EPtr edge = graph_->FindEdge(n->Index(), v);

        // // todo david
        if (n->Parent() == nullptr)
        {
            // std::cout << "n->Parent() == nullptr: " << n->Index() << " " << v << std::endl;
            edge->valid = true;
            edge->checkedMC = false;
        }
        if (!edge->valid)
        {
            // map_->RemoveDirectEdge(edge->source, edge->target);
            continue;
        }
        bool isLum = false;
        if (MonteCarloNum < 0.5) // ref iris, irisLimit, irisMinimize
        {
            auto childPosition = graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Position();
            if (isLum && planner->IsPointInsideBox(childPosition))
            {
                // RealNum GraphSearch::LocationErrorFunc(const RealNum b_a, const RealNum b_g, const RealNum timeRiskZone) const
                cost = cost+30;
                //continue;
            }
            isNodeValidateForRiskZone = true;
            new_node->Extend(v, cost, graph_->Vertex(v)->vis);
        }
        else
        {
            vis.Clear();
            samplingForIpv = true;
            isNodeValidateForRiskZone = ReCalculateIPVCostMC(n, v, new_node, cost, edge, successors.size());
            // new_node->Extend(v, cost, graph_->Vertex(v)->vis);
            if (!isNodeValidateForRiskZone) // larger than max time in risk zone or collision
            {
                continue;
            }

            if (samplingForIpv)
            {
                new_node->Extend(v, cost, vis);
            }
            else
            {
                new_node->Extend(v, cost, graph_->Vertex(v)->vis);
            }
        }

        // std::cout << "cost" << cost << std::endl;
#endif
        // std::cout << "v: " << graph_->Vertex(v)->vis.Size() << std::endl;
        // new_node->Extend(v, cost, graph_->Vertex(v)->vis);

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

bool GraphSearch::ReCalculateIPVCostMC(NodePtr n, Idx v, NodePtr new_node, RealNum &cost, Inspection::EPtr edge, Idx SuccessorSize)
{
    bool isGNSSINS = true;
    bool isInsideBox = false;
    // std::cout << "bug000" << cost << std::endl;
    bool usedPOIcache = false;
    auto previousTotalLocationError = new_node->GetTotalLocationError();
    auto perviousCostRiskZone = new_node->GetCostToComeRiskZone();

    // todo for same runs
    // RealNormalDist NormR(0, 1);
    // RealNormalDist NormAngle(0, 2 * M_PI);
    // auto r = abs(NormR(rng));
    // auto azimuth = NormAngle(rng);
    // auto elevation = NormAngle(rng);

    if (!edge->valid)
    {
        std::cout << "bug000" << cost << std::endl;

        return false;
    }
    // std::cout << "ssss: " << n->Index() << " " << v << std::endl;

    _isfirstTime = false;

    auto childPosition = graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Position();
    auto parentPosition = graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Position();

    bool isChildOutsideRiskZone = false;
    bool isEdgeOutsideRiskZone = false;
    bool isParentOutsideRiskZone = false;

    if (!planner->IsPointInsideBox(parentPosition))
    {
        isParentOutsideRiskZone = true;
    }
    if (!planner->IsPointInsideBox(childPosition))
    {
        isChildOutsideRiskZone = true;
        if (isParentOutsideRiskZone)
        {
            isEdgeOutsideRiskZone = true;
        }
    }

    auto CountVertexInRiskZone = new_node->GetCountVertexInRiskZone();
    if (isEdgeOutsideRiskZone)
    {
        CountVertexInRiskZone = 0;
    }
    else // the vertex is inside the Box
    {
        CountVertexInRiskZone++;
    }
    new_node->SetCountVertexInRiskZone(CountVertexInRiskZone);

    VertexVisCache::const_iterator it;
    if (samplingForIpv)
    {
        if (use_smart_cache)
        {
            size_t hashValue = v; // Initialize hashValue with v

            for (size_t i = 0; i < CountVertexInRiskZone; i++)
            {
                boost::hash_combine(hashValue, new_node->Parent()->Index());
            }

            // Add a constant offset to ensure hashValue is different from v
            if (CountVertexInRiskZone > 0)
            {
                hashValue += graph_->NumVertices(); // todo max vertices number
            }

            // Insert into cache
            it = vertex_vis_cache_.find(hashValue);
        }
        else
        {
            // Without smart cache, use v directly
            it = vertex_vis_cache_.find(v);
        }
    }
    if (it != vertex_vis_cache_.end() || !samplingForIpv)
    {
        usedPOIcache = true;
        vis = std::get<0>(it->second); // it->second.first;
        if (use_smart_cache && CountVertexInRiskZone > 0 && samplingForIpv)
        {
            edge->cost = std::get<1>(it->second);
            edge->collision_prob = std::get<2>(it->second);
            edge->costMc = std::get<5>(it->second);
            previousTotalLocationError = std::get<3>(it->second);
            new_node->SetTotalLocationError(previousTotalLocationError);
            perviousCostRiskZone = std::get<4>(it->second);
            new_node->SetCostToComeRiskZone(perviousCostRiskZone);
        }
        // todo david

        if ((edge->checkedMC || (use_smart_cache && CountVertexInRiskZone > 0)) && ba_x.size() > 0) // todo
        {
            // const Idx index = v;
            //  std::vector<Idx> temp;

            //   if (CountVertexInRiskZone>0)
            //     {
            //         std::cout << "used history cache" << std::endl;
            //     }
            auto previousPColl = new_node->GetCollisionProbability();
            auto temp_p_coll_MC = 1 - (1 - previousPColl) * (1 - 1.0 * edge->collision_prob);
            if (temp_p_coll_MC > (Threshold_p_coll + 1e-4))
            {
                edge->valid = false;
                // std::cout << "vertex = " << v << std::endl;
                // map_->RemoveDirectEdge(edge->source, edge->target);
                // std::cout << "temp_p_coll_MC " << temp_p_coll_MC << std::endl;
                // std::cout << "Threshold_p_coll " << Threshold_p_coll << std::endl;
                //std::cout << "collision2: " << n->Index() << " " << v << std::endl;

                return false;
            }
            new_node->SetCollisionProbability(temp_p_coll_MC);

            // TODO david
            auto perviousCostToComeMc = new_node->GetCostToComeMc();
            for (size_t i = 0; i < ba_x.size(); i++)
            {
                perviousCostToComeMc[i] += edge->costMc[i];
            }
            new_node->SetCostToComeMc(perviousCostToComeMc);

            // The visibility set for this vertex has already been computed and cached.
            cost = edge->cost;

            if (samplingForIpv)
            {
                // vis = std::get<0>(it->second); // it->second.first;
                Idx MonteCarloNum = ba_x.size();

                for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
                {
                    if (virtual_graph_coverage_.bitset_[j] > 0.5)
                    {
                        RealNum p = 0.0;
                        if (MonteCarloNum > 0.5)
                        {
                            p = 1.0 * vis.bitset_[j];
                        }
                        else
                        {
                            p = 1;
                        }

                        vis.bitset_[j] = 1 - (1 - p) * (1 - new_node->VisSet().bitset_[j]);

                        // if (vis.bitset_[j] > 0.99)
                        // {
                        //     vis.bitset_[j] = 1;
                        // }
                        // if (vis.bitset_[j] < 0)
                        // {
                        //     vis.bitset_[j] = 0;
                        // }
                    }
                }
            }

            return true;
        }
    }
    // Idx SuccessorSize = 1;
    SuccessorSize = 1;
    int i_Successor = 0;
    // for (int i_Successor = 0; i_Successor < SuccessorSize; i_Successor++)
    {
        //  std::cout << "bug111" << cost << std::endl;
        // std::cout << "edge->checkedMC " << edge->checkedMC << std::endl;
        RealNum totalCost = 0.0, currentTimeRiskZone = 0.0, perviousTimeRiskZone = 0.0, currentCost = 0.0, p_coll_MC = 0.0;
        Idx MonteCarloNum = ba_x.size();
        Vec3 parentPosition_fix;
        Vec3 childPosition_fix;

        auto perviousCostToComeMc = new_node->GetCostToComeMc();
        if (!edge->checkedMC)
        {
            edge->costMc = new_node->GetCostToComeMc();
        }
        auto previousPColl = new_node->GetCollisionProbability();

        // space_info_->copyState(vertex->state, graph_->Vertex(v)->state);
        // space_info_->copyState(parentVertex->state, graph_->Vertex(n->Index())->state);
        // vertex->state = space_info_->allocState();
        // parentVertex->state = space_info_->allocState();
        // todo Yaw and camera angle
        vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->Yaw());
        vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph_->Vertex(v)->state->as<DroneStateSpace::StateType>()->CameraAngle());
        parentVertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->Yaw());
        parentVertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph_->Vertex(n->Index())->state->as<DroneStateSpace::StateType>()->CameraAngle());

        // return true;
        // const Idx index = v;

        RealNum locationErrorMean = 0;

        // #pragma omp parallel for
        for (size_t i = 0; i < ba_x.size(); i++)
        {

            /// for cost calculation

            if (!isGNSSINS)
            {
                if (n->Parent() != nullptr)
                {
                }
                else
                {
                    previousTotalLocationError[i][0] = 0;
                    previousTotalLocationError[i][1] = 0;
                    previousTotalLocationError[i][2] = 0;
                }
            }
            parentPosition_fix[0] = parentPosition[0] + previousTotalLocationError[i][0];
            parentPosition_fix[1] = parentPosition[1] + previousTotalLocationError[i][1];
            parentPosition_fix[2] = parentPosition[2] + previousTotalLocationError[i][2];
            // bool IsParentInRiskZone = planner->IsPointInsideBox(parentPosition);

            // this is approximation of the childPosition
            childPosition_fix[0] = childPosition[0] + previousTotalLocationError[i][0];
            childPosition_fix[1] = childPosition[1] + previousTotalLocationError[i][1];
            childPosition_fix[2] = childPosition[2] + previousTotalLocationError[i][2];

            if (!planner->IsPointInsideBox(childPosition))
            {

                // reset previousTotalLocationError - RandomNoiseGNSS
                RealNormalDist NormR(0, 1);
                RealNormalDist NormAngle(0, 2 * M_PI);
                auto r = abs(NormR(rng));
                auto azimuth = NormAngle(rng);
                auto elevation = NormAngle(rng);

                previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
                previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
                previousTotalLocationError[i][2] = -r * sin(elevation);

                // update childPosition_fix with RandomNoiseGPS
                childPosition_fix[0] = childPosition[0] + previousTotalLocationError[i][0];
                childPosition_fix[1] = childPosition[1] + previousTotalLocationError[i][1];
                childPosition_fix[2] = childPosition[2] + previousTotalLocationError[i][2];

                // reset perviousCostRiskZone
                perviousCostRiskZone[i] = 0.0;

                // compute cost
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPosition_fix);
                if (!edge->checkedMC)
                {
                    parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition_fix);
                    if (!planner->CheckEdge(parentVertex->state, vertex->state))
                    {
                        p_coll_MC += 1.0;
                        // edge->valid = false;
                        //   std::cout << "parentPosition_fix11: " << i << " " << parentPosition_fix[0] << " " << parentPosition_fix[1] << " " << parentPosition_fix[2] << std::endl;
                        //   std::cout << "childPosition_fix11: " << i << " " << childPosition_fix[0] << " " << childPosition_fix[1] << " " << childPosition_fix[2] << std::endl;

                        // graph_->SetEdgeValidity(n->Index(), v, false);

                        // return false;
                    }
                    currentCost += space_info_->distance(parentVertex->state, vertex->state);
                }
                // recalculate POI

                if (!usedPOIcache && samplingForIpv)
                {
                    planner->ComputeVisibilitySet(vertex);
                }
                // todo find the exit point to compute accurate cost
            }
            // #if ToyProblem
            else if (!isGNSSINS)
            {
                // return false;
                //  reset previousTotalLocationError - RandomNoiseGNSS
                RealNormalDist NormR(0, 1);
                RealNormalDist NormAngle(0, 2 * M_PI);
                auto r = abs(NormR(rng)) * 3;
                auto azimuth = NormAngle(rng);
                auto elevation = NormAngle(rng);
                

                previousTotalLocationError[i][0] = r * cos(elevation) * cos(azimuth);
                previousTotalLocationError[i][1] = r * cos(elevation) * sin(azimuth);
                previousTotalLocationError[i][2] = -r * sin(elevation);

                // update childPosition_fix with RandomNoiseGPS
                childPosition_fix[0] = childPosition[0] + previousTotalLocationError[i][0];
                childPosition_fix[1] = childPosition[1] + previousTotalLocationError[i][1];
                childPosition_fix[2] = childPosition[2] + previousTotalLocationError[i][2];

                // reset perviousCostRiskZone
                // compute cost
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPosition_fix);
                if (!edge->checkedMC)
                {
                    parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition_fix);
                    if (!planner->CheckEdge(parentVertex->state, vertex->state))
                    {
                        p_coll_MC += 1.0;
                        // edge->valid = false;
                        //  std::cout << "parentPosition_fix11: " << i << " " << parentPosition_fix[0] << " " << parentPosition_fix[1] << " " << parentPosition_fix[2] << std::endl;
                        //  std::cout << "childPosition_fix11: " << i << " " << childPosition_fix[0] << " " << childPosition_fix[1] << " " << childPosition_fix[2] << std::endl;

                        // graph_->SetEdgeValidity(n->Index(), v, false);

                        // return false;
                    }
                    currentCost += space_info_->distance(parentVertex->state, vertex->state);
                }
                // recalculate POI
                if (!usedPOIcache && samplingForIpv)
                {
                    planner->ComputeVisibilitySet(vertex);
                }
                // planner->ComputeVisibilitySet(vertex);
            }
            // // if (false)
            // #else
            //         else
            // #endif
            else // if (false)
            {
                // if (!planner->IsPointInsideBox(parentPosition_fix)) // if q^c_{i-1} \in w_cov , find intersect point
                if (!planner->IsPointInsideBox(parentPosition)) // if q^c_{i-1} \in w_cov , find intersect point
                {
                    // auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPosition_fix, childPosition_fix, insertPoint);
                    auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPosition, childPosition, insertPoint);
                    if (!IsInsertToRiskZone)
                    {
                        std::cout << "child is inside and parent is outside and there is not intersection point " << std::endl;
                        // return false;
                    }
                    else
                    {
                        // compute cost
                        parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition_fix);
                        vertex->state->as<DroneStateSpace::StateType>()->SetPosition(insertPoint);
                        currentCost += space_info_->distance(parentVertex->state, vertex->state);

                        // if (!planner->CheckEdge(parentVertex->state, vertex->state)) // todo threshold collision >0
                        // {
                        //     p_coll_MC += 1.0;
                        //     return false;
                        // }

                        // update parentPosition_fix to be the insert point
                        parentPosition_fix[0] = insertPoint[0];
                        parentPosition_fix[1] = insertPoint[1];
                        parentPosition_fix[2] = insertPoint[2];
                    }
                }

                parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition_fix);
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPosition_fix);
                //  This is an approximation for calculate "currentTimeRiskZone"

                perviousTimeRiskZone = perviousCostRiskZone[i];

                currentTimeRiskZone = perviousTimeRiskZone + space_info_->distance(parentVertex->state, vertex->state);

                auto x = childPosition_fix[0] - parentPosition_fix[0];
                auto y = childPosition_fix[1] - parentPosition_fix[1];
                auto z = childPosition_fix[2] - parentPosition_fix[2];
                // todo psi and theta
                RealNum psi = atan2(y, x);
                RealNum theta = -atan2(z, sqrt(x * x + y * y));

                auto temp_x_error = ba_x[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
                temp_x_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_z[i] + bg_y[i]);

                auto temp_y_error = ba_y[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
                temp_y_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (bg_z[i] - bg_x[i]);

                auto g = 9.81;
                auto temp_z_error = ba_z[i] * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
                temp_z_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_y[i] + bg_x[i]) * (-g);

                previousTotalLocationError[i][0] += cos(theta) * cos(psi) * temp_x_error - sin(psi) * temp_y_error + sin(theta) * cos(psi) * temp_z_error;
                previousTotalLocationError[i][1] += cos(theta) * sin(psi) * temp_x_error + cos(psi) * temp_y_error + sin(theta) * sin(psi) * temp_z_error;
                previousTotalLocationError[i][2] += -sin(theta) * temp_x_error + cos(theta) * temp_z_error;

                // update childPosition_fix with IMUNoise
                childPosition_fix[0] = childPosition[0] + previousTotalLocationError[i][0];
                childPosition_fix[1] = childPosition[1] + previousTotalLocationError[i][1];
                childPosition_fix[2] = childPosition[2] + previousTotalLocationError[i][2];

                auto temp = sqrt(previousTotalLocationError[i][0] * previousTotalLocationError[i][0] + previousTotalLocationError[i][1] * previousTotalLocationError[i][1] + previousTotalLocationError[i][2] * previousTotalLocationError[i][2]);
                // std::cout << "temp: " << temp << std::endl;

                parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition_fix);
                vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPosition_fix);
                // if (!edge->checkedMC)
                // {
                if (!planner->CheckEdge(parentVertex->state, vertex->state)) // todo threshold collision >0
                {
                    p_coll_MC += 1.0;
                    // edge->valid = false;
                    //  graph_->SetEdgeValidity(n->Index(), v, false);
                    // std::cout << "333" << std::endl;
                    // return false;
                }
                // }
                perviousCostRiskZone[i] += space_info_->distance(parentVertex->state, vertex->state);
                currentCost += space_info_->distance(parentVertex->state, vertex->state);
                // currentTimeRiskZone = perviousCostRiskZone[i];
                // if (currentTimeRiskZone > maxTimeAllowInRistZone)
                // {
                //     // option to limit the max value of LU
                //     // new_node->SetCostToComeRiskZone(perviousCostRiskZone);

                //     return false;
                // }

                planner->ComputeVisibilitySet(vertex);

                // cost
            }
            if (!usedPOIcache && samplingForIpv)
            {

                for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
                {
                    // todo make bitset_ private
                    if (virtual_graph_coverage_.bitset_[j] > 0.5 && vertex->vis.bitset_[j] > 0.5)
                    {
                        vis.bitset_[j] += 1.0 / MonteCarloNum;
                    }
                }
            }
            /////////////////////////////////////////////////////////////////////////////////////
            // if ((currentTimeRiskZone + perviousTimeRiskZone) > maxTimeAllowInRistZone)
            // {
            //     // std::cout << "accumulatedCostRiskZone " << CostRiskZone+accumulatedCostRiskZone;
            //     std::cout << "," << currentTimeRiskZone + perviousTimeRiskZone << std::endl;
            //     //vis.Insert(graph_->Vertex(v)->vis);
            //     //return false;
            // }

            // if ((currentTimeRiskZone + perviousTimeRiskZone) > minTimeAllowInRistZone)
            // {
            //    // totalCost += currentCost * 5;
            // }
            /////////////////////////////////////////////////////////////////////////////////////
            if (i_Successor < 0.5)
            {
                perviousCostToComeMc[i] += currentCost;
            }
            totalCost += currentCost;
            if (!edge->checkedMC)
            {
                edge->costMc[i] = currentCost;
            }
            currentCost = 0;
            locationErrorMean += sqrt(previousTotalLocationError[i][0] * previousTotalLocationError[i][0] + previousTotalLocationError[i][1] * previousTotalLocationError[i][1] + previousTotalLocationError[i][2] * previousTotalLocationError[i][2]);
        }
        // if (isInsideBox)
        // {
        //     std::cout << "perviousCostRiskZone=";
        //     for (size_t i = 0; i < ba_x.size(); i++)
        //     {
        //         std::cout << perviousCostRiskZone[i] << ",";
        //     }
        //     std::cout<<std::endl;
        // }

        for (const auto &currentTimeRiskZone : perviousCostRiskZone)
        {
            if (currentTimeRiskZone > maxTimeAllowInRistZone)
            {
                // Option to limit the max value of LU
                // new_node->SetCostToComeRiskZone(perviousCostRiskZone);
                std::cout << "111 " << currentTimeRiskZone << std::endl;
                return false;
            }
        }

        locationErrorMean /= MonteCarloNum;
        // std::cout << "locationErrorMean: " << locationErrorMean << std::endl;

        if (locationErrorMean > maxTimeAllowInRistZone) // todo change the name of maxTimeAllowInRistZone
        {
            // std::cout << "locationErrorMean - no: " << locationErrorMean << std::endl;
            std::cout << "222" << std::endl;

            return false;
        }
        // if (!usedPOIcache || isChildOutsideRiskZone || isParentOutsideRiskZone && samplingForIpv)

        if (!edge->checkedMC)
        {
            if (i_Successor >= SuccessorSize - 1.5)
            {
                if (!isGNSSINS || isEdgeOutsideRiskZone)
                {
                    edge->checkedMC = 1;
                }
            }
            edge->collision_prob = p_coll_MC / MonteCarloNum;
            if (i_Successor < 0.5)
            {
                edge->cost = 1.0 * totalCost / MonteCarloNum;
            }
            // else
            // {
            //     edge->cost = (edge->cost * i_Successor + 1.0 * totalCost / MonteCarloNum) / (i_Successor + 1);
            // }
        }

        // if ((!usedPOIcache || isChildOutsideRiskZone) && samplingForIpv)
        // if ((!usedPOIcache && (isChildOutsideRiskZone || use_smart_cache)) && samplingForIpv)
        if ((!usedPOIcache && (isChildOutsideRiskZone || use_smart_cache)) && samplingForIpv)
        {
            if (use_smart_cache) // todo david is edgeoutsideriskzone
            {
                size_t hashValue = v; // Initialize hashValue with v

                for (size_t i = 0; i < CountVertexInRiskZone; i++)
                {
                    boost::hash_combine(hashValue, new_node->Parent()->Index());
                }

                // Add a constant offset to ensure hashValue is different from v
                if (CountVertexInRiskZone > 0)
                {
                    hashValue += graph_->NumVertices();
                }

                // Insert into cache
                vertex_vis_cache_[hashValue] = std::make_tuple(vis, edge->collision_prob, edge->cost, previousTotalLocationError, perviousCostRiskZone, edge->costMc);
                // vertex_vis_cache_[hashValue] = vis;
            }
            else
            {
                if (isChildOutsideRiskZone)
                {
                    // Without smart cache, use v directly
                    vertex_vis_cache_[v] = std::make_tuple(vis, edge->collision_prob, edge->cost, previousTotalLocationError, perviousCostRiskZone, edge->costMc);
                    // vertex_vis_cache_[v] = vis;
                }
            }
        }

        cost = edge->cost;
        auto temp_p_coll_MC = 1 - (1 - previousPColl) * (1 - 1.0 * edge->collision_prob);
        // auto temp_p_coll_MC = (p_coll_MC / MonteCarloNum);

        if (temp_p_coll_MC > (Threshold_p_coll + 1e-4))
        {
            if (!isGNSSINS)
            {
                edge->valid = false;
                // std::cout << "collision: " << n->Index() << " " << v << std::endl;
            }
            //
            //  map_->RemoveDirectEdge(edge->source, edge->target);

            // graph_->SetEdgeValidity(n->Index(), v, false);
            //std::cout << "collision1: " << n->Index() << " " << v << std::endl;

            return false;
        }
        // set MC parameter
        new_node->SetTotalLocationError(previousTotalLocationError);
        new_node->SetCostToComeMc(perviousCostToComeMc);
        new_node->SetCostToComeRiskZone(perviousCostRiskZone);
        new_node->SetCollisionProbability(temp_p_coll_MC);

        // new_node->SetCollisionProbability(temp_p_coll_MC);
        // Mean MC cost
        //  std::cout << "cost: " << cost;

        // std::cout << "costUpdate: " << cost << std::endl;
        // Mean MC POI
        if (samplingForIpv)
        {

            for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
            {
                if (virtual_graph_coverage_.bitset_[j] > 0.5)
                {
                    RealNum p = 0.0;
                    if (MonteCarloNum > 0.5)
                    {
                        p = 1.0 * vis.bitset_[j];
                    }
                    else
                    {
                        p = 1;
                    }

                    vis.bitset_[j] = 1 - (1 - p) * (1 - new_node->VisSet().bitset_[j]);

                    // if (vis.bitset_[j] > 0.99)
                    // {
                    //     vis.bitset_[j] = 1;
                    // }
                    // if (vis.bitset_[j] < 0)
                    // {
                    //     vis.bitset_[j] = 0;
                    // }
                }
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
                // todo Contains
                // if (node->GhostCost() < s->GhostCost() || !s->GhostVisSet().Contains(node->GhostVisSet()))
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

bool GraphSearch::InGoalSet(const NodePtr n)
{
#if USE_GHOST_DATA
    auto GhostSize = n->GhostVisSet().Size();
    auto virtualSize = virtual_graph_coverage_.Size();

    // std::cout << "InGoalSet: " << GhostSize << " " << virtualSize << std::endl;

    // return (n->VisSet().Size() >= p_ * virtual_graph_coverage_.Size());
    // return (n->GhostVisSet() == virtual_graph_coverage_);
    // if (abs(GhostSize - virtualSize) < 1e-3 || GhostSize > virtualSize)
    if ((n->VisSet().Size() >= p_ * virtual_graph_coverage_.Size()))
    {
        // if (p_ < 1)
        // {
        //     p_ += 0.01;
        //     if (p_ > 1)
        //     {
        //         p_ = 1;
        //     }
        //     std::cout << "InGoalSet: " << GhostSize << " " << virtualSize << std::endl;

        //     return false;
        // }
        std::cout << "InGoalSet: " << GhostSize << " " << virtualSize << std::endl;
        return true;
    }
    return false;

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
            // todo david
            if (s->VisSet().Contains(node->VisSet()))
            {
                s->Subsume(node, true);
                return true;
            }
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
            // todo Contains
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
            // todo Contains
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
                // todo Contains
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

    //
    // if (n1->CostToCome() < n2->CostToCome())
    // {
    //     return false;
    // }
    // todo david
    if (n1->CoverageSize() < n2->CoverageSize())
    {
        return false;
    }

    if ((1 + eps_) * (n2->GhostCost()) < n1->CostToCome())
    {
        return false;
    }
    // todo smaller with epsilon

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
        // for (auto i = len - 1; i > 0; --i)
        // todo david
        for (auto i = len - 1; i > 1; --i)
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

RealNum GraphSearch::ResultCoverageSize() const
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
        << "RayTracingTime "
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
        << planner->GetElapsedTimeRayTracing() << " " << std::endl;
}
void GraphSearch::SaveResultsMC(std::ostream &out, NodePtr result_node) const
{
    auto temp = result_node->GetCostToComeMc();
    for (auto &p : temp)
    {
        out << " " << p;
    }
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
