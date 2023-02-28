#include <iostream>

#include "global_common.h"

#if USE_CRISP
#include "crisp_robot.h"
#include "crisp_planner.h"
#else
#if USE_PLANAR
#include "planar_robot.h"
#include "planar_planner.h"
#else
#include <cmath>
#include "drone_robot.h"
#include "drone_planner.h"
#endif // USE_PLANAR
#endif // USE_CRISP
#include "inspection_graph.h"
#include <iostream>
#include <string>
#include <fstream>
using VPtr = std::shared_ptr<Inspection::Vertex>;
using EPtr = std::shared_ptr<Inspection::Edge>;
VisibilitySet virtual_graph_coverage_;

int main(int argc, char **argv)
{

	// if (argc < 4) {
	//     std::cerr << "Usage:" << argv[0] << " seed num_vertex file_to_write" << std::endl;
	//     exit(1);
	// }

	// Drone robot

	// Parse input.

	String file_to_read = argv[1];
	Idx seed = std::stoi(argv[2]);

	Inspection::GPtr graph(new Inspection::Graph);
	graph->ReadFromSimulationFile(file_to_read);

	// Robot
	auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, -0.049);
	robot->SetCameraParameters(94.0 / 180 * M_PI, 0.2, 10.0);
	robot->Initialize();

	// Environment setup.
	auto env = std::make_shared<drone::BridgeEnvironment>();

	// Planner
	auto planner = std::make_shared<drone::DronePlanner>(robot, env, seed);
	ob::SpaceInformationPtr space_info_;
	space_info_ = planner->Define_space_info_();
	Inspection::VPtr vertex(new Inspection::Vertex(0));
	Inspection::VPtr parentVertex(new Inspection::Vertex(0));
	vertex->state = space_info_->allocState();
	parentVertex->state = space_info_->allocState();

	Idx countCollision = 0;
	auto total_length = 0.0;

	Inspection::GPtr graph1(new Inspection::Graph);

	// Replace "SimulationPath_" with "test"
	String original_file_to_read = file_to_read;
	std::size_t index = original_file_to_read.find("SimulationPath_");
	original_file_to_read.replace(index, 15, "");

	
	graph1->ReadFromFiles(original_file_to_read, true, 5);

	for (SizeType i = 0; i < graph1->NumVertices(); ++i)
	{
		Inspection::VPtr v = graph1->Vertex(i);
		virtual_graph_coverage_.Insert(v->vis);
	}

	for (size_t i = 0; i < graph->NumVertices(); i++)
	{
		planner->ComputeVisibilitySet(graph->Vertex(i));
		if (i < graph->NumVertices() - 1)
		{
			graph->Edge(i)->valid = planner->CheckEdge(graph->Vertex(i)->state, graph->Vertex(i + 1)->state);
			if (!graph->Edge(i)->valid)
			{
				countCollision++;
			}

			auto parentPosition = graph->Vertex(i)->state->as<DroneStateSpace::StateType>()->Position();
			auto childPosition = graph->Vertex(i + 1)->state->as<DroneStateSpace::StateType>()->Position();

			parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPosition);
			vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPosition);

			// vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(i)->state->as<DroneStateSpace::StateType>()->Yaw());
			// vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(i)->state->as<DroneStateSpace::StateType>()->CameraAngle());
			// parentVertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(i + 1)->state->as<DroneStateSpace::StateType>()->Yaw());
			// parentVertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(i + 1)->state->as<DroneStateSpace::StateType>()->CameraAngle());

			space_info_->copyState(vertex->state, graph->Vertex(i + 1)->state);
			space_info_->copyState(parentVertex->state, graph->Vertex(i)->state);

			// std::cout << "parent:" << std::endl;
			std::cout << parentPosition << std::endl;
			// std::cout << "childPosition:" << std::endl;
			std::cout << childPosition << std::endl;
			// std::cout << "distance" << space_info_->distance(parentVertex->state, vertex->state) << std::endl;

			total_length += space_info_->distance(parentVertex->state, vertex->state);
			// std::cout << "total_length : " << total_length << std::endl;
		}

		for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
		{

			// todo make bitset_ private
			if (virtual_graph_coverage_.bitset_[j] < 0.5)
			{
				graph->Vertex(i)->vis.bitset_[j] = 0;
			}
		}
		graph->UpdateGlobalVisibility(graph->Vertex(i)->vis);
	}

	graph->Save(file_to_read, false);

	std::cout << "CoveragePOIs is : " << graph->NumTargetsCovered() << "\n"
			  << "Number of collision is: " << countCollision << "\n"
			  << "length: " << total_length << std::endl;

	// std::ofstream foutMC(file_to_read + "_MC");

	// if (!foutMC.is_open())
	// {
	// 	std::cout << "Vertex file cannot be opened!" << std::endl;
	// 	exit(1);
	// }

	String mc_file = file_to_read + "_MC";
	std::ofstream fout;
	// std::ifstream fin;

	fout.open(mc_file, std::ios::app);

	if (!fout.is_open())
	{
		std::cerr << mc_file << std::endl;
		exit(1);
	}
	fout << graph->NumTargetsCovered() << " " << total_length << " " << countCollision;
	for (size_t i = 0; i < graph->NumEdges(); i++)
	{
 		fout<< " " << !graph->Edge(i)->valid; 
	}
	fout << " " << std::endl;
	fout.close();
	return 0;

	return 0;
}