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
VisibilitySet virtual_graphcoverage_;
Rand rng;
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
	Idx MonteCarloNum = std::stoi(argv[3]);

	Inspection::GPtr graph(new Inspection::Graph);
	graph->ReadFromFiles(file_to_read, true, 5);

	// read result path
	String file_to_read_result = file_to_read + "_result";

	std::ifstream infile(file_to_read_result);
	std::string line;
	std::getline(infile, line); // read the first line of the file

	int id;
	std::vector<int> numbers;
	std::istringstream iss(line);
	iss >> id;
	int num;
	while (iss >> num)
	{
		numbers.push_back(num);
	}

	// std::cout << "ID: " << id << std::endl;
	// std::cout << "Numbers: ";
	// for (auto n : numbers)
	// {
	// 	std::cout << n << " ";
	// }
	// std::cout << std::endl;

	infile.close(); // close the file
	std::cout << "result read!" << std::endl;
	rng.seed(seed+13);
	// Robot
	auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, 0);
	robot->SetCameraParameters(94.0 / 180 * M_PI, 0.2, 10.0);
	robot->Initialize();

	// Environment setup.
	auto env = std::make_shared<drone::BridgeEnvironment>();

	// Planner
	auto planner = std::make_shared<drone::DronePlanner>(robot, env, seed);
#if ToyProblem==0
	for (SizeType i = 0; i < 3; ++i)
	{
		planner->LowerBordersXYZ[i] += 2;
		planner->UpperBordersXYZ[i] -= 2;
	}
#endif

	ob::SpaceInformationPtr space_info_;
	space_info_ = planner->Define_space_info_();
	Inspection::VPtr vertex(new Inspection::Vertex(0));
	Inspection::VPtr parentVertex(new Inspection::Vertex(0));
	vertex->state = space_info_->allocState();
	parentVertex->state = space_info_->allocState();

	Inspection::GPtr graph1(new Inspection::Graph);

	for (SizeType i = 0; i < graph->NumVertices(); ++i)
	{
		Inspection::VPtr v = graph->Vertex(i);
		virtual_graphcoverage_.Insert(v->vis);
	}
	String mc_file = file_to_read + "_MC";
	std::ofstream fout;
	// std::ifstream fin;

	fout.open(mc_file, std::ios::app);

	if (!fout.is_open())
	{
		std::cerr << mc_file << std::endl;
		exit(1);
	}
	for (size_t j = 0; j < MonteCarloNum; j++)
	{
		Idx countCollision = 0;
		auto total_length = 0.0;
		graph->ResetGlobalVisibility();

		std::vector<bool> edgeValid;
		Vec3 previousTotalLocationError;
		for (size_t k = 0; k < 3; k++)
		{
			previousTotalLocationError[k] = 0;
		}

		for (size_t i = 0; i < numbers.size(); i++)
		{

			auto parentPosition = graph->Vertex(numbers[i])->state->as<DroneStateSpace::StateType>()->Position();
			// std::cout << "previousTotalLocationError " << previousTotalLocationError[0] << " " << previousTotalLocationError[1] << " " << previousTotalLocationError[2] << std::endl;
			Vec3 parentPositionFix;
			for (size_t k = 0; k < 3; k++)
			{
				parentPositionFix[k] = parentPosition[k] + previousTotalLocationError[k];
			}

			parentVertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(numbers[i])->state->as<DroneStateSpace::StateType>()->Yaw());
			parentVertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(numbers[i])->state->as<DroneStateSpace::StateType>()->CameraAngle());
			parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPositionFix);

			planner->ComputeVisibilitySet(parentVertex);

			if (i < numbers.size() - 1)
			{

				auto childPosition = graph->Vertex(numbers[i + 1])->state->as<DroneStateSpace::StateType>()->Position();
				RealNum sigmaNoise = 3.0;
				if (!planner->IsPointInsideBox(childPosition))
				{
					sigmaNoise = 1.0;
				}

				// reset previousTotalLocationError - RandomNoiseGNSS
				RealNormalDist NormR(0, 1);
				RealNormalDist NormAngle(0, 2 * M_PI);
				auto r = abs(NormR(rng)) * sigmaNoise;
				auto azimuth = NormAngle(rng);
				auto elevation = NormAngle(rng);

				previousTotalLocationError[0] = r * cos(elevation) * cos(azimuth);
				previousTotalLocationError[1] = r * cos(elevation) * sin(azimuth);
				previousTotalLocationError[2] = -r * sin(elevation);

				Vec3 childPositionFix;
				for (size_t k = 0; k < 3; k++)
				{
					childPositionFix[k] = childPosition[k] + previousTotalLocationError[k];
				}

				vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPositionFix);
				vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(numbers[i + 1])->state->as<DroneStateSpace::StateType>()->Yaw());
				vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(numbers[i + 1])->state->as<DroneStateSpace::StateType>()->CameraAngle());

				auto tempValid = planner->CheckEdge(parentVertex->state, vertex->state);
				edgeValid.push_back(tempValid);
				if (!tempValid)
				{
					countCollision++;
				}

				// vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(i)->state->as<DroneStateSpace::StateType>()->Yaw());
				// vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(i)->state->as<DroneStateSpace::StateType>()->CameraAngle());
				// parentVertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(i + 1)->state->as<DroneStateSpace::StateType>()->Yaw());
				// parentVertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(i + 1)->state->as<DroneStateSpace::StateType>()->CameraAngle());

				// space_info_->copyState(vertex->state, graph->Vertex(i + 1)->state);
				// space_info_->copyState(parentVertex->state, graph->Vertex(i)->state);

				// // std::cout << "parent:" << std::endl;
				// std::cout << parentPosition << std::endl;
				// // std::cout << "childPosition:" << std::endl;
				// std::cout << childPosition << std::endl;
				// // std::cout << "distance" << space_info_->distance(parentVertex->state, vertex->state) << std::endl;

				total_length += space_info_->distance(parentVertex->state, vertex->state);
				// std::cout << "total_length : " << total_length << std::endl;
			}

			for (size_t j = 0; j < MAX_COVERAGE_SIZE; j++)
			{

				// todo make bitset_ private
				if (virtual_graphcoverage_.bitset_[j] < 0.5)
				{
					parentVertex->vis.bitset_[j] = 0;
				}
			}
			graph->UpdateGlobalVisibility(parentVertex->vis);
		}

		// graph->Save(file_to_read, false);

		// std::cout << "CoveragePOIs is : " << graph->NumTargetsCovered() << "\n"
		// 		  << "Number of collision is: " << countCollision << "\n"
		// 		  << "length: " << total_length << std::endl;

		// std::ofstream foutMC(file_to_read + "_MC");

		// if (!foutMC.is_open())
		// {
		// 	std::cout << "Vertex file cannot be opened!" << std::endl;
		// 	exit(1);
		// }

		fout << graph->NumTargetsCovered() << " " << total_length << " " << countCollision;
		for (size_t i = 0; i < edgeValid.size(); i++)
		{
			fout << " " << !edgeValid[i];
		}
		fout << " " << std::endl;
	}
	fout.close();
	return 0;
}