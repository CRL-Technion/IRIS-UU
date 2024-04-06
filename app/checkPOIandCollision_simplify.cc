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
	rng.seed(seed + 13);
	// Robot

	std::ifstream fin;
	String Location_Error_file_name;
	if (argc > 4)
	{
		Location_Error_file_name = argv[4];
	}
	else
	{
		std::cerr << "LocationErrorParameters file cannot be found!" << std::endl;
		exit(1);
	}
	// String fileLocationError ="LocationErrorParameterFile";
	fin.open(Location_Error_file_name);
	if (!fin.is_open())
	{
		std::cerr << "LocationErrorParameters file cannot be opened!" << std::endl;
		exit(1);
	}

	// String line;
	Idx i = 0;

	RealNum b_a_milli_g = 0.0;
	RealNum b_g_degPerHr = 0.0;
	RealNum avarageVelocity = 0.0;
	RealNum minTimeAllowInRistZone = 0.0;
	RealNum maxTimeAllowInRistZone = 0.0;
	RealNum multipleCostFunction = 1.0;
	RealNum Threshold_p_coll = 1.0;
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
		Idx MonteCarloNumber = std::stod(line);
		fin >> line;
		Threshold_p_coll = std::stod(line);
		break;
	}

	auto milli_g2mpss = 9.81 / 1000.0;				   //   Conversion from [mili g ] to [m/s^2]
	auto degPerHr2radPerSec = (3.14 / 180.0) / 3600.0; //  Conversion from [deg/hr] to [rad/s]
	auto b_a = b_a_milli_g * milli_g2mpss;
	auto b_g = b_g_degPerHr * degPerHr2radPerSec;

	RealNum ba_x;
	RealNum ba_y;
	RealNum ba_z;
	RealNum bg_x;
	RealNum bg_y;
	RealNum bg_z;

	RealNormalDist Norm1(0, sqrt(b_a) / 4);
	ba_x = (Norm1(rng));
	ba_y = (Norm1(rng));
	ba_z = (Norm1(rng));

	RealNormalDist Norm2(0, sqrt(b_g) / 4);
	bg_x = (Norm2(rng));
	bg_y = (Norm2(rng));
	bg_z = (Norm2(rng));

	auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, 0);
	robot->SetCameraParameters(94.0 / 180 * M_PI, 0.2, 10.0);
	robot->Initialize();

	// Environment setup.
	auto env = std::make_shared<drone::BridgeEnvironment>();

	// Planner
	auto planner = std::make_shared<drone::DronePlanner>(robot, env, seed);
#if ToyProblem == 0
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
		RealNum perviousCostRiskZone=0.0;
		for (size_t k = 0; k < 3; k++)
		{
			previousTotalLocationError[k] = 0;
		}
		for (size_t k = 0; k < 3; k++)
		{
			previousTotalLocationError[k] = 0;
		}

		for (size_t i = 0; i < numbers.size(); i++)
		{

			auto parentPosition = graph->Vertex(numbers[i])->state->as<DroneStateSpace::StateType>()->Position();
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
				RealNum sigmaNoise = 1;
				Vec3 childPositionFix;

				if (!planner->IsPointInsideBox(childPosition))
				{
					RealNormalDist NormR(0, 1);
					RealNormalDist NormAngle(0, 2 * M_PI);
					auto r = abs(NormR(rng)) * sigmaNoise;
					auto azimuth = NormAngle(rng);
					auto elevation = NormAngle(rng);

					previousTotalLocationError[0] = r * cos(elevation) * cos(azimuth);
					previousTotalLocationError[1] = r * cos(elevation) * sin(azimuth);
					previousTotalLocationError[2] = -r * sin(elevation);
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

					total_length += space_info_->distance(parentVertex->state, vertex->state);
				}
				else
				{
					if (!planner->IsPointInsideBox(parentPosition)) // if q^c_{i-1} \in w_cov , find intersect point
					{
						// auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPositionFix, childPositionFix, insertPoint);
						Vec3 insertPoint;
						auto IsInsertToRiskZone = planner->FindInsertPointRiskZone(parentPosition, childPosition, insertPoint);
						parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPositionFix);
						vertex->state->as<DroneStateSpace::StateType>()->SetPosition(insertPoint);
						total_length += space_info_->distance(parentVertex->state, vertex->state);

						// update parentPositionFix to be the insert point
						parentPositionFix[0] = insertPoint[0];
						parentPositionFix[1] = insertPoint[1];
						parentPositionFix[2] = insertPoint[2];
					}

					for (size_t k = 0; k < 3; k++)
					{
						childPositionFix[k] = childPosition[k] + previousTotalLocationError[k];
					}

					parentVertex->state->as<DroneStateSpace::StateType>()->SetPosition(parentPositionFix);
					vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPositionFix);
					//  This is an approximation for calculate "currentTimeRiskZone"

					auto perviousTimeRiskZone = perviousCostRiskZone;

					auto currentTimeRiskZone = perviousTimeRiskZone + space_info_->distance(parentVertex->state, vertex->state);

					auto x = childPositionFix[0] - parentPositionFix[0];
					auto y = childPositionFix[1] - parentPositionFix[1];
					auto z = childPositionFix[2] - parentPositionFix[2];
					// todo psi and theta
					RealNum psi = atan2(y, x);
					RealNum theta = -atan2(z, sqrt(x * x + y * y));

					auto temp_x_error = ba_x * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
					temp_x_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_z + bg_y);

					auto temp_y_error = ba_y * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
					temp_y_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (bg_z - bg_x);

					auto g = 9.81;
					auto temp_z_error = ba_z * (1.0 / 2.0) * (currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone);
					temp_z_error += (1.0 / 6.0) * (currentTimeRiskZone * currentTimeRiskZone * currentTimeRiskZone - perviousTimeRiskZone * perviousTimeRiskZone * perviousTimeRiskZone) * (-bg_y + bg_x) * (-g);

					previousTotalLocationError[0] += cos(theta) * cos(psi) * temp_x_error - sin(psi) * temp_y_error + sin(theta) * cos(psi) * temp_z_error;
					previousTotalLocationError[1] += cos(theta) * sin(psi) * temp_x_error + cos(psi) * temp_y_error + sin(theta) * sin(psi) * temp_z_error;
					previousTotalLocationError[2] += -sin(theta) * temp_x_error + cos(theta) * temp_z_error;

					// update childPositionFix with IMUNoise
					childPositionFix[0] = childPosition[0] + previousTotalLocationError[0];
					childPositionFix[1] = childPosition[1] + previousTotalLocationError[1];
					childPositionFix[2] = childPosition[2] + previousTotalLocationError[2];

					vertex->state->as<DroneStateSpace::StateType>()->SetPosition(childPositionFix);
					vertex->state->as<DroneStateSpace::StateType>()->SetYaw(graph->Vertex(numbers[i + 1])->state->as<DroneStateSpace::StateType>()->Yaw());
					vertex->state->as<DroneStateSpace::StateType>()->SetCameraAngle(graph->Vertex(numbers[i + 1])->state->as<DroneStateSpace::StateType>()->CameraAngle());

					perviousCostRiskZone += space_info_->distance(parentVertex->state, vertex->state);
					total_length += space_info_->distance(parentVertex->state, vertex->state);

					auto tempValid = planner->CheckEdge(parentVertex->state, vertex->state);
					edgeValid.push_back(tempValid);
					if (!tempValid)
					{
						countCollision++;
					}
				}

				// reset previousTotalLocationError - RandomNoiseGNSS

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