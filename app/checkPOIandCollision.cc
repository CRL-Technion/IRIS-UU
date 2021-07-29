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

using VPtr = std::shared_ptr<Inspection::Vertex>;
using EPtr = std::shared_ptr<Inspection::Edge>;


int main(int argc, char** argv) {
	
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


	//Robot
    auto robot = std::make_shared<drone::DroneRobot>(0.196, 0.2895, -0.049);
    robot->SetCameraParameters(94.0/180 * M_PI, 0.2, 10.0);
    robot->Initialize();

	
    // Environment setup.
    auto env = std::make_shared<drone::BridgeEnvironment>();

    // Planner
    auto planner = std::make_shared<drone::DronePlanner>(robot, env, seed);
	Idx countCollision = 0;

	for (size_t i = 0; i < graph->NumVertices(); i++)
	{
		planner->ComputeVisibilitySet(graph->Vertex(i));
        graph->UpdateGlobalVisibility(graph->Vertex(i)->vis);
		if (i<graph->NumVertices()-1)
		{
			graph->Edge(i)->valid =planner->CheckEdge(graph->Vertex(i)->state,graph->Vertex(i+1)->state);
			if (!graph->Edge(i)->valid)
			{
				countCollision++;
			}
		}
	
	}
	graph->Save(file_to_read, false);

	std::cout << "CoveragePOIs is : " << graph->NumTargetsCovered() << "\n"
        << "Number of collision is: " << countCollision
        << std::endl;

	

    //delete &graph;


	
	

// #endif
// #endif

	return 0;
}