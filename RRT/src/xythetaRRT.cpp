#include <cstring>
#include <iostream>
#include <string>
#include <ctime>
#include <vector>

#include "include/RRTPlanner.h"
#include "include/environment_navxythetaC.h"

using namespace std;

// creating the footprint
void createFootprint(vector<sbpl_2Dpt_t>& perimeter){
    sbpl_2Dpt_t pt_m;
    double halfwidth = 0.025;
    double halflength = 0.025;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeter.push_back(pt_m);
}

void initializeEnv(EnvironmentNAVXYTHETAC& env,
                   vector<sbpl_2Dpt_t>& perimeter,
                   char* envCfgFilename, char* motPrimFilename){
  if (!env.InitializeEnv(envCfgFilename, perimeter, motPrimFilename)) {
        printf("ERROR: InitializeEnv failed\n");
       
    }
}

void setEnvStartGoal(EnvironmentNAVXYTHETAC& env,
                     double start_x, double start_y, double start_theta,
                     double goal_x, double goal_y, double goal_theta,
                     int& start_id, int& goal_id){

    env.SetGoalTolerance(1.5, 1.5, 6.2832); 
    start_id = env.SetStart(start_x, start_y, start_theta);
    goal_id = env.SetGoal(goal_x, goal_y, goal_theta);
}


void initializePlanner(RRTPlanner* &planner,
                       EnvironmentNAVXYTHETAC& env) { 

    // number of samples
    planner = new RRTPlanner(&env, 500000);

}

int runPlanner(RRTPlanner* planner) {
    
    clock_t start = clock();

    int bRet = planner->Run();

    clock_t end = clock();
    float diff = float(end- start)/CLOCKS_PER_SEC;
    std::cout << "Time: " << diff << " s" << std::endl;

    if (bRet)
        printf("Solution is found\n");
    else
        printf("Solution does not exist\n");
    return bRet;
}

void writeSolution(EnvironmentNAVXYTHETAC& env, vector<int> solution_stateIDs,
                   const char* filename){

    std::string discrete_filename(std::string(filename) + std::string(".discrete"));
    FILE* fSol_discrete = fopen(discrete_filename.c_str(), "w");
    FILE* fSol = fopen(filename, "w");
    if (fSol == NULL) {
        printf("ERROR: could not open solution file\n");
        throw SBPL_Exception();
    }
    // write the continuous solution to file
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    env.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &xythetaPath);
    for (unsigned int i = 0; i < xythetaPath.size(); i++) {
        fprintf(fSol, "%.3f %.3f %.3f\n", xythetaPath.at(i).x,
                                          xythetaPath.at(i).y,
                                          xythetaPath.at(i).theta);
    }
    fclose(fSol);
    // write the discrete solution to file
    for (size_t i = 0; i < solution_stateIDs.size(); i++) {
        double x, y, theta;
        env.GetCoordFromState(solution_stateIDs[i], x, y, theta);
        int cont_x, cont_y, cont_theta;
        cont_x = DISCXY2CONT(x, 0.025);
        cont_y = DISCXY2CONT(y, 0.025);
        cont_theta = DiscTheta2Cont(theta, 16);

        int dsctx, dscty, dsctheta;
        dsctx = CONTXY2DISC(x, env.EnvNAVXYTHETACCfg.cellsize_m);
        dscty = CONTXY2DISC(y, env.EnvNAVXYTHETACCfg.cellsize_m);
        dsctheta = env.ContTheta2DiscNew(theta);
        fprintf(fSol_discrete, "%d %d %d\n", dsctx, dscty, dsctheta);
    }
    fclose(fSol_discrete);
}

// read and store star & goal info
void ReadinStartGoal(char* SGFilename, std::vector<double> &startgoal){
    startgoal.clear();
    ifstream sg;
    std::string line;
    std::string buf;

    sg.open(SGFilename);
    if(!sg.is_open()) {
        std::cout << "SGFile open failed." << std::endl;
        exit(0);
    }
    
    while(getline(sg, line)) {
        std::stringstream ss(line);
        ss >> buf;
        while (ss >> buf)
            startgoal.push_back(atof(buf.c_str()));
    }
    
    if ( 7 != startgoal.size() ){
        std::cout << "Wrong .sg file format, parameters missing!" << std::endl;
        exit(0);
    }
}

// RRT
void planxythetalat(char* envCfgFilename, char* SGFilename, char*  motPrimFilename){

    // set the perimeter of the robot
    vector<sbpl_2Dpt_t> perimeter;
    createFootprint(perimeter);
    std::vector<double> startgoal;

    // initialize an environment
    // continuousspace environment
    EnvironmentNAVXYTHETAC env;
    initializeEnv(env, perimeter, envCfgFilename, motPrimFilename);

    ReadinStartGoal(SGFilename, startgoal);

    // specify a start and goal state
    int start_id, goal_id;

    setEnvStartGoal(env,
            startgoal[0], startgoal[1], startgoal[2], 
            startgoal[3], startgoal[4], startgoal[5],
            start_id, goal_id);


    // initialize a planner with start and goal state
    RRTPlanner* planner = NULL;

    initializePlanner(planner, env);

   
    // plan
    vector<int> solution_stateIDs;
    runPlanner(planner);

    // print stats
    env.PrintTimeStat(stdout);

    // write out solutions
    std::string filename("sol.txt");
    writeSolution(env, solution_stateIDs, filename.c_str());

    delete planner;
}

int main(int argc, char *argv[])
{   
    planxythetalat(argv[1], argv[2], argv[3]);
}
