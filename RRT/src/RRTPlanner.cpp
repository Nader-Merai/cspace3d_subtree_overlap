/*
 * This is a RRT planner.
 *
 * Date: Aug/20/2018
 * Author: Wei Du
 *
 * Version: 1.0
 *
 */

#include "include/RRTPlanner.h"


RRTPlanner::RRTPlanner() {
    numOfSamples = 0;
    env = nullptr;
    goalstateid = -1;
    startstateid = -1;
} 

RRTPlanner::RRTPlanner(EnvironmentNAVXYTHETAC* env_, int numOfSamples_):env(env_), numOfSamples(numOfSamples_) {
    if (nullptr == env_){
        std::cerr << "env does not exist." << std::endl;
        exit(0);
    }

    if (numOfSamples <= 10) {
        std::cerr << "Number of samples are too small." << std::endl;
        exit(0);
    }

    goalstateid = env_->GetGoalStateID();
    startstateid = env_->GetStartStateID();
}

RRTPlanner::~RRTPlanner(){
    if (nullptr != env) {
        env = nullptr;
    }
}

bool RRTPlanner::Run()  {

    std::clock_t samTime = std::clock();

    int start_tree = Init(startstateid);
    if (start_tree) {

        for (int k = 0; k < numOfSamples; ++k ) {

            int s_randID = Random_state();

            auto ext = Extend(start_tree, s_randID);

            if(ext == Goal){
                return true;
            }

            if( (std::clock() - samTime )/CLOCKS_PER_SEC > 120 ){
                break;
            }
        }
        return false;
    }   

    return false;
}


/*
 *bool RRTPlanner::GoalReached(int stateID) {
 *    return env->IsWithinGoalRegion(stateID);
 *}
 */

int RRTPlanner::Init(int stateID) {
    return env->InitTree(stateID);
}

int RRTPlanner::Random_state() {
    return env->GetRandomSample();
}

EXT RRTPlanner::Extend(int treeID, int sampleStateID) {

    int s_near = NearestNeighbor(treeID, sampleStateID);

    int s_new = NewConfig(s_near, sampleStateID);
    
    if(s_new == goalstateid) return Goal;

    if (-1 != s_new) {

        if(s_new != sampleStateID) {
            env->PrintStateExpanded(s_new);
            return Advanced;
        } else {
            return Reached;
        }
    }
    return Trapped;
}

int RRTPlanner::NearestNeighbor(int treeID, int queryStateID) {
    return env->GetNN(treeID, queryStateID);
}

int RRTPlanner::NewConfig(int parentID, int sampleStateID) {
    return env->GoTo(parentID, sampleStateID);
}


