#include <iostream>
#include <fstream>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "SE3StateSpaceAligned.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    return true;
}

void planWithConstraints()
{

    // auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    auto space(std::make_shared<ob::ImplicitChainSpace>());

    ob::RealVectorBounds bounds(7);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

     // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    ob::ScopedState<> start(space);
    // start.random();
    start = {0, 0, 0, 0, 0, 0, 1};
    ob::ScopedState<> goal(space);
    goal.random();
    // goal = {1, 1, 1, 0, 0, 0, 1};
    ss.setStartAndGoalStates(start, goal);


    ss.setup();
    ss.print();
    ob::PlannerStatus solved = ss.solve(1.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        auto path = ss.getSolutionPath();
        path.interpolate();

        std::ofstream file;
        file.open("path_se3.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
        std::cout << "No solution found" << std::endl;


    
}

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();
    ob::ScopedState<> goal(space);
    goal.random();
    ss.setStartAndGoalStates(start, goal);


    ss.setup();
    ss.print();
    ob::PlannerStatus solved = ss.solve(1.0);
    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.simplifySolution();
        auto path = ss.getSolutionPath();
        path.interpolate();

        std::ofstream file;
        file.open("path_se3.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // planWithSimpleSetup();
    planWithConstraints();

    return 0;
}