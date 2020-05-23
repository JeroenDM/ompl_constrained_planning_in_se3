#include <iostream>
#include <fstream>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include "SE3StateSpaceAligned.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // const auto *vec = state->as<ob::RealVectorStateSpace::StateType>();
    // auto v = Eigen::Map<Eigen::VectorXd>(vec->values, 7);

    // Eigen::Vector3d point;
    // point << 0, 0, 0;

    // if ((v.head(3) - point).norm() < 0.5)
    // {
    //     return false;
    // }
    return true;
}

bool obstacle(const ob::State *state)
{
    // As ob::ConstrainedStateSpace::StateType inherits from
    // Eigen::Map<Eigen::VectorXd>, we can grab a reference to it for some easier
    // state access.
    const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();
    // Alternatively, we could access the underlying real vector state with the
    // following incantation:
    //   auto x = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<ob::RealVectorStateSpace::StateType>();
    // Note the use of "getState()" on the constrained state. This accesss the
    // underlying state that was allocated by the ambient state space.
    // Define a narrow band obstacle with a small hole on one side.
    if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    return true;
}

void planWithoutConstraints()
{

    // auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    auto space(std::make_shared<ob::SE3StateSpaceAligned>());

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
    start = {1, 0, 0, 0, 0, 0, 1};
    ob::ScopedState<> goal(space);
    // goal.random();
    goal = {-1, 0, 0, 0, 0, 0, 1};
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

class Sphere : public ob::Constraint
{
public:
    Sphere() : ob::Constraint(7, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    // void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;
};

void Sphere::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // The function that defines a sphere is f(q) = || q || - 1, as discussed
    // above. Eigen makes this easy to express:
    out[0] = x.head(3).norm() - 1;
}

// void Sphere::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
// {
//     out.head(3) = x.head(3).transpose().normalized();
//     // out[3] = 0.0;
//     // out[4] = 0.0;
//     // out[5] = 0.0;
// }

void planWithConstraints()
{

    // auto space(std::make_shared<ob::RealVectorStateSpace>(7));
    auto space(std::make_shared<ob::SE3StateSpaceAligned>());

    ob::RealVectorBounds bounds(7);
    bounds.setLow(-2);
    bounds.setHigh(2);
    space->setBounds(bounds);

    auto constraint = std::make_shared<Sphere>();

    // problem setup stuff
    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    auto ss = std::make_shared<og::SimpleSetup>(csi);
    auto pp = std::make_shared<og::PRM>(csi);
    ss->setPlanner(pp);

    // add the obstacles, the implemetation also uses casting of a state
    // to a real vector state space in this case
    // I could cast to an eigen position and quaternion -> eigen transform
    // for collision checking
    ss->setStateValidityChecker(obstacle);

    // problem specific admin
    Eigen::VectorXd sv(7), gv(7);
    sv << 0, 0, 1, 0, 0, 0, 1;
    gv << 0, 0, -1, 0, 0, 0, 1;
    ob::ScopedState<> start(css);
    ob::ScopedState<> goal(css);
    start->as<ob::ConstrainedStateSpace::StateType>()->copy(sv);
    goal->as<ob::ConstrainedStateSpace::StateType>()->copy(gv);
    ss->setStartAndGoalStates(start, goal);

    // solving it
    ss->setup();
    ob::PlannerStatus stat = ss->solve(5.);
    if (stat)
    {
        // Path simplification also works when using a constrained state space!
        ss->simplifySolution(5.);
        auto path = ss->getSolutionPath();
        path.interpolate();

        std::ofstream file;
        file.open("path_se3.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
        OMPL_WARN("No solution found!");
    
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
    // planWithoutConstraints();
    planWithConstraints();

    return 0;
}