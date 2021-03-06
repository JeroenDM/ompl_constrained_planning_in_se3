/** OMPL Constrained planning with multiple constraints
 * 
 * This example shows how we can use ompl::base::ConstraintIntersection
 * to stack two different constraints, instead of combining them in
 * a single constraint (see constraint_se3_planning.cpp).
 * */
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

/** Obstacle from the constrained planning tutorial
 *  https://ompl.kavrakilab.org/constrainedPlanningTutorial.html
 * */
bool obstacle(const ob::State *state)
{
    const Eigen::Map<Eigen::VectorXd> &x = *state->as<ob::ConstrainedStateSpace::StateType>();
    if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    return true;
}

/** Constraints from the constrained planning tutorial
 *  https://ompl.kavrakilab.org/constrainedPlanningTutorial.html
 * 
 * Analytical Jacobian not added, the default numerical one works just fine
 * for this simple example.
 * */
class Sphere : public ob::Constraint
{
public:
    Sphere() : ob::Constraint(7, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        // position constrained to sphere with radius 1
        out[0] = x.head(3).norm() - 1;
    }
};

/** Constraints on rotatation around X and Y axis. * */
class ZRotation : public ob::Constraint
{
public:
    ZRotation() : ob::Constraint(7, 2)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        // only rotation around z-axis allowed
        out[0] = x[3];
        out[1] = x[4];
    }
};

void planWithConstraints()
{

    // the space is based on ob::RealVectorStateSpace>(7)
    auto space(std::make_shared<ob::SE3StateSpaceAligned>());

    // the bounds on the quaternion part are not used (at least that I know of)
    // the quaterion is normalized when bounds are enforced
    ob::RealVectorBounds bounds(7);
    bounds.setLow(-2);
    bounds.setHigh(2);
    space->setBounds(bounds);

    std::vector<ob::ConstraintPtr> constraints(2);
    constraints[0] = std::make_shared<Sphere>();
    constraints[1] = std::make_shared<ZRotation>();
    auto constraint_intersection = std::make_shared<ob::ConstraintIntersection>(7, constraints);

    // problem setup stuff
    auto css = std::make_shared<ob::ProjectedStateSpace>(space, constraint_intersection);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
    auto ss = std::make_shared<og::SimpleSetup>(csi);
    auto pp = std::make_shared<og::PRM>(csi);
    ss->setPlanner(pp);

    // Add the obstacles, the implemetation also uses casting of a state
    // to a real vector state space in this case
    // I could cast to an eigen position and quaternion -> eigen transform
    // for collision checking in the future for more interesting examples
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

        // write the interpolated path to a file for visualization (using Python)
        std::ofstream file;
        file.open("path_se3.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
    {
        OMPL_WARN("No solution found!");
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    planWithConstraints();
    return 0;
}