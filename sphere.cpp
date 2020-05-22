#include <iostream>
#include <fstream>
#include <memory>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Sphere : public ob::Constraint
{
public:
    Sphere() : ob::Constraint(3, 1)
    {
    }

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;
};

void Sphere::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const
{
    // The function that defines a sphere is f(q) = || q || - 1, as discussed
    // above. Eigen makes this easy to express:
    out[0] = x.norm() - 1;
}

void Sphere::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const
{
    out = x.transpose().normalized();
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

int main()
{
    std::cout << "OMPL Sphere tutorial" << std::endl;

    // ambient space to plan in, this is where my se3 space will come in
    auto rvss = std::make_shared<ob::RealVectorStateSpace>(3);
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-2);
    bounds.setHigh(2);
    rvss->setBounds(bounds);

    // constrained state space, this is independent of the above,
    // but the above is always handles as a real vector space
    // you have to know what the elements of this vector represent.
    auto constraint = std::make_shared<Sphere>();

    // problem setup stuff
    auto css = std::make_shared<ob::ProjectedStateSpace>(rvss, constraint);
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
    Eigen::VectorXd sv(3), gv(3);
    sv << 0, 0, -1;
    gv << 0, 0,  1;
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
        file.open("path.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
        OMPL_WARN("No solution found!");
}