#ifndef OMPL_SE3_STATE_SPACE_ALIGNED_
#define OMPL_SE3_STATE_SPACE_ALIGNED_
/** Alternative implementation of SE3 state space.
 * 
 * This implementation uses a RealVectorStateSpace as base class,
 * intead of using a compound state space of a position and orientation.
 * This is a requirement to use the state space in constrained planning problems.
 **/

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {

        class SE3StateSpaceAligned : public RealVectorStateSpace
        {
        public:
            SE3StateSpaceAligned() : RealVectorStateSpace(7)
            {
                type_ = STATE_SPACE_SE3;
                VEC_DIM_ = 3; // TODO Hardcoded at the moment
                QUAT_DIM_ = 4;
            }

            void enforceBounds(State *state) const override;
            double distance(const State *state1, const State *state2) const override;
            void interpolate(const State *from, const State *to, double t, State *state) const override;
            StateSamplerPtr allocDefaultStateSampler() const override;

            std::size_t VEC_DIM_;  // Length of the vector represening positions
            std::size_t QUAT_DIM_; // Length of the tail of the vector representing quaternions
        };

        class SE3StateSpaceAlignedSampler : public RealVectorStateSampler
        {
        public:
            SE3StateSpaceAlignedSampler(const StateSpace *space) : RealVectorStateSampler(space)
            {
            }
            void sampleUniform(State *state) override;
            void sampleUniformNear(State *state, const State *near, double distance) override;
            //void sampleGaussian(State *state, const State *mean, double stdDev) override;
        };

    } // namespace base
} // namespace ompl

#endif /* OMPL_SE3_STATE_SPACE_ALIGNED_ */