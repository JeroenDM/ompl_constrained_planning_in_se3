#include "SE3StateSpaceAligned.h"

#include <boost/math/constants/constants.hpp>

#include <ompl/base/State.h>
#include "QuaterionUtil.h"

namespace ompl
{
    namespace base
    {
        void ImplicitChainSpace::enforceBounds(State *state) const
        {
            auto *rstate = static_cast<StateType *>(state);
            // enforce bounds on vector part
            for (unsigned int i = 0; i < VEC_DIM_; ++i)
            {
                if (rstate->values[i] > bounds_.high[i])
                    rstate->values[i] = bounds_.high[i];
                else if (rstate->values[i] < bounds_.low[i])
                    rstate->values[i] = bounds_.low[i];
            }

            // enforce unit norm on quaternion part
            // how can I do this better, implement normize function
            // that takes raw array input
            ompl::util::Quaternion qstate(&rstate->values[VEC_DIM_]);
            ompl::util::normalizeQuaternion(qstate);
            rstate->values[VEC_DIM_] = qstate.x;
            rstate->values[VEC_DIM_ + 1] = qstate.y;
            rstate->values[VEC_DIM_ + 2] = qstate.z;
            rstate->values[VEC_DIM_ + 3] = qstate.w;
        }

        ompl::base::StateSamplerPtr ImplicitChainSpace::allocDefaultStateSampler() const
        {
            return std::make_shared<ImplicitChainSampler>(this);
        }

        double ImplicitChainSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
        {
            double vec_dist = 0.0, quat_dist = 0.0;
            const double *s1 = static_cast<const StateType *>(state1)->values;
            const double *s2 = static_cast<const StateType *>(state2)->values;

            // sum squared distance for vector part
            for (unsigned int i = 0; i < VEC_DIM_; ++i)
            {
                double diff = (*s1++) - (*s2++);
                vec_dist += diff * diff;
            }
            vec_dist = sqrt(vec_dist);

            // some kind of arc length for the quaternion part
            for (unsigned int i = 0; i < QUAT_DIM_; ++i)
            {
                quat_dist += (*s1++) * (*s2++);
            }
            quat_dist = fabs(quat_dist);
            if (quat_dist > (1.0 - ompl::util::MAX_QUATERNION_NORM_ERROR))
                quat_dist = 0.0;
            else
                quat_dist = acos(quat_dist);

            // std::cout << "vec: " << vec_dist << " quat: " << quat_dist << std::endl;
            return vec_dist + quat_dist;
        }

        void ImplicitChainSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const
        {
            const auto *rfrom = static_cast<const StateType *>(from);
            const auto *rto = static_cast<const StateType *>(to);
            const StateType *rstate = static_cast<StateType *>(state);

            // vector part
            for (unsigned int i = 0; i < VEC_DIM_; ++i)
                rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;

            // quaternion part
            ompl::util::Quaternion qFrom(&rfrom->values[VEC_DIM_]), qTo(&rto->values[VEC_DIM_]), qResult;
            ompl::util::interpolateQuaternions(qFrom, qTo, t, qResult);
            rstate->values[VEC_DIM_] = qResult.x;
            rstate->values[VEC_DIM_ + 1] = qResult.y;
            rstate->values[VEC_DIM_ + 2] = qResult.z;
            rstate->values[VEC_DIM_ + 3] = qResult.w;
        }

        void ImplicitChainSampler::sampleUniform(ompl::base::State *state)
        {
            const ompl::base::RealVectorBounds &bounds = static_cast<const ImplicitChainSpace *>(space_)->getBounds();

            auto *rstate = static_cast<ImplicitChainSpace::StateType *>(state);
            //auto *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType *>(state);

            // sample vector part
            for (unsigned int i = 0; i < 3; ++i)
                rstate->values[i] = rng_.uniformReal(bounds.low[i], bounds.high[i]);

            rng_.quaternion(&rstate->values[3]);
        }

        void ImplicitChainSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance)
        {
            const ompl::base::RealVectorBounds &bounds = static_cast<const ImplicitChainSpace *>(space_)->getBounds();
            auto *rstate = static_cast<ompl::base::RealVectorStateSpace::StateType *>(state);
            const auto *rnear = static_cast<const ompl::base::RealVectorStateSpace::StateType *>(near);

            // first check for special case
            // TODO this distance is wrong because only for orientation part
            if (distance >= .25 * boost::math::constants::pi<double>())
            {
                sampleUniform(state);
                return;
            }
            else
            {
                // sample position
                for (unsigned int i = 0; i < 3; ++i)
                {
                    rstate->values[i] = rng_.uniformReal(std::max(bounds.low[i], rnear->values[i] - distance),
                                                         std::min(bounds.high[i], rnear->values[i] + distance));
                }

                // sample orientation
                ompl::util::Quaternion qState, q, qs;
                const ompl::util::Quaternion qNear = {rnear->values[3], rnear->values[4], rnear->values[5], rnear->values[6]};
                qState = {rstate->values[3], rstate->values[4], rstate->values[5], rstate->values[6]};

                double d = rng_.uniform01();
                ompl::util::computeAxisAngle(q, rng_.gaussian01(), rng_.gaussian01(), rng_.gaussian01(),
                                             2. * pow(d, boost::math::constants::third<double>()) * distance);

                qs = qNear * q;
                rstate->values[3] = qs.x;
                rstate->values[4] = qs.y;
                rstate->values[5] = qs.z;
                rstate->values[6] = qs.w;
                return;
            }
        }
    } // namespace base
} // namespace ompl