#ifndef KDLCHAINIKSOLVERPOS_NR_JL_POSITION_ONLY_HPP
#define KDLCHAINIKSOLVERPOS_NR_JL_POSITION_ONLY_HPP

#include <kdl/chainiksolver.hpp>
#include <kdl/chainfksolver.hpp>

namespace KDL {
     class ChainIkSolverPos_NR_JL_PositionOnly : public ChainIkSolverPos
     {
     public:
         ChainIkSolverPos_NR_JL_PositionOnly(const Chain& chain,const JntArray& q_min, const JntArray& q_max, ChainFkSolverPos& fksolver,ChainIkSolverVel& iksolver,unsigned int maxiter=100,double eps=1e-6);
         ~ChainIkSolverPos_NR_JL_PositionOnly();
 
         virtual int CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out);

     private:
         const Chain chain;
         JntArray q_min;
         JntArray q_max;
         ChainIkSolverVel& iksolver;
         ChainFkSolverPos& fksolver;
         JntArray delta_q;
         Frame f;
         Twist delta_twist;
 
         unsigned int maxiter;
         double eps;
     };
 
}
#endif

