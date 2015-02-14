#include "chainiksolverpos_nr_jl_position_only.hpp"
namespace KDL
 {
     ChainIkSolverPos_NR_JL_PositionOnly::ChainIkSolverPos_NR_JL_PositionOnly(const Chain& _chain, const JntArray& _q_min, const JntArray& _q_max, ChainFkSolverPos& _fksolver,ChainIkSolverVel& _iksolver,
                                              unsigned int _maxiter, double _eps):
         chain(_chain), q_min(chain.getNrOfJoints()), q_max(chain.getNrOfJoints()), fksolver(_fksolver),iksolver(_iksolver),delta_q(_chain.getNrOfJoints()),
         maxiter(_maxiter),eps(_eps)
     {
         q_min = _q_min;
         q_max = _q_max;
     }
 
     int ChainIkSolverPos_NR_JL_PositionOnly::CartToJnt(const JntArray& q_init, const Frame& p_in, JntArray& q_out)
     {
             q_out = q_init;
 
             unsigned int i;
             for(i=0;i<maxiter;i++){
                 fksolver.JntToCart(q_out,f);
                 delta_twist = diff(f,p_in);
                 
                 // Clear the rotation
                 delta_twist.rot = Vector::Zero();
                 if(Equal(delta_twist,Twist::Zero(),eps))
                     break;
 
                 iksolver.CartToJnt(q_out,delta_twist,delta_q);
                 Add(q_out,delta_q,q_out);
 
                 for(unsigned int j=0; j<q_min.rows(); j++) {
                   if(q_out(j) < q_min(j))
                     q_out(j) = q_min(j);
                 }
 
 
                 for(unsigned int j=0; j<q_max.rows(); j++) {
                     if(q_out(j) > q_max(j))
                       q_out(j) = q_max(j);
                 }
             }
 
             if(i!=maxiter)
                 return 0;
             else
                 return -3;
     }
 
     ChainIkSolverPos_NR_JL_PositionOnly::~ChainIkSolverPos_NR_JL_PositionOnly()
     {
     }
 
 }
 
