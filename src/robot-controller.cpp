#include <Ravelin/MatrixNd.h>
#include <Ravelin/LinAlgd.h>
#include <Ravelin/VectorNd.h>
#include <Opt/QPActiveSet.h>

using namespace Ravelin;

/**
 * \param M the generalized inertia matrix of the humanoid
 * \param vminus the generalized velocity of the humanoid at the current time
 * \param fext the generalized external force vector of the humanoid at the
 *             current time
 * \param N the generalized normal contact wrench for the humanoid 
 * \param D the generalized tangent contact wrench for the humanoid 
 * \param R the generalized constraint wrench for the humanoid
 * \param J the generalized constraint wrench for the robot
 * \param lolimit the lower (negative) actuator limits imposed on the robot
 * \param hilimit the upper (positive) actuator limits imposed on the robot
 * \param fr contains, on return, the force that the robot should apply to the human at the point of contact
 */
void controller(const MatrixNd& M, const VectorNd& v, const VectorNd& fext, const MatrixNd& N, const MatrixNd& D, const MatrixNd& R, const MatrixNd& J, const VectorNd& lolimit, const VectorNd& hilimit, VectorNd& fr)
{
  const double INF = std::numeric_limits<double>::max();
  const double DT = 1e-3;  // default value for delta t

  // setup variable sizes
  const unsigned N_ACT = J.columns();    // number of actuators robot commands
  const unsigned N_CONTACTS = N.rows();
  const unsigned N_SIZE = N_CONTACTS;
  const unsigned D_SIZE = N_CONTACTS*2;  // two tangent directions 
  const unsigned R_SIZE = R.rows();      // 6 x X robot/humanoid constraints 

  // setup indices for variables
  const unsigned R_INDEX = 0;
  const unsigned D_INDEX = R_INDEX + R_SIZE;
  const unsigned N_INDEX = D_INDEX + D_SIZE;

  // setup the number of QP variables
  const unsigned NVARS = N_SIZE + R_SIZE + D_SIZE;

  // do a Cholesky factorization on M so that we can solve with it quickly
  MatrixNd cholM = M;
  if (!LinAlgd::factor_chol(cholM))
    throw std::runtime_error("Unexpectedly unable to factorize generalized inertia matrix!");
 
  // compute k = v^- + inv(M)*fext*DT
  VectorNd k = fext;
  k *= DT;
  LinAlgd::solve_chol_fast(cholM, k);
  k += v; 

  // compute matrices
  MatrixNd workM, RiMRT, RiMDT, RiMNT, DiMRT, DiMDT, DiMNT, NiMRT, NiMDT, NiMNT;

  // first compute inv(M)*R'
  MatrixNd::transpose(R, workM);
  LinAlgd::solve_chol_fast(cholM, workM);

  // now compute R*inv(M)*R', D*inv(M)*R' (this is identical to R*inv(M)*D'),
  // and N*inv(M)*R' (this is identical to R*inv(M)*N')
  R.mult(workM, RiMRT);
  D.mult(workM, DiMRT);
  N.mult(workM, NiMRT);
  MatrixNd::transpose(DiMRT, RiMDT);
  MatrixNd::transpose(NiMRT, RiMNT);

  // now compute inv(M)*D'
  MatrixNd::transpose(D, workM);
  LinAlgd::solve_chol_fast(cholM, workM);

  // now compute D*inv(M)*D' and N*inv(M)*D' (this is identical to D*inv(M)*N')
  D.mult(workM, DiMDT);
  N.mult(workM, NiMDT);
  MatrixNd::transpose(NiMDT, DiMNT);

  // finally, compute N*inv(M)*N'
  MatrixNd::transpose(N, workM);
  LinAlgd::solve_chol_fast(cholM, workM);
  N.mult(workM, NiMNT);

  // setup the G matrix
  MatrixNd G(NVARS, NVARS);
  G.block(R_INDEX, R_INDEX+R_SIZE, R_INDEX, R_INDEX+R_SIZE) = RiMRT;
  G.block(R_INDEX, R_INDEX+R_SIZE, D_INDEX, D_INDEX+D_SIZE) = RiMDT;
  G.block(R_INDEX, R_INDEX+R_SIZE, N_INDEX, N_INDEX+N_SIZE) = RiMNT;
  G.block(D_INDEX, D_INDEX+D_SIZE, R_INDEX, R_INDEX+R_SIZE) = DiMRT; 
  G.block(D_INDEX, D_INDEX+D_SIZE, D_INDEX, D_INDEX+D_SIZE) = DiMDT;
  G.block(D_INDEX, D_INDEX+D_SIZE, N_INDEX, N_INDEX+N_SIZE) = DiMNT;
  G.block(N_INDEX, N_INDEX+N_SIZE, R_INDEX, R_INDEX+R_SIZE) = NiMRT; 
  G.block(N_INDEX, N_INDEX+N_SIZE, D_INDEX, D_INDEX+D_SIZE) = NiMDT; 
  G.block(N_INDEX, N_INDEX+N_SIZE, N_INDEX, N_INDEX+N_SIZE) = NiMNT;

  // setup the c vector
  VectorNd workv;
  VectorNd c(NVARS);
  c.segment(R_INDEX, R_INDEX+R_SIZE) = R.mult(k, workv);
  c.segment(D_INDEX, D_INDEX+D_SIZE) = R.mult(k, workv);
  c.segment(N_INDEX, N_INDEX+N_SIZE) = R.mult(k, workv);

  // setup the inequality constraint matrix (P)
  MatrixNd JT;
  MatrixNd::transpose(J, JT);
  MatrixNd P(N_CONTACTS+N_ACT*2,NVARS);
  P.block(0, N_CONTACTS, R_INDEX, R_INDEX+R_SIZE) = RiMRT;
  P.block(0, N_CONTACTS, R_INDEX, D_INDEX+D_SIZE) = RiMDT;
  P.block(0, N_CONTACTS, R_INDEX, N_INDEX+N_SIZE) = RiMNT;
  P.block(N_CONTACTS, P.rows(), 0, NVARS).set_zero();
  P.block(N_CONTACTS, N_CONTACTS+J.columns(), R_INDEX, R_INDEX+R_SIZE) = JT;
  P.block(N_CONTACTS+J.columns(), P.rows(), R_INDEX, R_INDEX+R_SIZE) = JT;
  P.block(N_CONTACTS+J.columns(), P.rows(), R_INDEX, R_INDEX+R_SIZE).negate();

  // setup the inequality constraint vector (p)
  VectorNd p(N_CONTACTS+N_ACT*2);
  p.segment(0, N_CONTACTS) = c.segment(N_INDEX, N_INDEX+N_SIZE);
  p.segment(N_CONTACTS, J.columns()) = lolimit;
  p.segment(N_CONTACTS+J.columns(), N_CONTACTS+J.columns()*2) = hilimit;
  p.segment(N_CONTACTS+J.columns(), N_CONTACTS+J.columns()*2).negate();

  // setup the equality constraint matrix (Q)
  const MatrixNd& Q = D;

  // setup the equality constraint vector (q)
  VectorNd q(D.rows());
  q.set_zero();

  // setup bounds on variables -- only N variables have lower bounds
  VectorNd lb(NVARS), ub(NVARS);
  lb.set_one() *= -INF;
  ub.set_one() *= INF;
  lb.segment(N_INDEX, N_INDEX+N_SIZE).set_zero();

  // solve the QP using QLCPD
  Opt::QPActiveSet qp;
  VectorNd z;
  bool success = qp.qp_activeset(G, c, lb, ub, P, p, Q, q, z);

  // verify that we have success
  if (!success)
    throw std::runtime_error("Unexpected QP solver failure!");
  else
    fr = z.segment(R_INDEX, R_INDEX+R_SIZE); 
}

