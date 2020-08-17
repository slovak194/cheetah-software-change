#include "SolverMPC.h"
#include "common_types.h"
#include "convexMPC_interface.h"
#include "RobotState.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <cmath>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <qpOASES.hpp>
#include <stdio.h>
#include <sys/time.h>
#include <Utilities/Timer.h>

#define BIG_NUMBER 5e10

RobotState rs;
using Eigen::StorageOptions::RowMajor;

Matrix<fpt,13*HORIZON_LENGTH,13> A_qp;  
Matrix<fpt,13*HORIZON_LENGTH,12*HORIZON_LENGTH> B_qp;
Matrix<fpt,13,12> Bdt;
Matrix<fpt,13,13> Adt;
Matrix<fpt,25,25> ABc,expmm;
Eigen::DiagonalMatrix<fpt,13*HORIZON_LENGTH> S;
Matrix<fpt,13*HORIZON_LENGTH,1> X_d;
Matrix<fpt,20*HORIZON_LENGTH,1> U_b;
Matrix<fpt,20*HORIZON_LENGTH,12*HORIZON_LENGTH,RowMajor> fmat;
Matrix<fpt,12*HORIZON_LENGTH,12*HORIZON_LENGTH,RowMajor> qH;
Matrix<fpt,12*HORIZON_LENGTH,1> qg;

Matrix<fpt,12*HORIZON_LENGTH,12*HORIZON_LENGTH> alpha12;

qpOASES::real_t* H_qpoases;
qpOASES::real_t* g_qpoases;
qpOASES::real_t* A_qpoases;
qpOASES::real_t* lb_qpoases;
qpOASES::real_t* ub_qpoases;
qpOASES::real_t* q_soln;     //最终结果存放在这儿

qpOASES::real_t* H_red;
qpOASES::real_t* g_red;
qpOASES::real_t* A_red;
qpOASES::real_t* lb_red;
qpOASES::real_t* ub_red;
qpOASES::real_t* q_red;

char var_elim[2000];
char con_elim[2000];

mfp* get_q_soln()
{
  return q_soln;
}

s8 near_zero(fpt a)
{
  return (a < 0.01 && a > -.01) ;
}

s8 near_one(fpt a)
{
  return near_zero(a-1);
}
/*
void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,12) = Bc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);

  Eigen::SparseMatrix<fpt> powerMats[HORIZON_LENGTH+1];

  for(int i=0;i<HORIZON_LENGTH+1;i++)
  {
    powerMats[i].resize(13,13);
  }
  for(int i=0;i<13;i++)
  {
    powerMats[0].insert(i,i) = 1;
  }

  for(int i = 1; i < horizon+1; i++) {
    powerMats[i] = Adt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) =   powerMats[r+1];
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12) = powerMats[a_num]* Bdt;
      }
    }
  }
}
*/
Matrix<fpt,13,13> powerMats[HORIZON_LENGTH+1];
void c2qp(Matrix<fpt,13,13> Ac, Matrix<fpt,13,12> Bc,fpt dt,s16 horizon)
{
  ABc.setZero();
  ABc.block(0,0,13,13) = Ac;
  ABc.block(0,13,13,12) = Bc;
  ABc = dt*ABc;
  expmm = ABc.exp();
  Adt = expmm.block(0,0,13,13);
  Bdt = expmm.block(0,13,13,12);

  powerMats[0].setIdentity();

  //std::cout<<"Adt"<<std::endl<<Adt<<std::endl;
  //std::cout<<"Bdt"<<std::endl<<Bdt<<std::endl;

  for(int i = 1; i < horizon+1; i++) {
    powerMats[i].noalias() = Adt * powerMats[i-1];
  }

  for(s16 r = 0; r < horizon; r++)
  {
    A_qp.block(13*r,0,13,13) = powerMats[r+1];//Adt.pow(r+1);
    for(s16 c = 0; c < horizon; c++)
    {
      if(r >= c)
      {
        s16 a_num = r-c;
        B_qp.block(13*r,12*c,13,12).noalias() = powerMats[a_num] * Bdt;
      }
    }
  }
}

//矩阵初始化
void resize_qp_mats()
{
  A_qp.setZero();
  B_qp.setZero();
  S.setZero();
  X_d.setZero();
  U_b.setZero();
  fmat.setZero();
  qH.setZero();
  qg.setZero();
  alpha12.setIdentity();

  H_qpoases = (qpOASES::real_t*)malloc(12*12*HORIZON_LENGTH*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  g_qpoases = (qpOASES::real_t*)malloc(12*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  A_qpoases = (qpOASES::real_t*)malloc(12*20*HORIZON_LENGTH*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  lb_qpoases = (qpOASES::real_t*)malloc(20*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  ub_qpoases = (qpOASES::real_t*)malloc(20*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  q_soln = (qpOASES::real_t*)malloc(12*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  H_red = (qpOASES::real_t*)malloc(12*12*HORIZON_LENGTH*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  g_red = (qpOASES::real_t*)malloc(12*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  A_red = (qpOASES::real_t*)malloc(12*20*HORIZON_LENGTH*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  lb_red = (qpOASES::real_t*)malloc(20*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  ub_red = (qpOASES::real_t*)malloc(20*1*HORIZON_LENGTH*sizeof(qpOASES::real_t));
  q_red = (qpOASES::real_t*)malloc(12*HORIZON_LENGTH*sizeof(qpOASES::real_t));
}

inline Matrix<fpt,3,3> cross_mat(Matrix<fpt,3,3> I_inv, Matrix<fpt,3,1> r)
{
  Matrix<fpt,3,3> cm;
  cm << 0.f, -r(2), r(1),
    r(2), 0.f, -r(0),
    -r(1), r(0), 0.f;
  return I_inv * cm;
}
//continuous time state space matrices.
void ct_ss_mats(Matrix<fpt,3,3> I_world, fpt m, Matrix<fpt,3,4> r_feet, Matrix<fpt,3,3> R_yaw, Matrix<fpt,13,13>& A, Matrix<fpt,13,12>& B)
{
  A.setZero();
  A(3,9) = 1.f;
  A(4,10) = 1.f;
  A(5,11) = 1.f;

  A(11,12) = 1.f;
  A.block(0,6,3,3) = R_yaw.transpose();

  B.setZero();
  Matrix<fpt,3,3> I_inv = I_world.inverse();

  for(s16 b = 0; b < 4; b++)
  {
    B.block(6,b*3,3,3) = cross_mat(I_inv,r_feet.col(b));
    B.block(9,b*3,3,3) = Matrix<fpt,3,3>::Identity() / m;
  }
}
//四元数转换成rpy角度
void quat_to_rpy(Quaternionf q, Matrix<fpt,3,1>& rpy)
{
  fpt as = t_min(-2.*(q.x()*q.z()-q.w()*q.y()),.99999);
  rpy(0) = atan2(2.f*(q.x()*q.y()+q.w()*q.z()),sq(q.w()) + sq(q.x()) - sq(q.y()) - sq(q.z()));
  rpy(1) = asin(as);
  rpy(2) = atan2(2.f*(q.y()*q.z()+q.w()*q.x()),sq(q.w()) - sq(q.x()) - sq(q.y()) + sq(q.z()));
}

Matrix<fpt,13,1> x_0;
Matrix<fpt,3,3> I_world;
Matrix<fpt,13,13> A_ct;
Matrix<fpt,13,12> B_ct_r;
Matrix<fpt,3,1> rpy;

bool firstRun = true;
void solve_mpc(update_data_t* update, problem_setup* setup)
{
  if(firstRun) //这里是仅存的两组魔法数字
  {
    //weights
    Matrix<fpt,13,1> full_weight;
    for(u8 i = 0; i < 12; i++)
      full_weight(i) = update->weights[i];
    full_weight(12) = 0.f;
    S.diagonal() = full_weight.replicate(setup->horizon,1);
    //alpha
    alpha12.setIdentity();
    alpha12 = (update->alpha)*alpha12;
    firstRun = false;
  }
  //robot state
  rs.set(update->p, update->v, update->q, update->w, update->r, update->yaw);
  //rpy
  quat_to_rpy(rs.q,rpy);
  //x0
  x_0 << rpy(2), rpy(1), rpy(0), rs.p , rs.w, rs.v, -9.8f;
  // I
  I_world = rs.R_yaw * rs.I_body * rs.R_yaw.transpose();
  //连续时间状态矩阵
  ct_ss_mats(I_world,rs.m,rs.r_feet,rs.R_yaw,A_ct,B_ct_r);
  //QP matrices
  c2qp(A_ct,B_ct_r,setup->dt,setup->horizon);  //0.13ms
  //x_d
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 12; j++)
      X_d(13*i+j,0) = update->traj[12*i+j];
  }
  //U_b
  s16 k = 0;
  for(s16 i = 0; i < setup->horizon; i++)
  {
    for(s16 j = 0; j < 4; j++)
    {
      U_b(5*k + 0) = BIG_NUMBER;
      U_b(5*k + 1) = BIG_NUMBER;
      U_b(5*k + 2) = BIG_NUMBER;
      U_b(5*k + 3) = BIG_NUMBER;
      U_b(5*k + 4) = update->gait[i*4 + j] * setup->f_max;
      k++;
    }
  }
  fpt mu = 1.f/setup->mu;
  Matrix<fpt,5,3> f_block;
  f_block <<  mu, 0,  1.f,
    -mu, 0,  1.f,
    0,  mu, 1.f,
    0, -mu, 1.f,
    0,   0, 1.f;

  for(s16 i = 0; i < setup->horizon*4; i++)
  {
    fmat.block(i*5,i*3,5,3) = f_block;
  }

  qH.noalias()  = 2*(B_qp.transpose()*S*B_qp + alpha12);
  qg.noalias()  = 2*B_qp.transpose()*S*(A_qp*x_0 - X_d);

  
  H_qpoases = qH.data();
  g_qpoases = qg.data();
  A_qpoases = fmat.data();
  ub_qpoases = U_b.data();

  for(s16 i = 0; i < 20*setup->horizon; i++)
    lb_qpoases[i] = 0.0f;

  s16 num_constraints = 20*setup->horizon;
  s16 num_variables = 12*setup->horizon;
  qpOASES::int_t nWSR = 100;
  int new_vars = num_variables;
  int new_cons = num_constraints;

  for(int i = 0; i < num_constraints; i++)
    con_elim[i] = 0;

  for(int i = 0; i < num_variables; i++)
    var_elim[i] = 0;
  for(int i = 0; i < num_constraints; i++)
  {
    if(! (near_zero(lb_qpoases[i]) && near_zero(ub_qpoases[i]))) continue;
    float* c_row = &A_qpoases[i*num_variables];
    for(int j = 0; j < num_variables; j++)
    {
      if(near_one(c_row[j]))
      {
        new_vars -= 3;
        new_cons -= 5;
        int cs = (j*5)/3 -3;
        var_elim[j-2] = 1;
        var_elim[j-1] = 1;
        var_elim[j  ] = 1;
        con_elim[cs] = 1;
        con_elim[cs+1] = 1;
        con_elim[cs+2] = 1;
        con_elim[cs+3] = 1;
        con_elim[cs+4] = 1;
      }
    }
  }
  int var_ind[new_vars];
  int con_ind[new_cons];
  int vc = 0;
  for(int i = 0; i < num_variables; i++)
  {
    if(!var_elim[i])
    {
      if(!(vc<new_vars))
      {
        printf("BAD ERROR 1\n");
      }
      var_ind[vc] = i;
      vc++;
    }
  }
  vc = 0;
  for(int i = 0; i < num_constraints; i++)
  {
    if(!con_elim[i])
    {
      if(!(vc<new_cons))
      {
        printf("BAD ERROR 1\n");
      }
      con_ind[vc] = i;
      vc++;
    }
  }
  for(int i = 0; i < new_vars; i++)
  {
    int olda = var_ind[i];
    g_red[i] = g_qpoases[olda];
    for(int j = 0; j < new_vars; j++)
    {
      int oldb = var_ind[j];
      H_red[i*new_vars + j] = H_qpoases[olda*num_variables + oldb];
    }
  }
  for (int con = 0; con < new_cons; con++)
  {
    for(int st = 0; st < new_vars; st++)
    {
      float cval = A_qpoases[(num_variables*con_ind[con]) + var_ind[st] ];
      A_red[con*new_vars + st] = cval;
    }
  }
  for(int i = 0; i < new_cons; i++)
  {
    int old = con_ind[i];
    ub_red[i] = ub_qpoases[old];
    lb_red[i] = lb_qpoases[old];
  }
  Timer t;
  /*
  qpOASES::QProblem problem_red (new_vars, new_cons);
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  problem_red.setOptions(op);

  int rval = problem_red.init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
  (void)rval;
  int rval2 = problem_red.getPrimalSolution(q_red);
  if(rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");

  */
  static qpOASES::SQProblem* problem_red;
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;

  static int pre_new_vars = -1;
  static int pre_new_cons = -1;
  static bool first_run = true;
  if((firstRun)||(pre_new_vars!=new_vars)||(pre_new_cons!=new_cons))
  {
    if(!first_run) delete problem_red;
    problem_red = new qpOASES::SQProblem(new_vars, new_cons);
    problem_red->setOptions(op);
    problem_red->init(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);

    first_run = false;
    pre_new_vars = new_vars;
    pre_new_cons = new_cons;
  }
  else
  {
    problem_red->hotstart(H_red, g_red, A_red, NULL, NULL, lb_red, ub_red, nWSR);
  }
  int rval2 = problem_red->getPrimalSolution(q_red);
  if(rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");


  printf("SOLVE TIME: %.3f\n", t.getMs());  //查看计算时间 
  vc = 0;
  for(int i = 0; i < num_variables; i++)
  {
    if(var_elim[i])
    {
      q_soln[i] = 0.0f;
    }
    else
    {
      q_soln[i] = q_red[vc];
      vc++;
    }
  }
  
}
