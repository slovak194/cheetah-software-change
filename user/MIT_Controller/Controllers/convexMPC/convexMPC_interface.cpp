#include "convexMPC_interface.h"
#include "common_types.h"
#include "SolverMPC.h"
#include <eigen3/Eigen/Dense>
#include <pthread.h>
#include <stdio.h>
#include <string.h>

#define K_NUM_LEGS 4

#define K_NUM_LEGS 4

problem_setup problem_configuration;
u8 gait_data[K_MAX_GAIT_SEGMENTS];
pthread_mutex_t problem_cfg_mt;
pthread_mutex_t update_mt;
update_data_t update;
pthread_t solve_thread;

u8 first_run = 1;  //初次运行标志,防止互斥锁重复初始化

//初始化两个互斥锁,有什么用?
void initialize_mpc()
{
  //printf("Initializing MPC!\n");
  if(pthread_mutex_init(&problem_cfg_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize problem configuration mutex.\n");

  if(pthread_mutex_init(&update_mt,NULL)!=0)
    printf("[MPC ERROR] Failed to initialize update data mutex.\n");

#ifdef K_DEBUG
  printf("[MPC] Debugging enabled.\n");
    printf("[MPC] Size of problem setup struct: %ld bytes.\n", sizeof(problem_setup));
    printf("      Size of problem update struct: %ld bytes.\n",sizeof(update_data_t));
    printf("      Size of MATLAB floating point type: %ld bytes.\n",sizeof(mfp));
    printf("      Size of flt: %ld bytes.\n",sizeof(flt));
#else
  //printf("[MPC] Debugging disabled.\n");
#endif
}

void setup_problem(double dt, int horizon, double mu, double f_max)
{
  if(first_run)
  {
    first_run = false;
    initialize_mpc();
  }

#ifdef K_DEBUG
  printf("[MPC] Got new problem configuration!\n");
    printf("[MPC] Prediction horizon length: %d\n      Force limit: %.3f, friction %.3f\n      dt: %.3f\n",
            horizon,f_max,mu,dt);
#endif

  //pthread_mutex_lock(&problem_cfg_mt);

  problem_configuration.horizon = horizon;
  problem_configuration.f_max = f_max;
  problem_configuration.mu = mu;
  problem_configuration.dt = dt;

  //pthread_mutex_unlock(&problem_cfg_mt);
  resize_qp_mats(horizon);
}

//inline to motivate gcc to unroll the loop in here.
//将matlab的float point类型转换为C++ float point类型
inline void mfp_to_flt(flt* dst, mfp* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}
//将matlab的int类型转换为C++ u8类型
inline void mint_to_u8(u8* dst, mint* src, s32 n_items)
{
  for(s32 i = 0; i < n_items; i++)
    *dst++ = *src++;
}

int first_solved = 0;  //第一次求解完成
void update_problem_data_floats(float* p, float* v, float* q, float* w,
                                float* r, float yaw, float* weights,
                                float* state_trajectory, float alpha, int* gait)
{
  update.alpha = alpha;
  update.yaw = yaw;
  mint_to_u8(update.gait,gait,4*problem_configuration.horizon);
  memcpy((void*)update.p,(void*)p,sizeof(float)*3);
  memcpy((void*)update.v,(void*)v,sizeof(float)*3);
  memcpy((void*)update.q,(void*)q,sizeof(float)*4);
  memcpy((void*)update.w,(void*)w,sizeof(float)*3);
  memcpy((void*)update.r,(void*)r,sizeof(float)*12);
  memcpy((void*)update.weights,(void*)weights,sizeof(float)*12);
  memcpy((void*)update.traj,(void*)state_trajectory, sizeof(float) * 12 * problem_configuration.horizon);

  solve_mpc(&update, &problem_configuration);
  first_solved = 1;
}

double get_solution(int index)
{
  if(!first_solved) return 0.f;
  mfp* qs = get_q_soln();
  return qs[index];
}
