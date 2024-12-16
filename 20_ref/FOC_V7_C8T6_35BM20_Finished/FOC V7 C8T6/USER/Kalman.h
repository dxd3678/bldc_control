#ifndef Kalman_H
#define Kalman_H


//卡尔曼滤波结构体
typedef struct{
	float X_last;
	float X_mid;
	float X_now;
	float P_mid;
	float P_now;
	float P_last;
	float kg;
	float A;
	float Q;
	float R;
	float H;
} Kalman;

typedef struct{
	int num;
	float buf_[30];
} Slid_t;

float Kalman_Filter(Kalman* p,float dat);
void Kalman_Init(Kalman* p,float T_Q,float T_R);
float Slid_Filter(Slid_t* p,float dat,int Filter_Num);

extern Kalman kalman_current_Iq_Filter;
extern Kalman kalman_shaft_velocity_Filter;
extern Kalman kalman_shaft_angle_Filter;
extern Slid_t Current_a,Current_b,Filter_shaft_velocity,Filter_ret_q;
extern Slid_t Slid_velocity;
#define   LENGTH      1*1
#define   ORDER       1
#define   N           100
#define   SEED        1567

typedef struct  _tCovariance
{
  float PNowOpt[LENGTH];
  float PPreOpt[LENGTH];
}tCovariance;

typedef struct  _tOptimal
{
  float XNowOpt[LENGTH];
  float XPreOpt[LENGTH];
}tOptimal;

typedef struct _KalMan
{
	tOptimal      tOpt;
	tCovariance   tCov;
	//float         Z[LENGTH]  = {4000};           //  测量值(每次测量的数据需要存入该数组)
	float         I[LENGTH];              //  单位矩阵
	float         X[LENGTH];            //  当前状态的预测值
	float         P[LENGTH];              //  当前状态的预测值的协方差
	float         K[LENGTH];              //  卡尔曼增益
	float         Temp3[LENGTH];           //  辅助变量
	//============================================================================//
	//==                    卡尔曼滤波需要配置的变量                            ==//
	//============================================================================//
	float         F[LENGTH];              		//  状态转移矩阵   即：坚信当前的状态与上一次状态的关系，如果坚信是一样的，则状态转移矩阵就为单位矩阵
	float         Q[LENGTH];//0.0001f       //  系统过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值
	float         R[LENGTH];              		//  测量过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值   
	//如果你需要滤波结果更依赖于观测量，那就调小R，增大Q；反之，调大R，调小Q，这样估计值就取决于系统。
	//如果R大Q小，就是说，状态估计值比测量值要可靠，这时，所得出的结果就是更接近估计值；
	//如果R小Q大，这时，计算出来的结果就会更接近测量值。
	float         H[LENGTH];              //  观测矩阵转移矩阵	测量值与状态预测值之间的单位换算关系，即把预测值单位换算成测量值单位
	float         Temp1[LENGTH];           //  辅助变量, 同时保存tOpt.XPreOpt[]的初始化值
	float         Temp2[LENGTH];       //  辅助变量, 同时保存tCov.PPreOpt[]的初始化值

}KalMan_t;


extern KalMan_t KalMan_current_Iq;	

void KalMan_PramInit(KalMan_t *Ka);
float KalMan_Update(float *Z,KalMan_t *Ka);
#endif
