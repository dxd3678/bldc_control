#ifndef Kalman_H
#define Kalman_H


//�������˲��ṹ��
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
	//float         Z[LENGTH]  = {4000};           //  ����ֵ(ÿ�β�����������Ҫ���������)
	float         I[LENGTH];              //  ��λ����
	float         X[LENGTH];            //  ��ǰ״̬��Ԥ��ֵ
	float         P[LENGTH];              //  ��ǰ״̬��Ԥ��ֵ��Э����
	float         K[LENGTH];              //  ����������
	float         Temp3[LENGTH];           //  ��������
	//============================================================================//
	//==                    �������˲���Ҫ���õı���                            ==//
	//============================================================================//
	float         F[LENGTH];              		//  ״̬ת�ƾ���   �������ŵ�ǰ��״̬����һ��״̬�Ĺ�ϵ�����������һ���ģ���״̬ת�ƾ����Ϊ��λ����
	float         Q[LENGTH];//0.0001f       //  ϵͳ���̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ
	float         R[LENGTH];              		//  �������̵�Э����	Э����Ķ��壺��ʵֵ������ֵ֮���ƽ��������ֵ   
	//�������Ҫ�˲�����������ڹ۲������Ǿ͵�СR������Q����֮������R����СQ����������ֵ��ȡ����ϵͳ��
	//���R��QС������˵��״̬����ֵ�Ȳ���ֵҪ�ɿ�����ʱ�����ó��Ľ�����Ǹ��ӽ�����ֵ��
	//���RСQ����ʱ����������Ľ���ͻ���ӽ�����ֵ��
	float         H[LENGTH];              //  �۲����ת�ƾ���	����ֵ��״̬Ԥ��ֵ֮��ĵ�λ�����ϵ������Ԥ��ֵ��λ����ɲ���ֵ��λ
	float         Temp1[LENGTH];           //  ��������, ͬʱ����tOpt.XPreOpt[]�ĳ�ʼ��ֵ
	float         Temp2[LENGTH];       //  ��������, ͬʱ����tCov.PPreOpt[]�ĳ�ʼ��ֵ

}KalMan_t;


extern KalMan_t KalMan_current_Iq;	

void KalMan_PramInit(KalMan_t *Ka);
float KalMan_Update(float *Z,KalMan_t *Ka);
#endif
