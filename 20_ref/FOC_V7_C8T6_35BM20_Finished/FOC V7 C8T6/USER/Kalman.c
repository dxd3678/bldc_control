#include "Kalman.h"
Slid_t Current_a,Current_b,Filter_shaft_velocity,Filter_ret_q;
Slid_t Slid_velocity;
KalMan_t KalMan_current_Iq;	

float Kalman_Filter(Kalman* p,float dat)
{
	if(!p) return 0;
	p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
	p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
	p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
	p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
	p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
	p->P_last = p->P_now;
	p->X_last = p->X_now;
	return p->X_now;
}

void Kalman_Init(Kalman* p,float T_Q,float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
	p->A = 1;
	p->H = 1;
	p->X_mid = p->X_last;
}


float Slid_Filter(Slid_t* p,float dat,int Filter_Num)
{

	int i;
	p->buf_[p->num]=dat;
	if(p->num<Filter_Num-1)
	{
		p->num++;
	}
	else 
	{
		float _buf_add=0;
		for(i=0;i<Filter_Num;i++)
		{
			_buf_add+=p->buf_[i];
		}
		
		
		for(i=0;i<Filter_Num-1;i++)
		{
			p->buf_[i]=p->buf_[i+1];
		}
		return _buf_add/Filter_Num;
	}	
	return dat;
}


/*
For Example

//	KalMan_t KalMan_Angle_L,KalMan_Angle_R;	
//	KalMan_PramInit(&KalMan_Angle_L);	//把卡尔曼的系数全部跟新一遍，下面调用滤波会用到这些参数
//	MPU_R.angle[1]=KalMan_Update(&MPU_R.angle[1],&KalMan_Angle_R);

*/
/*==============================================================================
1.预估计
   X(k|k-1) = F(k,k-1)*X(k-1|k-1)        //控制量为0


2.计算预估计协方差矩阵
   P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q(k)
   Q(k) = U(k)×U(k)' 


3.计算卡尔曼增益矩阵
   Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R(k))
   R(k) = N(k)×N(k)' 


4.更新估计
   X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))


5.计算更新后估计协防差矩阵
   P(k|k) =（I-Kg(k)*H）*P(k|k-1)


6. 更新最优值


F(k,k-1):     状态转移矩阵
X(k|k-1):     根据k-1时刻的最优值估计k时刻的值
X(k-1|k-1):   k-1时刻的最优值
P(k|k-1):     X(k|k-1)对应的covariance
P(k-1|k-1):   X(k-1|k-1)对应的covariance
Q(k):         系统过程的covariance
R(k):         测量过程的协方差
H(k):          转移矩阵
Z(k):         k时刻的测量值


基本思路: 首先根据上一次(如果是第一次则根据预赋值计算)的数据计算出本次的估计值,
          同理,根据上一次的数据计算出本次估计值的协方差;  接着,由本次估计值的协
          方差计算出卡尔曼增益;  最后,根据估测值和测量值计算当前最优值及其协方差
		  
基本思路:  首先根据上一次(如果是第一次则根据预赋值计算)的数据计算出本次的估计值,
          根据上一次的数据计算出本次估计值的协方差
		  本次估计值的协方差计算出卡尔曼增益; 
		  根据估测值和测量值计算当前最优值
		  更新协方差

==============================================================================*/



//================================================//
//==             最优值方差结构体               ==//
//================================================//



void KalMan_PramInit(KalMan_t *Ka)
{
	unsigned char   i;
	Ka->I[LENGTH-1]  =  1 ;     //  单位矩阵
	Ka->X[LENGTH-1]  =  9.8 ;    //  当前状态的预测值
	Ka-> P[LENGTH-1]  =  0 ;      //  当前状态的预测值的协方差
	Ka-> K[LENGTH-1]  =  0 ;      //  卡尔曼增益
	Ka-> Temp3[LENGTH-1] =  0 ;   //  辅助变量
	Ka-> F[LENGTH-1]  =  1 ;      		//  状态转移矩阵   即：坚信当前的状态与上一次状态的关系，如果坚信是一样的，则状态转移矩阵就为单位矩阵
	
	Ka-> Q[LENGTH-1]  =  0.01 ;//0->0001f       //  系统过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值	
	Ka-> R[LENGTH-1]  =  10 ;      		//  测量过程的协方差	协方差的定义：真实值与期望值之差的平方的期望值   
	
	Ka-> H[LENGTH-1]  =  1 ;      //  观测矩阵转移矩阵	测量值与状态预测值之间的单位换算关系，即把预测值单位换算成测量值单位
	Ka-> Temp1[LENGTH-1] =  1 ;   //  辅助变量, 同时保存tOpt->XPreOpt[]的初始化值
	Ka-> Temp2[LENGTH-1] =  10000 ;       //  辅助变量, 同时保存tCov->PPreOpt[]的初始化值
	
	for (i=0; i<LENGTH; i++)
	{
		Ka->tOpt.XPreOpt[i] = Ka->Temp1[i];           //零值初始化
	}
	for (i=0; i<LENGTH; i++)
	{
		Ka->tCov.PPreOpt[i] =Ka-> Temp2[i];           //零值初始化
	}
  
}







//============================================================================//
//==                          实矩阵相加                                    ==//
//============================================================================//
//==函数说明: m*n阶矩阵和m*n阶矩阵相加, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixAdd(float *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;
  
  for (i=0; i<m*n; i++)
  {
    c[i] = a[i] + b[i];
  }
}





//============================================================================//
//==                          实矩阵相减                                    ==//
//============================================================================//
//==函数说明: m*n阶矩阵和m*n阶矩阵相减, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixMinus(float *a, float *b, float *c, unsigned char m, unsigned char n)
{
  unsigned char i;
  
  for (i=0; i<m*n; i++)
  {
    c[i] = a[i] - b[i];
  }
}





//============================================================================//
//==                          实矩阵相乘                                    ==//
//============================================================================//
//==函数说明: m*p阶矩阵和p*n阶矩阵相乘, 矩阵均以行优先存储                  ==//
//==入口参数: *a                指向左矩阵的指针                            ==//
//==          *b                指向右矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 左矩阵行数                                  ==//
//==          p                 左矩阵列数(右矩阵行数)                      ==//
//==          n                 右矩阵列数                                  ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixMul(float *a, float *b, float *c, unsigned char m, unsigned char p, unsigned char n)
{
  unsigned char i,j,k;                      //循环变量
  
  for (i=0; i<m; i++)                       //扫描矩阵的行
  {
    for (j=0; j<n; j++)                     //扫描矩阵的列
    {
      c[i*n+j] = 0.0;
      for (k=0; k<p; k++)
      {
                                            //按照矩阵乘法的概念计算
        c[i*n+j] += a[i*p+k] * b[k*n+j];    //a[i*p+k]: 由于k的循环,扫描了a[]数组的第i行
                                            //b[k*n+j]: 由于k的循环,扫描了a[]数组的第j行
      }
    }
  }
}





//============================================================================//
//==                            实矩阵求转置                                ==//
//============================================================================//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixTrans(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j;
  
  for (i=0; i<n; i++)
  {
    for (j=0; j<m; j++)
    {
      c[i*m+j] = a[j*n+i];      //对于m*n的矩阵,i*m+j表示i行j列的元素; 对于n*m的矩阵,i*n+j表示i行j列的元素
    }
  }
}





//============================================================================//
//==                          实矩阵求行列式1                               ==//
//============================================================================//
//==函数说明: 根据行列式展开法求得, 对高阶行列式有问题                      ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: 无                                                            ==//
//==返回值:   X                 行列式的值                                  ==//
//============================================================================//
float MatrixDet1(float *a, unsigned char m, unsigned char n)
{
  signed char i, j, k, p, r;                  //循环变量
  float Temp=1, Temp1=1, S=0, S1=0;           //辅助变量
  float X;                                    //行列式的值
  
  if (n==2)
  {
    for(i=0; i<m; i++)                        //扫描矩阵的行
    {
      for(j=0 ;j<n; j++)                      //扫描矩阵的列
      {
        if((i+j)%2)                           //根据行优先的法则,对角线元素下标之和为偶数
        {
          Temp1 *= a[i*n+j];
        }
        else
        {
          Temp  *= a[i*n+j];
        }
      }
    }
    X=Temp-Temp1;
  }
  else
  {
    for (k=0; k<n; k++)                       //计算行列式展开式中的被减数
    { 
      for (i=0,j=k; i<m&&j<n; i++,j++)        //斜线方向的数据相乘
      {
        Temp *= a[i*n+j];
      }
      if (m-i)                                //每次相乘都是m个元素,若不足m个元素,则需要继续处理
      { 
        for (p=m-i,r=m-1; p>0; p--,r--)
        {
          Temp  *= a[r*n+p-1];
        }
      }
      S += Temp;
      Temp = 1;
    }
      
      
    for (k=n-1; k>=0; k--)                    //计算行列式展开式中的减数
    { 
      for(i=0,j=k; i<m&&j>=0; i++,j--)
      {
        Temp1 *= a[i*n+j];
      }
      if (m-i) 
      {
        for(p=m-1,r=i; r<m; p--,r++)
        {
          Temp1 *= a[r*n+p];
        }
      } 
      S1 += Temp1;
      Temp1 = 1;
    }
    X=S-S1;
  }
  
  return   X;
}





//============================================================================//
//==                           实矩阵求逆1                                  ==//
//============================================================================//
//==函数说明: 根据矩阵行列式等求实矩阵的逆矩阵,对高阶行列式有问题           ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *c                指向逆矩阵的指针                            ==//
//==          m                 矩阵行数                                    ==//
//==          n                 矩阵列数                                    ==//
//==出口参数: *c                指向逆矩阵的指针                            ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixInv1(float *a, float *c, unsigned char m, unsigned char n)
{
  unsigned char i,j,k,x,y;
  float AB[LENGTH], SP[LENGTH], B[LENGTH];
  float X;                                    //行列式的值
  
  X = MatrixDet1(a, m, n);
  X = 1/X;                                    //行列式的倒数
  
  for (i=0; i<m; i++)
  {
    for (j=0; j<n; j++)
    {
      for (k=0; k<m*n; k++)
      {
        B[k] = a[k];                          //初始化辅助矩阵B=a
      }
      for(x=0; x<n; x++)
      {
        B[i*n+x] = 0;                         //i行所有元素清零
      }
      for(y=0; y<m; y++)
      {
        B[m*y+j] = 0;                         //j列所有元素清零
      }
      B[i*n+j] = 1;                           //当前的主元置1
      SP[i*n+j] = MatrixDet1(B,m,n);
      AB[i*n+j] = X*SP[i*n+j];
    }
  }
  MatrixTrans(AB, c, m, n);
}





//============================================================================//
//==                     Gauss-Jordan法实矩阵求逆                           ==//
//============================================================================//
//==函数说明: 求n阶矩阵的逆矩阵,矩阵以行优先存储; 结果保存在源数组里,因此需 ==//
//==          注意会破坏源数组的数据;                                       ==//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          n                 矩阵阶数                                    ==//
//==出口参数: *a                指向逆矩阵的指针                            ==//
//==返回值:   BOOL              操作成功、失败                              ==//
//============================================================================//
unsigned char Gauss_Jordan(float *a, unsigned char n)
{
    signed char i,j,k,l,u,v;
    signed char is[ORDER];
    signed char js[ORDER];
    float d,p;
    
    for (k=0; k<=n-1; k++)
      { d=0.0;
        for (i=k; i<=n-1; i++)
        for (j=k; j<=n-1; j++)
          { l=i*n+j; p=fabs(a[l]);
            if (p>d) { d=p; is[k]=i; js[k]=j;}
          }
        if (d+1.0==1.0)
          {
            return(0);
          }
        if (is[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=is[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (js[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+js[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        l=k*n+k;
        a[l]=1.0/a[l];
        for (j=0; j<=n-1; j++)
          if (j!=k)
            { u=k*n+j; a[u]=a[u]*a[l];}
        for (i=0; i<=n-1; i++)
          if (i!=k)
            for (j=0; j<=n-1; j++)
              if (j!=k)
                { u=i*n+j;
                  a[u]=a[u]-a[i*n+k]*a[k*n+j];
                }
        for (i=0; i<=n-1; i++)
          if (i!=k)
            { u=i*n+k; a[u]=-a[u]*a[l];}
      }
    for (k=n-1; k>=0; k--)
      { if (js[k]!=k)
          for (j=0; j<=n-1; j++)
            { u=k*n+j; v=js[k]*n+j;
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
        if (is[k]!=k)
          for (i=0; i<=n-1; i++)
            { u=i*n+k; v=i*n+is[k];
              p=a[u]; a[u]=a[v]; a[v]=p;
            }
      }
    
    return(1);
}





//============================================================================//
//==                            计算A*B*A'                                  ==//
//============================================================================//
//==入口参数: *a                指向实矩阵的指针                            ==//
//==          *b                指向实矩阵的指针                            ==//
//==          *c                指向结果矩阵的指针                          ==//
//==          n                 矩阵的阶数                                  ==//
//==出口参数: *c                指向结果矩阵的指针                          ==//
//==返回值:   无                                                            ==//
//============================================================================//
void MatrixCal(float *a, float *b, float *c, unsigned char n)
{
  float Temp1[LENGTH] = {0};
  float Temp2[LENGTH] = {0};
  
  MatrixMul(a, b, Temp1, n, n, n);
  MatrixTrans(a, Temp2, n, n);
  MatrixMul(Temp1, Temp2, c, n, n, n);
}
//============================================================================//
//==                          卡尔曼滤波                                    ==//
//============================================================================//
//==入口参数: 当前时刻的测量值                                                            ==//
//==出口参数: 当前时刻的最优值                                                            ==//
//==返回值:   当前时刻的最优值                                                            ==//
//============================================================================//

float KalMan_Update(float *Z,KalMan_t *Ka)
{
	MatrixMul(Ka->F, Ka->tOpt.XPreOpt, Ka->X, ORDER, ORDER, ORDER);       //  基于系统的上一状态而预测现在状态; X(k|k-1) = F(k,k-1)*X(k-1|k-1)

	MatrixCal(Ka->F, Ka->tCov.PPreOpt, Ka->Temp1, ORDER);
	MatrixAdd(Ka->Temp1, Ka->Q, Ka->P, ORDER, ORDER);                     //  预测数据的协方差矩阵; P(k|k-1) = F(k,k-1)*P(k-1|k-1)*F(k,k-1)'+Q

	MatrixCal(Ka->H, Ka->P, Ka->Temp1, ORDER);
	MatrixAdd(Ka->Temp1, Ka->R, Ka->Temp1, ORDER, ORDER);				  //计算卡尔马增益的分母
	Gauss_Jordan(Ka->Temp1, ORDER);
	MatrixTrans(Ka->H, Ka->Temp2, ORDER, ORDER);
	MatrixMul(Ka->P, Ka->Temp2, Ka->Temp3, ORDER, ORDER, ORDER);
	MatrixMul(Ka->Temp1, Ka->Temp3, Ka->K, ORDER, ORDER, ORDER);          //  计算卡尔曼增益; Kg(k) = P(k|k-1)*H' / (H*P(k|k-1)*H' + R)

	MatrixMul(Ka->H, Ka->X, Ka->Temp1, ORDER, ORDER, ORDER);
	MatrixMinus(Z ,Ka-> Temp1,Ka-> Temp1, ORDER, ORDER);
	MatrixMul(Ka->K,Ka-> Temp1,Ka-> Temp2, ORDER, ORDER, ORDER);
	MatrixAdd(Ka->X,Ka-> Temp2, Ka->tOpt.XNowOpt, ORDER, ORDER);          //  根据估测值和测量值计算当前最优值; X(k|k) = X(k|k-1)+Kg(k)*(Z(k)-H*X(k|k-1))

	MatrixMul(Ka->K, Ka->H,Ka-> Temp1, ORDER, ORDER, ORDER);
	MatrixMinus(Ka->I,Ka-> Temp1,Ka-> Temp1, ORDER, ORDER);
	MatrixMul(Ka->Temp1, Ka->P, Ka->tCov.PNowOpt, ORDER, ORDER, ORDER);   //  计算更新后估计协防差矩阵; P(k|k) =（I-Kg(k)*H）*P(k|k-1)

	for (char i=0; i<LENGTH; i++)
	{
	 Ka-> tOpt.XPreOpt[i] =Ka-> tOpt.XNowOpt[i];
	 Ka-> tCov.PPreOpt[i] =Ka-> tCov.PNowOpt[i];
	}
	
	*Z=Ka->tOpt.XNowOpt[0];
	return Ka->tOpt.XNowOpt[0];
}
