#include "MahonyAHRS.h"
#include "math.h"
//#include "IMU.H"


#define sampleFreq 190.48f
#define twoKpDef   1.0f
#define twoKiDef   0.05f

volatile float twoKp=twoKpDef;
volatile float twoKi=twoKiDef;
volatile float p0=1.0f,p1=0.0f,p2=0.0f,p3=0.0f;

volatile float integralFBx=0.0f,integralFBy=0.0f,integralFBz=0.0f;

double Anglex=0,Angley=0,Anglez=0;

extern float test_angle_error_x,test_angle_error_y,test_angle_error_z;

void MahonyAHRSupdateIMU(float gx,float gy,float gz,float ax,float ay,float az);
float invSqrt(float x);
void recode(void);
float Sign(float x);

void MahonyAHRSupdate(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz)
{
	float recipNorm;
	float p0p0,p0p1,p0p2,p0p3,p1p1,p1p2,p1p3,p2p2,p2p3,p3p3;
	float hx,hy,bx,bz;
	float halfvx,halfvy,halfvz,halfwx,halfwy,halfwz;
	float halfex,halfey,halfez;
	float qa,qb,qc,qd;
	
	if((mx==0.0f)&&(my==0.0f)&&(mz==0.0f)){
		MahonyAHRSupdateIMU(gx,gy,gz,ax,ay,az);
		return;
	}
	if(!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))){
		recipNorm=invSqrt(ax*ax+ay*ay+az*az);
		ax*=recipNorm;
		ay*=recipNorm;
		az*=recipNorm;
		
		recipNorm=invSqrt(mx*mx+my*my+mz*mz);
		mx*=recipNorm;
		my*=recipNorm;
		mz*=recipNorm;
		
		p0p0 = p0*p0;
		p0p1 = p0*p1;
		p0p2 = p0*p2;
		p0p3 = p0*p3;
		p1p1 = p1*p1;
		p1p2 = p1*p2;
		p1p3 = p1*p3;
		p2p2 = p2*p2;
		p2p3 = p2*p3;
		p3p3 = p3*p3;
		
		hx=2.0f*(mx*(0.5f-p2p2-p3p3)+my*(p1p2-p0p3)+mz*(p1p3+p0p2));
		hy=2.0f*(mx*(p1p2+p0p3)+my*(0.5f-p1p1-p3p3)+mz*(p2p3-p0p1));
		
		bx=sqrt(hx*hx+hy*hy);
		bz=2.0f*(mx*(p1p3-p0p2)+my*(p2p3+p0p1)+mz*(0.5f-p1p1-p2p2));
		
		halfvx=p1p3-p0p2;
		halfvy=p0p1+p2p3;
		halfvz=p0p0-0.5f+p3p3;
		halfwx=bx*(0.5f-p2p2-p3p3)+bz*(p1p3-p0p2);
		halfwy=bx*(p1p2-p0p3)+bz*(p0p1+p2p3);
		halfwz=bx*(p0p2+p1p3)+bz*(0.5f-p1p1-p2p2);
		
		halfex=(ay*halfvz-az*halfvy)+(my*halfwz-mz*halfwy);
		halfey=(az*halfvx-ax+halfvz)+(mz*halfwx-mx*halfwz);
		halfez=(ax*halfvy-ay*halfvx)+(mx*halfwy-my*halfwx);
		
		if(twoKi>0.0f){
			integralFBx+=twoKi*halfex*(1.0f/sampleFreq);
			integralFBy+=twoKi*halfey*(1.0f/sampleFreq);
			integralFBz+=twoKi*halfez*(1.0f/sampleFreq);
			
			gx+=integralFBx;
			gy+=integralFBy;
			gz+=integralFBz;
			
		}
		else {
			integralFBx=0.0f;
			integralFBy=0.0f;
			integralFBz=0.0f;
			
			
		}
		gx+=twoKp*halfex;
		gy+=twoKp*halfey;
		gz+=twoKp*halfez;
		
	}
	gx*=(0.5f*(1.0f/sampleFreq));
	gy*=(0.5f*(1.0f/sampleFreq));
	gz*=(0.5f*(1.0f/sampleFreq));
	
	qa=p0;
	qb=p1;
	qc=p2;
	qd=p3;
	
//	p0+=(-qb*gx-qc*gy-qc*gz);
//	p1+=(qa*gx+qc*gz-qc*gy);
//	p2+=(qa*gy-qb*gz+qc*gx);
//	p3+=(qa*gz+qb*gy-qc*gx);
	p0+=(			-qb*gx-qc*gy-qd*gz);
	p1+=(qa*gx			+qc*gz-qd*gy);
	p2+=(qa*gy-qb*gz			+qd*gx);
	p3+=(qa*gz+qb*gy-qc*gx			);
	
	
	recipNorm=invSqrt(p0*p0+p1*p1+p2*p2+p3*p3);
	p0*=recipNorm;
	p1*=recipNorm;
	p2*=recipNorm;
	p3*=recipNorm;
	
	
	Anglex=asin(2*(p0*p2-p1*p3 ))* 57.2957795f; // ¸©Ñö
	Angley=asin(2*(p0*p1+p2*p3 ))* 57.2957795f; // ºá¹ö
	Anglez=asin(2.0f*(p1*p2-p0*p3))*57.2957795f;

//	recode();
	
}




void MahonyAHRSupdateIMU(float gx,float gy,float gz,float ax,float ay,float az)
{
	float recipNorm;
	float halfvx,halfvy,halfvz;
	float halfex,halfey,halfez;
	float qa,qb,qc,qd;
	
	if(!((ax==0.0f)&&(ay==0.0f)&&(az==0.0f))){
		recipNorm=1.0/sqrt(ax*ax+ay*ay+az*az);
		ax*=recipNorm;
		ay*=recipNorm;
		az*=recipNorm;
		
		halfvx=p1*p3-p0*p2;
		halfvy=p0*p1+p2*p3;
		halfvz=p0*p0-0.5f+p3*p3;
		

	halfex=(ay*halfvz-az*halfvy);
	halfey=(az*halfvx-ax*halfvz);
	halfez=(ax*halfvy-ay*halfvx);
//	test_angle_error_x=ax;
//	test_angle_error_y=ay;
//	test_angle_error_z=az;
		
		
		
		if(twoKi>0.0f){
			
			integralFBx+=twoKi*halfex*(1.0f/sampleFreq);
			integralFBy+=twoKi*halfey*(1.0f/sampleFreq);
			integralFBz+=twoKi*halfez*(1.0f/sampleFreq);
			
			gx+=integralFBx;
			gy+=integralFBy;
			gz+=integralFBz;
			
		}
		else{
			integralFBx=0.0f;
			integralFBy=0.0f;
			integralFBz=0.0f;
			
		}
		gx+=twoKp*halfex;
		gy+=twoKp*halfey;
		gz+=twoKp*halfez;
	}
//	gx*=(0.5/(1+0.5*twoKp)*(1.0f/sampleFreq));
//	gy*=(0.5/(1+0.5*twoKp)*(1.0f/sampleFreq));
//	gz*=(0.5/(1+0.5*twoKp)*(1.0f/sampleFreq));
	
	gx*=(0.5*(1.0f/sampleFreq));
	gy*=(0.5*(1.0f/sampleFreq));
	gz*=(0.5*(1.0f/sampleFreq));
	
	qa=p0;
	qb=p1;
	qc=p2;
	qd=p3;
	
//	p0+=(-qb*gx-qc*gy-qc*gz);
//	p1+=(qa*gx+qc*gz-qc*gy);
//	p2+=(qa*gy-qb*gz+qc*gx);
//	p3+=(qa*gz+qb*gy-qc*gx);
	p0+=(			-qb*gx-qc*gy-qd*gz);
	p1+=(qa*gx			+qc*gz-qd*gy);
	p2+=(qa*gy-qb*gz			+qd*gx);
	p3+=(qa*gz+qb*gy-qc*gx			);
	
	recipNorm=invSqrt(p0*p0+p1*p1+p2*p2+p3*p3);
	p0*=recipNorm;
	p1*=recipNorm;
	p2*=recipNorm;
	p3*=recipNorm;
	
//	
	Anglex=asin(2.0f*(p0*p2-p1*p3 ))* 57.2957795f; // ¸©Ñö
	Angley=asin(2.0f*(p0*p1+p2*p3 ))* 57.2957795f; // ºá¹ö
	Anglez=asin(2.0f*(p1*p2-p0*p3))*57.2957795f;
//	
	
//	recode();
	
	
	
	
	
	
}

void recode()
{
	double Q[4];
	double A11,A12,A13,A23,A33,P_f_1,P_f_2,R_f_1,R_f_2,Y_f,P_si,R_si,Y_si;
Q[0] = p2;
Q[1] = p3;
Q[2] = p1;
Q[3] = p0;


//==========================================================
A11 = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];
A12 = 2*(Q[0]*Q[1] + Q[2]*Q[3]);
A13 = 2*(Q[0]*Q[2] - Q[1]*Q[3]);
A23 = 2*(Q[1]*Q[2] + Q[0]*Q[3]);
A33 = -Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3];

//==========================================================
P_f_1 = atan(A12/A11);
P_f_2 = -3.14f * Sign(A12) + atan(A12/A11);


R_f_1 = atan(A23/A33);
R_f_2 = -3.14f * Sign(A23) + atan(A23/A33);


Y_f = -3.14f * Sign(A13) - asin(-A13);


//==========================================================
if(A11>0)
{
P_si = (int)(P_f_2*1800/3.14f);
}
else
{
P_si = (int)(P_f_1*1800/3.14f);
}


if(A33<0)
{
R_si = (R_f_1*1800/3.14f);
}
else
{
R_si =(R_f_2*1800/3.14f);
}

Y_si =(Y_f*1800/3.14f);


//==========================================================
//if(P_si<0)
//{
//P_si = 32768 - P_si;
//}


//if(R_si<0)
//{
//R_si = 32768 - R_si;
//}


//if(Y_f<0)
//{
//Y_si = 32768 - Y_si;
//}

//=======================
// Pitch_out = P_si + 1800;
// Roll_out = R_si + 1800;
// Yaw_out = Y_si + 1800;


//=======================
Anglex = P_si+1800;
Angley = R_si+1800;
Anglez = Y_si+1800;
}



float invSqrt(float x){
	float halfx=0.5f*x;
	float y=x;
	long i=*(long*)&y;
	i=0x5f3759df-(i>>1);
	y=*(float*)&i;
	y=y*(1.5f-(halfx*y*y));
	return y;
}

float Sign(float x)
{
if(x>0)
return 1;
else
return -1;
}

