/*
 * File: hcc4_4.c
 * V. Pilipchuk/586-986-4603
 * Abstract: includes minimization of control temporal rates
 * Date: March/2010
 *  
 *
 *  The outputs are
 *   y[0] = dQ(1)
 *   y[1] = dQ(2)
 *   y[2] = dQ(3)
 *   y[3] = dQ(4)
 *
 * See matlabroot/simulink/src/sfuntmpl_doc.c for more details
 */

#define S_FUNCTION_NAME  hcc4_4
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"

#define NUMOFINPUTS     45
#define NUMOFOUTPUTS    4

#define BUFF_LEN        10      /* Length of input buffer */

#define MAX_OFFSET      0.01
#define EPSILON         0.01

/* ========================================================== */

#define PI      3.1415
#define TRUE    1
#define FALSE   0
#define MatSize 4
#define dim1 4
#define dim2 4
#define dim3 1
 


#include "math.h"
int  invert_matrix(real_T mat[][MatSize],real_T inv[][MatSize]);
void identity_matrix (real_T I[MatSize][MatSize]);
void product_matrix(real_T mat1[dim1][dim2],real_T mat2[dim2][dim3],real_T mat3[dim1][dim3]);
void sum_matrix( real_T mat1[dim1][dim2],real_T mat2[dim1][dim2],real_T mat3[dim1][dim2]);

/* ========================================================== */


/*==================*
 * Global Variables *
 *==================*/
// static real_T r_time[BUFF_LEN], t_offset[BUFF_LEN];
// static int_T buff_index;

/*====================*
 * S-function methods *
 *====================*/
static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    if (!ssSetNumInputPorts(S, 1)) {
        return;
    }
    ssSetInputPortWidth(S, 0, NUMOFINPUTS);
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) {
        return;
    }
    ssSetOutputPortWidth(S, 0, NUMOFOUTPUTS);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(S, 0);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS   /* Change to #undef to remove function */
static void mdlInitializeConditions(SimStruct *S)
{
   
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    
    real_T Fz1      	= *uPtrs[0];
    real_T Fz2      	= *uPtrs[1];
    real_T Fz3      	= *uPtrs[2];
    real_T Fz4      	= *uPtrs[3];
    
    real_T Fx1      	= *uPtrs[4];
    real_T Fx2      	= *uPtrs[5];
    real_T Fx3      	= *uPtrs[6];
    real_T Fx4      	= *uPtrs[7];
    
    real_T Fy1      	= *uPtrs[8];
    real_T Fy2      	= *uPtrs[9];
    real_T Fy3      	= *uPtrs[10];
    real_T Fy4      	= *uPtrs[11];
    
    real_T mux1      	= *uPtrs[12];
    real_T mux2     	= *uPtrs[13];
    real_T mux3      	= *uPtrs[14];
    real_T mux4     	= *uPtrs[15];
    real_T muy1      	= *uPtrs[16];
    real_T muy2      	= *uPtrs[17];
    real_T muy3      	= *uPtrs[18];
    real_T muy4      	= *uPtrs[19];
    
    real_T Q1_max      	= *uPtrs[20];
    real_T Q2_max      	= *uPtrs[21];
    real_T Q3_max      	= *uPtrs[22];
    real_T Q4_max      	= *uPtrs[23];
    
    real_T Q1_min      	= *uPtrs[24];
    real_T Q2_min      	= *uPtrs[25];
    real_T Q3_min      	= *uPtrs[26];
    real_T Q4_min      	= *uPtrs[27];
    
     
    real_T delta1   	= *uPtrs[28];
    real_T delta2   	= *uPtrs[29];
    real_T delta3   	= *uPtrs[30];
    real_T delta4   	= *uPtrs[31];
    
    real_T Ex       	= *uPtrs[32];
    real_T Ey       	= *uPtrs[33];
    real_T Ez       	= *uPtrs[34];
    
    real_T wFx      	= *uPtrs[35];
    real_T wFy      	= *uPtrs[36];
    real_T wGz      	= *uPtrs[37];
    
    real_T wxd1      	= *uPtrs[38];
    real_T wxd2      	= *uPtrs[39];
    real_T wxd3      	= *uPtrs[40];
    real_T wxd4      	= *uPtrs[41];
    
    real_T gamma    	= *uPtrs[42];
    int_T n         	= *uPtrs[43];
    
    real_T Q_gain   	= *uPtrs[44];
   

       
    real_T num[MatSize][1], den[MatSize][MatSize], inv_den[MatSize][MatSize], df[MatSize][1];
    real_T af11, af12, af13, af14, af21, af22, af23, af24, af31, af32, af33, af34;
    
    real_T Fx_max1, Fy_max1, Fx_max2, Fy_max2, Fx_max3, Fy_max3, Fx_max4, Fy_max4;
    
    real_T ro21, ro22, ro23, ro24, wf1, wf2, wf3, wf4;
    real_T a, b, tr, Reff;
      
    a        = 1.3265;
    b        = 1.3685;
    tr       = 1.603;
    Reff     = 0.323;
  
 
 
       
    Fx_max1  =mux1*Fz1;
    Fy_max1  =muy1*Fz1;
    Fx_max2  =mux2*Fz2;
    Fy_max2  =muy2*Fz2;
    Fx_max3  =mux3*Fz3;
    Fy_max3  =muy3*Fz3;
    Fx_max4  =mux4*Fz4;
    Fy_max4  =muy4*Fz4;

    ro21=pow((Fx1/Fx_max1),2) + pow((Fy1/Fy_max1),2);
    ro22=pow((Fx2/Fx_max2),2) + pow((Fy2/Fy_max2),2);
    ro23=pow((Fx3/Fx_max3),2) + pow((Fy3/Fy_max3),2);
    ro24=pow((Fx4/Fx_max4),2) + pow((Fy4/Fy_max4),2);
    
   
    wf1=gamma*pow(ro21,n); 
    wf2=gamma*pow(ro22,n);
    wf3=gamma*pow(ro23,n);
    wf4=gamma*pow(ro24,n);
   
    if (Fx1 > 0)
			wf1 = wf1 + gamma*pow((Reff*Fx1/Q1_max),2*n);
	else    wf1 = wf1 + gamma*pow((Reff*Fx1/Q1_min),2*n);
    
    if (Fx2 > 0)
			wf2 = wf2 + gamma*pow((Reff*Fx2/Q2_max),2*n);
	else    wf2 = wf2 + gamma*pow((Reff*Fx2/Q2_min),2*n);
    
    if (Fx3 > 0)
			wf3 = wf3 + gamma*pow((Reff*Fx3/Q3_max),2*n);
	else    wf3 = wf3 + gamma*pow((Reff*Fx3/Q3_min),2*n);
    
    if (Fx4 > 0)
			wf4 = wf4 + gamma*pow((Reff*Fx4/Q4_max),2*n);
	else    wf4 = wf4 + gamma*pow((Reff*Fx4/Q4_min),2*n);
    
//      wf1 = 1000;
//      wf2 = 1000;
//      wf3 = 1000;
//      wf4 = 1000;
    
    
    af11=   cos(delta1);
    af12=   cos(delta2);
    af13=   cos(delta3);
    af14=   cos(delta4);
    
    af21=   sin(delta1);
    af22=   sin(delta2);
    af23=   sin(delta3);
    af24=   sin(delta4);
    
    af31=   -(tr/2)*af11+a*af21;
    af32=    (tr/2)*af12+a*af22;
    af33=   -(tr/2)*af13-a*af23;
    af34=    (tr/2)*af14-a*af24;
    
        
    num[0][0]=  af11*Ex*wFx + af21*Ey*wFy + af31*Ez*wGz - Fx1*wf1;  
    num[1][0]=  af12*Ex*wFx + af22*Ey*wFy + af32*Ez*wGz - Fx2*wf2;
    num[2][0]=  af13*Ex*wFx + af23*Ey*wFy + af33*Ez*wGz - Fx3*wf3;
    num[3][0]=  af14*Ex*wFx + af24*Ey*wFy + af34*Ez*wGz - Fx4*wf4;
   
    den[0][0]=  af11*af11*wFx + af21*af21*wFy + af31*af31*wGz + wf1 + wxd1;    
    den[0][1]=  af11*af12*wFx + af21*af22*wFy + af31*af32*wGz;
    den[0][2]=  af11*af13*wFx + af21*af23*wFy + af31*af33*wGz;
    den[0][3]=  af11*af14*wFx + af21*af24*wFy + af31*af34*wGz;
    den[1][0]=  af11*af12*wFx + af21*af22*wFy + af31*af32*wGz;
    den[1][1]=  af12*af12*wFx + af22*af22*wFy + af32*af32*wGz + wf2 + wxd2;
    den[1][2]=  af12*af13*wFx + af22*af23*wFy + af32*af33*wGz;
    den[1][3]=  af12*af14*wFx + af22*af24*wFy + af32*af34*wGz;
    den[2][0]=  af11*af13*wFx + af21*af23*wFy + af31*af33*wGz;
    den[2][1]=  af12*af13*wFx + af22*af23*wFy + af32*af33*wGz;
    den[2][2]=  af13*af13*wFx + af23*af23*wFy + af33*af33*wGz + wf3 + wxd3;
    den[2][3]=  af13*af14*wFx + af23*af24*wFy + af33*af34*wGz;
    den[3][0]=  af11*af14*wFx + af21*af24*wFy + af31*af34*wGz;
    den[3][1]=  af12*af14*wFx + af22*af24*wFy + af32*af34*wGz;
    den[3][2]=  af13*af14*wFx + af23*af24*wFy + af33*af34*wGz;
    den[3][3]=  af14*af14*wFx + af24*af24*wFy + af34*af34*wGz + wf4 + wxd4;
    
    

    invert_matrix(den,inv_den);
    product_matrix(inv_den,num,df);
    
   
     
        y[0] = df[0][0]*Reff*Q_gain;            /* dQ(1) */
        y[1] = df[1][0]*Reff*Q_gain;            /* dQ(2) */
        y[2] = df[2][0]*Reff*Q_gain;            /* dQ(3) */
        y[3] = df[3][0]*Reff*Q_gain;            /* dQ(4) */
      
       
}

/* ====================== BEGIN MATRIX OPERATIONS ===================== */


#define  TRUE   1
#define  FALSE  0


int invert_matrix(real_T mat[][MatSize],real_T inv[][MatSize])

/*** Returns a value of 0 if mat is invertable.		***/
/*** Returns a value of 1 if mat is not invertable. ***/
/*** inv exits this function as the inverse of mat	***/
/*** mat exits this function as the identity matrix ***/
{
	real_T	m,temp[MatSize][MatSize];
	int_T	i,j,col,finder;

	for (i=0; i<MatSize; ++i)
		for (j=0; j<MatSize; ++j)
			temp[i][j] = mat[i][j];

	identity_matrix (inv);

	for (i=0; i<MatSize; ++i)
	{	finder = i;
		if (temp[i][i]==0.0)	/*** Find a row with a non-zero ***/
								/*** ith element and add it to  ***/
								/*** the ith row.               ***/

		{	while ( (finder<MatSize) && (temp[finder][i]==0.0) )
				++finder;
			if (finder<MatSize)
			for (col=0; col<MatSize; ++col)
			{	temp[i][col] += temp[finder][col];
				inv[i][col] += inv[finder][col];
			}
		}
		if (finder<MatSize)
		{	m = temp[i][i];
			for (col=0; col<MatSize; ++col)
			{	temp[i][col] /= m;
				inv[i][col] /= m;
			}
			for (j=i+1; j<MatSize; ++j)
			{	m = - temp[j][i];
				for (col=0; col<MatSize; ++col)
				{	temp[j][col] += ( temp[i][col] * m );
					inv[j][col] += ( inv[i][col] * m );
				}
			}
		}
		/* else printf ("\nMatrix uninvertable."); */
	}

	if (finder<MatSize)
	for (i=0; i<MatSize; ++i)
		for (j=i+1; j<MatSize; ++j)
		{	m = - temp[i][j];
			for (col=0; col<MatSize; ++col)
			{	temp[i][col] += ( temp[j][col] * m );
				inv[i][col] += ( inv[j][col] * m );
			}
		}

	if (finder<MatSize)
	     return (0);
	else return (1);

} /*** End invert_matrix () ***/

void identity_matrix (real_T I[MatSize][MatSize])

/*** N is #defined; m_size is global variable ***/
{
	int_T	row,col;
	for (row=0; row<MatSize; ++row)
		for (col=0; col<MatSize; ++col)
			if (row == col)
			I[row][col] = 1.0;
			else I[row][col] = 0.0;
} /*** End identity_matrix () ***/



void product_matrix(real_T mat1[dim1][dim2],real_T mat2[dim2][dim3],real_T mat3[dim1][dim3])
{
	int_T r, c, k;
	for (r = 0; r < dim1; r++)                      /* r - row number */
	{
		for (c = 0; c < dim3; c++)
			mat3[r][c] = 0.;
		for (k = 0; k < dim2; k++)
			for (c = 0; c < dim3; c++)              /* c - column number */
				mat3[r][c] += mat1[r][k]*mat2[k][c];
	}
}

void sum_matrix( double mat1[dim1][dim2],double mat2[dim1][dim2],double mat3[dim1][dim2])
{
    int_T r, c;
		for ( r=0; r< dim1; r++)                    /* r - row number */
		    for (c=0; c< dim2; c++)                 /* c - column number */
				mat3[r][c] = mat1[r][c] + mat2[r][c];
}




 /* ========================= END MATRIX OPERATIONS ====================== */
 


static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
