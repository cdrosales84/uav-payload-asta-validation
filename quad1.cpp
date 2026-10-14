#include "mex.h"
#include "SimQuad.h"

/* Input Arguments */

#define	Y_IN1	prhs[0]
#define	Y_IN2	prhs[1]

/* output Arguments*/

#define	YP_OUT	plhs[0]

#if !defined(max)
#define	max(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(min)
#define	min(A, B)	((A) < (B) ? (A) : (B))
#endif

////FUNCION LLAMADA POR MATLAB (en este caso "helicop")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double	*u1, *u2;
	double	*yp;
	unsigned int	m1,n1,m2,n2;
	int k;
	
	/* Check for proper number of arguments */
	if (nrhs == 0)//pregunta si no hay algun argumento de entrada
    {
        if ((nlhs != 1))//pregunta si no hay un argumento de salida
        {
            mexErrMsgTxt("Si no hay un argumento de entrada se requiere un argumento de salida, ejemplo: simCount=quadcopter");
        }
        
        /* Create a matrix for the return argument */
        
        YP_OUT = mxCreateDoubleMatrix(1, 1, mxREAL);
        
        /* Assign pointers to the various parameters */
        
        yp = mxGetPr(YP_OUT);
        
        CSharedStruct<Quads>m_SharedData_Quads(M_COMPARTIDA_QUAD);
        
        m_SharedData_Quads.Acquire();
        
        /* Do the computations*/
        yp[0] = (double)m_SharedData_Quads->simCount;
        
        m_SharedData_Quads.Release();
        
        return;
    }
	if (nrhs == 1) //pregunta si hay un solo argumento de entrada
	{
        //if ((nlhs > 1) || ( nlhs == 0))//pregunta si hay mas de un argumento de salida
        if ((nlhs > 1))//pregunta si hay mas de un argumento de salida
		{
			mexErrMsgTxt("Si hay un argumento de entrada se requiere solo un argumento de salida");
		}
		else
		{
            m1 = mxGetM(Y_IN1);
            n1 = mxGetN(Y_IN1);
            
            /* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
            if (!mxIsNumeric(Y_IN1) || mxIsComplex(Y_IN1) ||
                (max(m1,n1) != 1) || (min(m1,n1) != 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de quadcopter, ejemplo: x=quad(numQuad)");
            }
            else
            {
                /* Assign pointers to the various parameters */
                u1 = mxGetPr(Y_IN1);
                if ((floor(u1[0]) != ceil(u1[0])) ||(u1[0] > 10) ||(u1[0] < 1))
                {
                    mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10) indicando el numero de quadcoptero");
                }
                else
                {
                    /* Create a matrix for the return argument */
                    
                    YP_OUT = mxCreateDoubleMatrix(24, 1, mxREAL);
                    
                    /* Assign pointers to the various parameters */
                    
                    yp = mxGetPr(YP_OUT);
                    
                    CSharedStruct<Quads>m_SharedData_Quads(M_COMPARTIDA_QUAD);
                    
                    m_SharedData_Quads.Acquire();
                    
                    int index = (int)u1[0] - 1;
                    /* Do the computations*/
                    yp[0] = m_SharedData_Quads->quadcopter[index].x;
                    yp[1] = m_SharedData_Quads->quadcopter[index].y;
                    yp[2] = m_SharedData_Quads->quadcopter[index].z;
                    yp[3] = m_SharedData_Quads->quadcopter[index].vx;
                    yp[4] = m_SharedData_Quads->quadcopter[index].vy;
                    yp[5] = m_SharedData_Quads->quadcopter[index].vz;
                    yp[6] = m_SharedData_Quads->quadcopter[index].u;
                    yp[7] = m_SharedData_Quads->quadcopter[index].v;
                    yp[8] = m_SharedData_Quads->quadcopter[index].w;
                    yp[9] = m_SharedData_Quads->quadcopter[index].p;
                    yp[10] = m_SharedData_Quads->quadcopter[index].q;
                    yp[11] = m_SharedData_Quads->quadcopter[index].r;
                    yp[12] = m_SharedData_Quads->quadcopter[index].roll;
                    yp[13] = m_SharedData_Quads->quadcopter[index].pitch;
                    yp[14] = m_SharedData_Quads->quadcopter[index].yaw;
                    yp[15] = m_SharedData_Quads->quadcopter[index].omega1;
                    yp[16] = m_SharedData_Quads->quadcopter[index].omega2;
                    yp[17] = m_SharedData_Quads->quadcopter[index].omega3;
                    yp[18] = m_SharedData_Quads->quadcopter[index].omega4;
                    yp[19] = m_SharedData_Quads->quadcopter[index].vxh;
                    yp[20] = m_SharedData_Quads->quadcopter[index].vyh;
                    yp[21] = m_SharedData_Quads->quadcopter[index].vzh;
                    yp[22] = m_SharedData_Quads->quadcopter[index].wzh;
                    yp[23] = m_SharedData_Quads->quadcopter[index].tiempoSim;
                    
                    m_SharedData_Quads.Release();
                }
            } 
        }
    }
    else if (nrhs == 2)//pregunta si hay dos argumentos de entrada
    {
        if ( nlhs != 0)//pregunta si hay algun argumento de salida
		{
			mexErrMsgTxt("Si hay dos argumentos de entrada no se requiere argumento de salida");
            return;
		}
        
  		m1 = mxGetM(Y_IN1);
		n1 = mxGetN(Y_IN1);
        
        m2 = mxGetM(Y_IN2);
		n2 = mxGetN(Y_IN2);
        
        /* Check the dimensions of Y1 and Y2.  Y1 must be 1 X 1. Y2 must be 1 X 4.*/
        if (!mxIsNumeric(Y_IN1) || mxIsComplex(Y_IN1) || !mxIsNumeric(Y_IN2) || mxIsComplex(Y_IN2) ||
			(max(m1,n1) != 1) || (min(m1,n1) != 1) || (max(m2,n2) != 4) || (min(m2,n2) != 1))
		{
			mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de quadcoptero a comandar y u2 sea un vector de 1 x 4 (o 4 x 1)");
		}
        else
        {
            /* Assign pointers to the various parameters */
            u1 = mxGetPr(Y_IN1);
            u2 = mxGetPr(Y_IN2);
            
            if ((floor(u1[0]) != ceil(u1[0])) ||(u1[0] > 10) ||(u1[0] < 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10) indicando el numero de quadcoptero");
                return;
            }
            
            CSharedStruct<Comandos>m_SharedData_Comandos(M_COMPARTIDA_COM);
            
            m_SharedData_Comandos.Acquire();
            
            int index = (int)u1[0] - 1;
            /* Do the computations */
            m_SharedData_Comandos->v_xh[index] = u2[0];
            m_SharedData_Comandos->v_yh[index] = u2[1];
            m_SharedData_Comandos->v_zh[index] = u2[2];
            m_SharedData_Comandos->w_zh[index] = u2[3];
                
            m_SharedData_Comandos.Release();
        }
	}
	else if (nrhs > 2)//pregunta si hay mas de dos argumentos de entrada
	{
        mexErrMsgTxt("Se requiere un argumento de salida y ninguno o uno de entrada, o dos argumentos de entrada, ejemplo: x=helicop(numHeli); o helicop(numHeli,[vxh vyh vzh wzh])");
	}	
	return;        
}
////FUNCION LLAMADA POR MATLAB (en este caso "helicop")/////
