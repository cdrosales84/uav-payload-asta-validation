#include "mex.h"
#include "SimQuad.h"

/* Input Arguments */

#define	Y_IN1	prhs[0]
#define	Y_IN2	prhs[1]
#define	Y_IN3	prhs[2]
#define	Y_IN4	prhs[3]
#define	Y_IN5	prhs[4]

/* output Arguments*/

#define	YP_OUT	plhs[0]

#if !defined(max)
#define	max(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(min)
#define	min(A, B)	((A) < (B) ? (A) : (B))
#endif

////FUNCION LLAMADA POR MATLAB (en este caso "simulation")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double	*u1, *u2, *u3, *u4, *u5;
    unsigned int	m1,n1,m2,n2,m3,n3,m4,n4,m5,n5;
    bool *yp;
    int j;
  		
	/* Check for proper number of arguments */
	if ((nlhs == 1) && (nrhs == 0)) //pregunta si hay un argumento de salida y ninguno de entrada
    {
        /* Create a matrix for the return argument */
        YP_OUT = mxCreateLogicalMatrix(1, 1);
        /* Assign pointers to the various parameters */
        yp = mxGetLogicals(YP_OUT);
        
        CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
        m_SharedData_Sim.Acquire();
        yp[0] = m_SharedData_Sim->running;
        m_SharedData_Sim.Release();
		return;
	}
    
    if (nlhs != 0) //pregunta si hay algún argumento de salida
    {
		mexErrMsgTxt("Se requiere cero (0) o un (1) argumento de salida");
	}
  
	if ((nrhs == 0) || (nrhs > 5))
	{
        mexErrMsgTxt("Se requieren uno (1), tres (3) o cinco (5) argumentos de entrada cuando no hay argumento de salida");
    }
    else
    {
        if (nrhs == 1)
        {
            m1 = mxGetM(Y_IN1);
            n1 = mxGetN(Y_IN1);
            
            /* Check the dimensions of Y */
            if (mxIsNumeric(Y_IN1) && !mxIsComplex(Y_IN1) &&
                    (max(m1,n1) == 1) && (min(m1,n1) == 1))
            {
                /* Y is 1 X 1. */
                /* Assign pointers to the various parameters */
                CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
                m_SharedData_Sim.Acquire();
                
                m_SharedData_Sim->running = false;
                
                m_SharedData_Sim.Release();
            }
            else
            {
                mexErrMsgTxt("Se requiere que u1 sea un escalar (para finalizar simulacion)");
            }
        }
        else if (nrhs == 3)
        {
            m1 = mxGetM(Y_IN1);
            n1 = mxGetN(Y_IN1);
            
            /* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
            if (!mxIsNumeric(Y_IN1) || mxIsComplex(Y_IN1) ||
                (max(m1,n1) != 1) || (min(m1,n1) != 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de obstáculo, ejemplo: simulation(numObs,[x y z],[dx dy dz])");
            }
            else
            {
                /* Assign pointers to the various parameters */
                u1 = mxGetPr(Y_IN1);
                if ((floor(u1[0]) != ceil(u1[0])) ||(u1[0] > 10) ||(u1[0] < 1))
                {
                    mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10) indicando el numero de obstáculo");
                }
                else
                {
                    m2 = mxGetM(Y_IN2);
                    n2 = mxGetN(Y_IN2);
                    m3 = mxGetM(Y_IN3);
                    n3 = mxGetN(Y_IN3);
                    
                    if (!mxIsNumeric(Y_IN2) || mxIsComplex(Y_IN2) ||
                        (max(m2,n2) != 3) || (min(m2,n2) != 1))
                    {
                        mexErrMsgTxt("Se requiere que u2 sea un vector un vector de 1 x 3 (o 3 x 1) para indicar posición [x y z] de obstáculo");
                    }
                    if (!mxIsNumeric(Y_IN3) || mxIsComplex(Y_IN3) ||
                        (max(m3,n3) != 3) || (min(m3,n3) != 1))
                    {
                        mexErrMsgTxt("Se requiere que u3 sea un vector un vector de 1 x 3 (o 3 x 1) para indicar tamaño [dx dy dz] de obstáculo");
                    }
                    
                    u2 = mxGetPr(Y_IN2);
                    u3 = mxGetPr(Y_IN3);
                    
                    CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
                    m_SharedData_Sim.Acquire();
                    
                    m_SharedData_Sim->Obs[(int)u1[0]].x = u2[0];
                    m_SharedData_Sim->Obs[(int)u1[0]].y = u2[1];
                    m_SharedData_Sim->Obs[(int)u1[0]].z = u2[2];
   
                    m_SharedData_Sim->dObs[(int)u1[0]].x = u3[0];
                    m_SharedData_Sim->dObs[(int)u1[0]].y = u3[1];
                    m_SharedData_Sim->dObs[(int)u1[0]].z = u3[2];
                    
                    m_SharedData_Sim.Release();
                }
            }
        }
        else if (nrhs == 5)
        {
            ///Cinco entradas//
            m1 = mxGetM(Y_IN1);
            n1 = mxGetN(Y_IN1);
            
            m2 = mxGetM(Y_IN2);
            n2 = mxGetN(Y_IN2);
            
            m3 = mxGetM(Y_IN3);
            n3 = mxGetN(Y_IN3);
            
            m4 = mxGetM(Y_IN4);
            n4 = mxGetN(Y_IN4);
            
            m5 = mxGetM(Y_IN5);
            n5 = mxGetN(Y_IN5);
            
            /* Check the dimensions of Y1 */
            if (!mxIsNumeric(Y_IN1) || mxIsComplex(Y_IN1) ||
                    (max(m1,n1) != 12) || (min(m1,n1) != 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un vector de 1 x 12 (o 12 x 1)");

            }
            
            /* Assign pointers to the various parameters */
            u1 = mxGetPr(Y_IN1);
            
            if ((floor(u1[0]) != ceil(u1[0])) ||(u1[0] > maxNumQuads) ||(u1[0] < 1))
            {
                mexErrMsgTxt("Se requiere que u1(1) sea un entero (entre 1 y 10) indicando el numero de helicopteros a simular");
                return;
            }
            if ((floor(u1[11]) != ceil(u1[11])) ||(u1[11] > 10) ||(u1[11] < 0))
            {
                mexErrMsgTxt("Se requiere que u1(12) sea un entero (entre 0 y 10) indicando el numero de obstáculos a simular");
            }
            
            if (!mxIsNumeric(Y_IN2) || mxIsComplex(Y_IN2) ||
                        (max(m2,n2) != (int)u1[0] * 4) || (min(m2,n2) != 1))
            {
                    mexErrMsgTxt("Se requiere que u2 sea un vector de 1 x n*4 (o n*4 x 1), siendo n el numero de helicopteros");
            }
            
            if (!mxIsNumeric(Y_IN3) || mxIsComplex(Y_IN3) ||
                        (max(m3,n3) != (int)u1[0]) || (min(m3,n3) != 1))
            {
                    mexErrMsgTxt("Se requiere que u3 sea un vector de 1 x n (o n x 1), siendo n el numero de helicopteros");
            }
            
            u2 = mxGetPr(Y_IN2);
            u3 = mxGetPr(Y_IN3);
            
            if(u1[11] != 0)
            {
                if (!mxIsNumeric(Y_IN4) || mxIsComplex(Y_IN4) ||
                            (max(m4,n4) != (int)u1[11] * 3) || (min(m4,n4) != 1))
                {
                        mexErrMsgTxt("Se requiere que u4 sea un vector de 1 x nObs (o nObs x 1), siendo nObs el numero de obstáculos");
                }

                if (!mxIsNumeric(Y_IN5) || mxIsComplex(Y_IN5) ||
                            (max(m5,n5) != (int)u1[11] * 3) || (min(m5,n5) != 1))
                {
                        mexErrMsgTxt("Se requiere que u5 sea un vector de 1 x nObs (o nObs x 1), siendo nObs el numero de obstáculos");
                }
                
                u4 = mxGetPr(Y_IN4);
                u5 = mxGetPr(Y_IN5);
            }         
            
            CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
            m_SharedData_Sim.Acquire();
            
            /* Do the computations */
            m_SharedData_Sim->number_quads = (int)u1[0];
            m_SharedData_Sim->time = u1[1];
            m_SharedData_Sim->weight_load = u1[2];
            m_SharedData_Sim->height_load = u1[3];
            m_SharedData_Sim->rope_length = u1[4];
            m_SharedData_Sim->spring_friction = u1[5];// Spring Inner Friction Constant (0.2 default)
            m_SharedData_Sim->spring_constant = u1[6];// Spring Constant In The Rope (10000.0 default)
            m_SharedData_Sim->air_friction = u1[7];// Air Friction Constant (0.02 default)
            m_SharedData_Sim->ground_repel = u1[8];// Ground Repel Constant (100 default)
            m_SharedData_Sim->ground_slide = u1[9];// Ground Slide Friction Constant (0.2 default)
            m_SharedData_Sim->ground_absorption = u1[10];// Ground Absoption Constant (2.0 default)
            m_SharedData_Sim->number_obs = (int)u1[11];//
            
            for(j=0; j < (int)u1[0]; j++)
            {
                m_SharedData_Sim->Xo[j] = u2[j*4];
                m_SharedData_Sim->Yo[j] = u2[j*4+1];
                m_SharedData_Sim->Zo[j] = u2[j*4+2];
                m_SharedData_Sim->Tho[j] = u2[j*4+3];
            }
            
            for(j=0; j < (int)u1[0]; j++)
            {
                if(u3[j] == 0)
                    m_SharedData_Sim->laser[j] = false;
                else
                    m_SharedData_Sim->laser[j] = true;
            }
            
            for(j=0; j < (int)u1[11]; j++)
            {
                m_SharedData_Sim->Obs[j].x = u4[j*3];
                m_SharedData_Sim->Obs[j].y = u4[j*3+1];
                m_SharedData_Sim->Obs[j].z = u4[j*3+2];
   
                m_SharedData_Sim->dObs[j].x = u5[j*3];
                m_SharedData_Sim->dObs[j].y = u5[j*3+1];
                m_SharedData_Sim->dObs[j].z = u5[j*3+2];
            }
            
            if ((int)u1[11] < 10)
            {
                for(j=(int)u1[11]; j < 10; j++)
                {
                    m_SharedData_Sim->Obs[j].x = 9999;
                    m_SharedData_Sim->Obs[j].y = 9999;
                    m_SharedData_Sim->Obs[j].z = 9999;
                    
                    m_SharedData_Sim->dObs[j].x = 1;
                    m_SharedData_Sim->dObs[j].y = 1;
                    m_SharedData_Sim->dObs[j].z = 1;
                }
            }
            
            m_SharedData_Sim->running = true;
            
            m_SharedData_Sim.Release();
        }
        else
        {
            mexErrMsgTxt("Se requieren uno (1), tres (3) o cinco (5) argumentos de entrada");
        }
	}
    return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "simulation")/////
