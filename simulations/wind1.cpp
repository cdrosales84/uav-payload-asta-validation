#include "mex.h"
#include "SimQuad.h"

/* Input Arguments */

#define	Y_IN1	prhs[0]
#define	Y_IN2	prhs[1]
#define	Y_IN3	prhs[2]

#if !defined(max)
#define	max(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(min)
#define	min(A, B)	((A) < (B) ? (A) : (B))
#endif

////FUNCION LLAMADA POR MATLAB (en este caso "windn")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double	*u1, *u2, *u3;
    unsigned int	m1,n1,m2,n2,m3,n3;
    bool *yp;
    int j;
  		
	/* Check for proper number of arguments */
    if (nlhs != 0) //pregunta si hay algún argumento de salida
    {
		mexErrMsgTxt("Se requiere cero (0) argumento de salida");
	}
  
	if ((nrhs < 2) || (nrhs > 3))
	{
        mexErrMsgTxt("Se requieren dos (2) o tres (3) argumentos de entrada");
    }
    
    m1 = mxGetM(Y_IN1);
    n1 = mxGetN(Y_IN1);

    /* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
    if (!mxIsNumeric(Y_IN1) || mxIsComplex(Y_IN1) ||
        (max(m1,n1) != 1) || (min(m1,n1) != 1))
    {
        mexErrMsgTxt("Se requiere que u1 sea un escalar indicando tipo de viento: 1 para heli, 2 para cuerda o 3 para carga");
    }

    /* Assign pointers to the various parameters */
    u1 = mxGetPr(Y_IN1);
    if ((floor(u1[0]) != ceil(u1[0])) ||(u1[0] > 3) ||(u1[0] < 1))
    {
        mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 3) indicando tipo de viento");
    }
    
    m2 = mxGetM(Y_IN2);
    n2 = mxGetN(Y_IN2);

    if(u1[0] == 3)
    {
        /* Check the dimensions of Y2.  Y2 must be 1 X 3 or 3 X 1.*/
        if (!mxIsNumeric(Y_IN2) || mxIsComplex(Y_IN2) ||
            (max(m2,n2) != 3) || (min(m2,n2) != 1))
        {
            mexErrMsgTxt("Se requiere que u2 sea un vector un vector de 1 x 3 (o 3 x 1) para indicar el viento [wx wy wz] que afecta a la carga");
        }

        u2 = mxGetPr(Y_IN2);
        CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
        m_SharedData_Sim.Acquire();

        m_SharedData_Sim->LoadWind.x = u2[0];
        m_SharedData_Sim->LoadWind.y = u2[1];
        m_SharedData_Sim->LoadWind.z = u2[2];

        m_SharedData_Sim.Release();
    }
    else
    {
        /* Check the dimensions of Y2.  Y2 must be 1 X 1.*/
        if (!mxIsNumeric(Y_IN2) || mxIsComplex(Y_IN2) ||
            (max(m2,n2) != 1) || (min(m2,n2) != 1))
        {
            mexErrMsgTxt("Se requiere que u2 sea un escalar indicando el número de heli o de cuerda");
        }

        u2 = mxGetPr(Y_IN2);
        if ((floor(u2[0]) != ceil(u2[0])) ||(u2[0] > 10) ||(u2[0] < 1))
        {
            mexErrMsgTxt("Se requiere que u2 sea un entero (entre 1 y 10) indicando el numero de heli o de cuerda");
        }
        
        m3 = mxGetM(Y_IN3);
        n3 = mxGetN(Y_IN3);

        if (!mxIsNumeric(Y_IN3) || mxIsComplex(Y_IN3) ||
            (max(m3,n3) != 3) || (min(m3,n3) != 1))
        {
            mexErrMsgTxt("Se requiere que u3 sea un vector un vector de 1 x 3 (o 3 x 1) para indicar el viento [wx wy wz]");
        }

        u3 = mxGetPr(Y_IN3);

        CSharedStruct<ConfigSim>m_SharedData_Sim(M_COMPARTIDA_SIM);
        m_SharedData_Sim.Acquire();
        int index = (int)u2[0] - 1;
        if(u1[0] == 1)
        {
            m_SharedData_Sim->QuadWind[index].x = u3[0];
            m_SharedData_Sim->QuadWind[index].y = u3[1];
            m_SharedData_Sim->QuadWind[index].z = u3[2];
        }
        else
        {
            m_SharedData_Sim->RopeWind[index].x = u3[0];
            m_SharedData_Sim->RopeWind[index].y = u3[1];
            m_SharedData_Sim->RopeWind[index].z = u3[2];
        }
        m_SharedData_Sim.Release();
    }
    return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "wind")/////