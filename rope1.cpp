#include "mex.h"
#include "SimQuad.h"

/* Input Arguments */

#define	Y_IN	prhs[0]

/* output Arguments*/

#define	YP_OUT	plhs[0]

#if !defined(max)
#define	max(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(min)
#define	min(A, B)	((A) < (B) ? (A) : (B))
#endif

////FUNCION LLAMADA POR MATLAB (en este caso "rope")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double	*u;
    double   *yp;
    const int	m = numParticles;
    const int   n = 3;
    unsigned int	m1,n1;
    int j, k;
    
    /* Check for proper number of arguments */
    if (nrhs == 0)
    {
        if (nlhs != 1)
        {
            //mexErrMsgTxt("Se requiere un argumento de salida, ejemplo: tSimRope=rope");
            mexErrMsgTxt("Se requiere un argumento de salida, ejemplo: simCount=rope");
        }
        else
        {
            /* Create a matrix for the return argument */
            
            YP_OUT = mxCreateDoubleMatrix(1, 1, mxREAL);
            
            /* Assign pointers to the various parameters */
            
            yp = mxGetPr(YP_OUT);
            
            CSharedStruct<RopePos>m_SharedData_Rope(M_COMPARTIDA_ROPE);
            
            m_SharedData_Rope.Acquire();
            
            /* Do the computations*/
            //yp[0] = m_SharedData_Rope->tiempoSim;
            yp[0] = (double)m_SharedData_Rope->simCount;
            
            m_SharedData_Rope.Release();
        }
        return;
    }
    else if ((nrhs > 1) || (nlhs != 1))
    {
        mexErrMsgTxt("Se requiere un argumento de entrada y uno de salida, ejemplo: r1=rope(numRope1)");
    }
    else
    {
        m1 = mxGetM(Y_IN);
        n1 = mxGetN(Y_IN);
        
        /* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
        if (!mxIsNumeric(Y_IN) || mxIsComplex(Y_IN) ||
                (max(m1,n1) != 1) || (min(m1,n1) != 1))
        {
            mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de cuerda");
        }
        else
        {
            /* Assign pointers to the various parameters */
            u = mxGetPr(Y_IN);
            if ((floor(u[0]) != ceil(u[0])) ||(u[0] > maxNumRopes) ||(u[0] < 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10 indicando el numero de cuerda");
            }
            else
            {
                /* Create a matrix for the return argument */
                YP_OUT = mxCreateNumericMatrix(m, n, mxDOUBLE_CLASS, mxREAL);
                /* Assign pointers to the various parameters */
                
                yp = mxGetPr(YP_OUT);
                
                CSharedStruct<RopePos>m_SharedData_Rope(M_COMPARTIDA_ROPE);
                
                m_SharedData_Rope.Acquire();
                
                int index = (int)u[0] - 1;
                /* Do the computations*/
                for(j=0; j < m; j++)
                {
                    yp[j + 0*m] = m_SharedData_Rope->rPoints[index][j].x;
                    yp[j + 1*m] = -m_SharedData_Rope->rPoints[index][j].z;
                    yp[j + 2*m] = m_SharedData_Rope->rPoints[index][j].y;
                }
                
                m_SharedData_Rope.Release();
            }
        }
    }
    return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "rope")/////
