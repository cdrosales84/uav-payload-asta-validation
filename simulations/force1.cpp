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

////FUNCION LLAMADA POR MATLAB (en este caso "force")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double	*u;
	double   *yp;
    unsigned int	m,n;
  		
	/* Check for proper number of arguments */
    if ((nrhs == 0) || (nrhs > 1) || (nlhs > 1))
    {
        mexErrMsgTxt("Se requiere un argumento de entrada y uno de salida, ejemplo: f1=force(numRope1)");
        return;
    }
    else
    {
        m = mxGetM(Y_IN);
        n = mxGetN(Y_IN);
		/* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
        if (!mxIsNumeric(Y_IN) || mxIsComplex(Y_IN) ||
                (max(m,n) != 1) || (min(m,n) != 1))
        {
            mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de cuerda");
        }
        else
        {
            /* Assign pointers to the various parameters */
            u = mxGetPr(Y_IN);
            if ((floor(u[0]) != ceil(u[0])) ||(u[0] > 10) ||(u[0] < 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10) indicando el numero de cuerda");
            }
            else
            {
                /* Create a matrix for the return argument */
                    
                YP_OUT = mxCreateDoubleMatrix(3, 1, mxREAL);
                    
                /* Assign pointers to the various parameters */
                    
                yp = mxGetPr(YP_OUT);
	    
                CSharedStruct<RopePos>m_SharedData_Rope(M_COMPARTIDA_ROPE);
 
                m_SharedData_Rope.Acquire();
                
                int index = (int)u[0] - 1;
                /* Do the computations*/
                yp[0] = m_SharedData_Rope->forceRope[index].x;
                yp[1] = - m_SharedData_Rope->forceRope[index].z;
                yp[2] = m_SharedData_Rope->forceRope[index].y;

                m_SharedData_Rope.Release();
            }
		} 
	}
	return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "force")/////
