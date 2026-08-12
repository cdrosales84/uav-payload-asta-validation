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

////FUNCION LLAMADA POR MATLAB (en este caso "VirtualLoad")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double	*u;
    unsigned int	m,n;
  		
	/* Check for proper number of arguments */
    if ((nrhs != 1) || (nlhs != 0))
    {
        mexErrMsgTxt("Se requiere un argumento de entrada y ninguno de salida, ejemplo: load([vLx vLy vLz vLp])");
        return;
    }
    else
    {
        m = mxGetM(Y_IN);
        n = mxGetN(Y_IN);
		/* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
        if (!mxIsNumeric(Y_IN) || mxIsComplex(Y_IN) ||
                (max(m,n) != 4) || (min(m,n) != 1))
        {
            mexErrMsgTxt("Se requiere que u sea un vector de 1 x 4 (o 4 x 1) para indicar posición y orientación de la carga virtual [x y z psi]");
        }
        else
        {
            /* Assign pointers to the various parameters */
            u = mxGetPr(Y_IN);
            
            CSharedStruct<VirtualLoad>m_SharedData_Load(M_COMPARTIDA_LOAD);
            
            m_SharedData_Load.Acquire();
            
            m_SharedData_Load->position.x = u[0];
            m_SharedData_Load->position.y = u[1];
            m_SharedData_Load->position.z = u[2];
            m_SharedData_Load->orientation = u[3];
            
            m_SharedData_Load.Release();
        }
    }
    return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "VirtualLoad")/////
