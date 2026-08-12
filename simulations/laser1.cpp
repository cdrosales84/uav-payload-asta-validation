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

////FUNCION LLAMADA POR MATLAB (en este caso "laser")/////
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    double	*u;
    uint16_T   *yp;
    const int	m = reg_ang1/res_ang1;
    const int   n = reg_ang2/res_ang2+1;
    unsigned int	m1,n1;
    int j, k;
    
    /* Check for proper number of arguments */
    if ((nrhs == 0) || (nrhs > 1) || (nlhs > 1))
    {
        mexErrMsgTxt("Se requiere un argumento de entrada y uno de salida, ejemplo: laserData=laser(numHeli)");
        return;
    }
    else
    {
        m1 = mxGetM(Y_IN);
        n1 = mxGetN(Y_IN);
        
        /* Check the dimensions of Y1.  Y1 must be 1 X 1.*/
        if (!mxIsNumeric(Y_IN) || mxIsComplex(Y_IN) ||
                (max(m1,n1) != 1) || (min(m1,n1) != 1))
        {
            mexErrMsgTxt("Se requiere que u1 sea un escalar indicando el numero de Heli");
        }
        else
        {
            /* Assign pointers to the various parameters */
            u = mxGetPr(Y_IN);
            if ((floor(u[0]) != ceil(u[0])) ||(u[0] > maxNumQuads) ||(u[0] < 1))
            {
                mexErrMsgTxt("Se requiere que u1 sea un entero (entre 1 y 10 indicando el numero de Heli");
            }
            else
            {
                /* Create a matrix for the return argument */
                YP_OUT = mxCreateNumericMatrix(m, n, mxUINT16_CLASS, mxREAL);
                /* Assign pointers to the various parameters */
                
                yp = (uint16_T *)mxGetData(YP_OUT);
                
                CSharedStruct<LaserData>m_SharedData_Laser(M_COMPARTIDA_LASER);
                
                m_SharedData_Laser.Acquire();
                
                int index = (int)u[0] - 1;
                /* Do the computations*/
                for(j=0; j < m; j++)
                {
                    for(k=0; k < n; k++)
                    {
                        
                        yp[j + k*m] = (uint16_T)(m_SharedData_Laser->laserData[index][j][k]*100.0f);
                    }
                }
                m_SharedData_Laser.Release();
            }
        }
    }
    return;
}
////FUNCION LLAMADA POR MATLAB (en este caso "laser")/////
