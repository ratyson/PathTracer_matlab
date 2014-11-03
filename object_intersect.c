#include "mex.h"
#include <float.h>
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	// [faceI, t] = object_intersect(r.o,r.ud,r.l, p.bb.bounds, p.nFaces, p.faceArr, p.nArr);
	double *ro, *rd, *rl, *bbBounds, *nFaces, *faceArr, *nArr; //in 
	double *faceI, *t;
	
	ro = (double *) mxGetPr(prhs[0]);
	rd = (double *) mxGetPr(prhs[1]);
	rl = (double *) mxGetPr(prhs[2]);
	bbBounds = (double *) mxGetPr(prhs[3]);
	nFaces = (double *) mxGetPr(prhs[4]);
	faceArr = (double *) mxGetPr(prhs[5]);
	nArr = (double *) mxGetPr(prhs[6]);
	
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);  //out
	faceI = (double *) mxGetPr(plhs[0]);
	plhs[1]=mxCreateDoubleMatrix(1,1,mxREAL);
	t = (double *) mxGetPr(plhs[1]);
	
   
}