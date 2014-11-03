#include "mex.h"
#include <float.h>
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	//mexPrintf("mex\n");
	// [faceI, t] = object_intersect(r.o,r.ud,r.l, p.nFaces, p.faceArr);
	double *ro, *rd, *rl, *nFaces, *faceArr; //in 
	double *intersect;
	
	ro = (double *) mxGetPr(prhs[0]);
	rd = (double *) mxGetPr(prhs[1]);
	rl = (double *) mxGetPr(prhs[2]);
	nFaces = (double *) mxGetPr(prhs[3]);
	faceArr = (double *) mxGetPr(prhs[4]);
	
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);  //out
	intersect = (double *) mxGetPr(plhs[0]);
	
	// Möller-Trumbore algorithm
    // t, distance from ray origin of intersect
    // u,v are Barycentric Coordinates
    
    double det, invDet, u, v, t_current;
    double *v1, *v2, *v3;
    double edge1[3];
    double edge2[3];
    double pvec[3];
    double tvec[3];
    double qvec[3];
    int faceI_current;
    
    intersect[0] = 0; // no intersect
	
	for( int f=0; f < nFaces[0]; f++){
		v1 = &faceArr[ f*9 ]; //face array 1D, 3 verts per face, x,y,z ==9 doubles
		v2 = &v1[3];	// pointers to verts
		v3 = &v2[3]; 
		
		edge1[0] = v2[0] - v1[0];
		edge1[1] = v2[1] - v1[1];
		edge1[2] = v2[2] - v1[2];
		edge2[0] = v3[0] - v1[0];
		edge2[1] = v3[1] - v1[1];
		edge2[2] = v3[2] - v1[2];
		
		pvec[0] = rd[1]*edge2[2] - rd[2]*edge2[1]; // cross product ray direction
		pvec[1] = rd[2]*edge2[0] - rd[0]*edge2[2];
		pvec[2] = rd[0]*edge2[1] - rd[1]*edge2[0];
		
		det = edge1[0]*pvec[0] + edge1[1]*pvec[1] + edge1[2]*pvec[2]; //dot prod
		if(det==0) continue; 
		
		invDet = 1/det;
		tvec[0] = ro[0]-v1[0];
		tvec[1] = ro[1]-v1[1];
		tvec[2] = ro[2]-v1[2];
		
		u = (tvec[0]*pvec[0] + tvec[1]*pvec[1] + tvec[2]*pvec[2])*invDet; //dot prod
		if( u < 0 || u > 1) continue;
		
		qvec[0] = tvec[1]*edge1[2] - tvec[2]*edge1[1]; // cross product
		qvec[1] = tvec[2]*edge1[0] - tvec[0]*edge1[2];
		qvec[2] = tvec[0]*edge1[1] - tvec[1]*edge1[0];
		
		v = (rd[0]*qvec[0] + rd[1]*qvec[1] + rd[2]*qvec[2])*invDet; //dot prod
		if( v < 0 || (u+v) > 1) continue;
		
		t_current = (edge2[0]*qvec[0] + edge2[1]*qvec[1] + edge2[2]*qvec[2])*invDet; //dot prod
		if( t_current > 0.0001 &&  t_current < rl[0]) {
			intersect[0] = 1;
			break;
		}
	}
}