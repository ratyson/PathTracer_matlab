#include "mex.h"
#include <float.h>
#include <math.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	//mexPrintf("mex\n");
	// [intersect] = intersect_bb();
	double *ro, *rSign, *rInv_d, *bounds; //in 
	double *intersect;
	
	ro = (double *) mxGetPr(prhs[0]);
	rSign = (double *) mxGetPr(prhs[1]);
	rInv_d = (double *) mxGetPr(prhs[2]);
	bounds = (double *) mxGetPr(prhs[3]);
	
	plhs[0]=mxCreateDoubleMatrix(1,1,mxREAL);  //out
	intersect = (double *) mxGetPr(plhs[0]);
    
    intersect[0] = 0; // no intersect
	
	double tmin, tmax, tymin, tymax, tzmin, tzmax;
	
	//mexPrintf("boundsMin (%.3f,%.3f,%.3f)\n", bounds[0], bounds[1], bounds[2]);
	//mexPrintf("boundsMax (%.3f,%.3f,%.3f)\n", bounds[3], bounds[4], bounds[5]);
	//mexPrintf("rSign: (%.3f,%.3f,%.3f)\n", rSign[0], rSign[1],rSign[2]);
	
	tmin = (bounds[ (int)(rSign[0]*3) ] - ro[0]) * rInv_d[0];
    tmax = (bounds[(int)((1-rSign[0])*3)] - ro[0]) * rInv_d[0]; 
    tymin = (bounds[(int)((rSign[1]*3)+1)] - ro[1]) * rInv_d[1];
    tymax = (bounds[(int)((1-rSign[1])*3 + 1) ] - ro[1]) * rInv_d[1];
    
   // mexPrintf("tmin: %.3f, tmax: %.3f\n", tmin, tmax);
   // mexPrintf("tymin: %.3f, tymax: %.3f\n", tymin, tymax);
    
    if ( (tmin > tymax) || (tymin > tmax) ) return;    
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;
     
    tzmin = (bounds[(int)((rSign[2]*3)+2)] - ro[2]) * rInv_d[2];
    tzmax = (bounds[(int)((1-rSign[2])*3 + 2)] - ro[2]) * rInv_d[2];
     
    //mexPrintf("tzmin: %.3f, tzmax: %.3f\n", tzmin, tzmax); 
     
    if ( (tmin > tzmax) || (tzmin > tmax) ) return;
     
    intersect[0] = 1;
}



   // tmin = ( b.bounds(1,r.sign(1)+1) - r.o(1))*r.inv_d(1);
  //  tmax = ( b.bounds(1,2-r.sign(1)) - r.o(1))*r.inv_d(1);
  //  tymin = ( b.bounds(2,r.sign(2)+1) - r.o(2))*r.inv_d(2);
  //  tymax = ( b.bounds(2,2-r.sign(2)) - r.o(2))*r.inv_d(2);
    
 //%  tmin = (bounds[r.sign[0]].x() - r.origin.x()) * r.inv_direction.x();
 //%  tmax = (bounds[1-r.sign[0]].x() - r.origin.x()) * r.inv_direction.x(); 
 //%  tymin = (bounds[r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
 //%  tymax = (bounds[1-r.sign[1]].y() - r.origin.y()) * r.inv_direction.y();
 

     
 //    tzmin = ( b.bounds(3,r.sign(3)+1) - r.o(3))*r.inv_d(3);
 //    tzmax = ( b.bounds(3,2-r.sign(3)) - r.o(3))*r.inv_d(3);
    
 //   tzmin = (bounds[r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();
 //   tzmax = (bounds[1-r.sign[2]].z() - r.origin.z()) * r.inv_direction.z();