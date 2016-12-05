/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

Colour PointLight::shade( Ray3D& ray ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.  
   
    double Ka = 0.2;
    double Kd = 0.7;
    double Ks = 0.8; 
    
    Intersection in = ray.intersection;
    Point3D p = in.point; //point on surface
    Vector3D s = _pos - p; //light vector
    Material *ma = in.mat;
    s.normalize(); 
    double lam = std::max((in.normal).dot(s),0.0);
    s.normalize();
    double spec = 0.0;
    // int shading = 1; 
    if (lam > 0.0) {
        // if (shading == 0){ //Phong
        //     Vector3D R = 2.0*(s.dot(in.normal)*in.normal) - s;
        //     R.normalize();
        //     Vector3D V = -ray.dir;
        //     spec = pow(std::max(R.dot(V), 0.0), ma->specular_exp);
        // } else {

        //Blinn-Phong
        Vector3D halfDir = (s + -ray.dir);
        halfDir.normalize();
        double specAngle = std::max(halfDir.dot(in.normal), 0.0);
        spec = pow(specAngle, ma->specular_exp);
        // }
    }
    Colour co = (Ka*_col_ambient + Kd*lam*ma->diffuse + Ks*spec*ma->specular);
    co.clamp();
    return co;
    
}

