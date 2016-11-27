/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements light_source.h

***********************************************************/

#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray ) {
	// TODO: implement this function to fill in values for ray.col 
	// using phong shading.  Make sure your vectors are normalized, and
	// clamp colour values to 1.0.
	//
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.  
    
    /////////////////////////////////////////////////////////////
    //based on this code right hurr vvv
  // Lambert's cosine law
  // float lambertian = max(dot(N, L), 0.0);
  // float specular = 0.0;
  // if(lambertian > 0.0) {
    // vec3 R = reflect(-L, N);      // Reflected light vector
    // vec3 V = normalize(-vertPos); // Vector to viewer
   // Compute the specular term
    // float specAngle = max(dot(R, V), 0.0);
    // specular = pow(specAngle, shininessVal);
  // }
  // gl_FragColor = vec4(Ka * ambientColor +
                      // Kd * lambertian * diffuseColor +
                      // Ks * specular * specularColor, 1.0);
   //////////////////////////////////////////////////////////////
   
    double Ka = 0.1;
    double Kd = 0.2;
    double Ks = 0.1; 
    
    Intersection in = ray.intersection;
    Point3D p = in.point; //point on surface
    Vector3D s = _pos - p; //light vector
    Material *ma = in.mat;
    s.normalize(); 
    double lam = std::max((in.normal).dot(s),0.0);
    s.normalize();
    double spec = 0.0;
    int shading = 1; 
    if (lam > 0.0) {
        if (shading == 0){ //Phong
            Vector3D R = 2.0*(s.dot(in.normal)*in.normal) - s;
            R.normalize();
            Vector3D V = -ray.dir;
            spec = pow(std::max(R.dot(V), 0.0), ma->specular_exp);
        } else { //Blinn-Phong
            Vector3D halfDir = (s + -ray.dir);
            halfDir.normalize();
            double specAngle = std::max(halfDir.dot(in.normal), 0.0);
            spec = pow(specAngle, ma->specular_exp);
        }
    }
    Colour co = (Ka*_col_ambient + Kd*lam*ma->diffuse + Ks*spec*ma->specular);
    co.clamp();
    ray.col = co;
    
}

