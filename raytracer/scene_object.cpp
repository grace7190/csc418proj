/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	return false;
}

bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.
	// std::cout << "Origin: " << ray.origin << "  Dir: " << ray.dir << "\n";
	Point3D modelOrigin = worldToModel * ray.origin;
	Vector3D modelDir = worldToModel * ray.dir;

	Point3D center = Point3D(0,0,0);
	double A = modelDir.dot(modelDir);
	double B = (modelOrigin - center).dot(modelDir);
	double C = (modelOrigin - center).dot(modelOrigin - center) - 1;
	double D = B*B - A*C;
	// std::cout << "A: " << A << "  B: " << B << "  C: " << C << "  D: " << D << "\n";

	if (D < 0) {
		return false;
	} else {
		double lambda1 = (-2*B + pow(D, 0.5)) / 2*A;
		double lambda2 = (-2*B - pow(D, 0.5)) / 2*A;
		if (lambda1 < 0.0 && lambda2 < 0.0) {
			return false;
		} else {
			double minLambda = std::min(lambda1, lambda2);
			ray.intersection.point = modelToWorld * (modelOrigin + minLambda*modelDir);
			ray.intersection.normal = modelToWorld * (((modelOrigin + minLambda*modelDir) + (modelOrigin + minLambda*modelDir)) - center);
			ray.intersection.normal.normalize();
			ray.intersection.none = false;
			ray.intersection.t_value = minLambda;
			return true;
		}
	}
	
	return false;
}

