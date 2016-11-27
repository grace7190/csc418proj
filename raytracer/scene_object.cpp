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
    
	Point3D modelOrigin = worldToModel * ray.origin;
	Vector3D modelDir = worldToModel * ray.dir;

	Vector3D normal = Vector3D(0,0,1);
	Point3D center = Point3D(0,0,0);
	if (modelDir.dot(normal) == 0) {
		return false;
	} else {
		double lambda = (center - modelOrigin).dot(normal) / modelDir.dot(normal);
		if (lambda < 0.0) {
			return false;
		}
		Point3D intersect = modelOrigin + lambda*modelDir;
		if (intersect[0] <= 0.5	&& intersect[0] >= -0.5
			&& intersect[1] <= 0.5 && intersect[1] >= -0.5) {
			if (ray.intersection.none || lambda < ray.intersection.t_value) { 
				ray.intersection.point = modelToWorld*intersect;
				ray.intersection.normal = transNorm(worldToModel, normal);
				ray.intersection.normal.normalize();
				ray.intersection.none = false;
				ray.intersection.t_value = lambda;
                return true;
			}
			return false;
		}
	}

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

	if (D < 0) {
		return false;
	} else {
		double lambda1 = (-B + pow(D, 0.5)) / A;
		double lambda2 = (-B - pow(D, 0.5)) / A;
		if (lambda1 < 0.0 && lambda2 < 0.0) {
			return false;
		} else {
			double minLambda; // Closest non-negative intersection
			if (lambda1 > 0.0 && lambda2 > 0.0) {
				minLambda = std::min(lambda1, lambda2);
			} else {
				minLambda = std::max(lambda1,lambda2);	
			}
			if (ray.intersection.none || minLambda < ray.intersection.t_value) {
				Point3D modelIntersection = modelOrigin + minLambda*modelDir;
                ray.intersection.point = modelToWorld * modelIntersection;
                ray.intersection.normal = transNorm(worldToModel, (modelIntersection - center));
                ray.intersection.normal.normalize();
                ray.intersection.none = false;
                ray.intersection.t_value = minLambda;
                return true;
			}
			return false;
		}
	}
	
	return false;
}

