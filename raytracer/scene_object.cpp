/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "scene_object.h"
#include <stdio.h>
#include <string.h>

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

bool Triangle::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	Point3D modelOrigin = worldToModel * ray.origin;
	Vector3D modelDir = worldToModel * ray.dir;
    Vector3D modelOriginVector = Vector3D(modelOrigin[0], modelOrigin[1], modelOrigin[2]);
    Point3D center = Point3D(0,0,0); //the true origin, not to be confused with modelOrigin...
    
    Vector3D AB = _vtB - _vtA;
    Vector3D AC = _vtC - _vtA;
    Vector3D BC = _vtC - _vtB;
    Vector3D CA = _vtA - _vtC;
    //if no input for _normal, calculate from ray

    _normal = AB.cross(AC);
    _normal.normalize(); 
    double D = _normal.dot(Vector3D(_vtA[0],_vtA[1],_vtA[2]));
    double t = -(_normal.dot(modelOriginVector) + D) / _normal.dot(modelDir);
    Point3D poi = modelOrigin + t*modelDir; //point of intersection (modelIntersection)

    //handle case where ray parallel to triangle
    if (modelDir.dot(_normal) == 0){
        return false;
    }
    
    //handle case where triangle is behind ray
    if (t < 0){
        return false;
    }
    
    if (ray.intersection.none || t < ray.intersection.t_value) {
        //test if ray is inside triangle
        Vector3D poiVec = Vector3D(poi[0],poi[1],poi[2]);
        Vector3D c0 = AB.cross(poiVec-(Vector3D(_vtA[0],_vtA[1],_vtA[2])));
        Vector3D c1 = BC.cross(poiVec-(Vector3D(_vtB[0],_vtB[1],_vtB[2])));
        Vector3D c2 = CA.cross(poiVec-(Vector3D(_vtC[0],_vtC[1],_vtC[2])));
        
        if (_normal.dot(c0)>0 && _normal.dot(c1)>0 && _normal.dot(c2)>0){
            ray.intersection.point = modelToWorld * poi;
            ray.intersection.normal = transNorm(worldToModel, _normal);
            ray.intersection.normal.normalize();
            ray.intersection.none = false;
            ray.intersection.t_value = t;
            return true;
        }
    }
    return false;
    
}


TriangleMesh::TriangleMesh(const char *file_name) : _file_name(file_name) {
    
    loadMeshFromFile();
    
}

void TriangleMesh::loadMeshFromFile(){
    //tutorial used: http://www.opengl-tutorial.org/beginners-tutorials/tutorial-7-model-loading/
    
    // temporary variables
    std::vector<int> vertexIndices, uvIndices, normalIndices;
    std::vector<Point3D> temp_vertices;
    std::vector<Point3D> temp_uvs;
    std::vector<Vector3D> temp_normals;

    std::cout<<"loading mesh: " << _file_name << "\n";
    
    FILE *file = fopen(_file_name, "r");
    if (file==NULL){
        std::cout << "Objection! Bad mesh!\n";
        return; 
    }
    
    while (1){
        
        char lineHeader [128]; // assumes first 'word' of line < 128 chars
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF){ //scan until end of file
            break;
        }
        //parse the file
        if (strcmp(lineHeader, "v") == 0){
            double x;
            double y;
            double z;
            int success = fscanf(file, "%lf %lf %lf\n", &x, &y, &z );
            temp_vertices.push_back(Point3D(x,y,z));
            
        } else if (strcmp(lineHeader, "vt") == 0){
            double u;
            double v;
            int success = fscanf(file, "%lf %lf\n", &u, &v);
            temp_uvs.push_back(Point3D(u,v,0.0));
            
        } else if (strcmp( lineHeader, "vn") == 0 ){
            double nx;
            double ny;
            double nz;
            int success = fscanf(file, "%lf %lf %lf\n", &nx, &ny, &nz );
            temp_normals.push_back(Vector3D(nx,ny,nz));
            
        } else if ( strcmp( lineHeader, "f" ) == 0 ){
            std::string vertex1, vertex2, vertex3;
            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
            
            if (matches != 9){
                std::cout << ("f part of obj unreadable. QQ more.\n");
                return;
            }
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices    .push_back(uvIndex[0]);
            uvIndices    .push_back(uvIndex[1]);
            uvIndices    .push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);
        }
    }

    for(int i=0; i<vertexIndices.size(); i+=3){
        int vtIndex = vertexIndices[i];
        Point3D v1 = temp_vertices[vtIndex-1];
        Point3D v2 = temp_vertices[vtIndex+1-1];
        Point3D v3 = temp_vertices[vtIndex+2-1];
        // TODO: txtr... ignore texture for now ;-;
        int nIndex = normalIndices[i];
        Vector3D normal = temp_normals[nIndex-1];
        
        Triangle *tri = new Triangle(v1, v2, v3, normal);
        _triangles.push_back(tri); 
    }

    std::cout << "polycount: " << _triangles.size() << "\n";
    
}

bool TriangleMesh::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
    //Ray3D rayray = Ray3D(ray.origin, ray.dir);
    
    for(int i=0; i<_triangles.size(); i++){
        Triangle *tri = _triangles[i];
        //std::cout << " " << tri->_vtA << tri->_vtB << tri->_vtC;
        //maybe make a temporary ray
        // do the other stuff if there's an intersection...? 
        tri->intersect(ray, worldToModel, modelToWorld);
    }
    return !ray.intersection.none;
    //return false;
}


