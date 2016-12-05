/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		classes defining primitives in the scene

***********************************************************/

#include "util.h"
#include <vector>

// All primitives should provide a intersection function.  
// To create more primitives, inherit from SceneObject.
// Namely, you can create, Sphere, Cylinder, etc... classes
// here.
class SceneObject {
public:
	// Returns true if an intersection occured, false otherwise.
	virtual bool intersect( Ray3D&, const Matrix4x4&, const Matrix4x4& ) = 0;
};

// Example primitive you can create, this is a unit square on 
// the xy-plane.
class UnitSquare : public SceneObject {
public:
	bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
			const Matrix4x4& modelToWorld );
};

class UnitSphere : public SceneObject {
public:
	bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
			const Matrix4x4& modelToWorld );
};

class Triangle : public SceneObject {
public: 
    // Triangle is defined by 3 Point3D's
    Triangle(Point3D A, Point3D B, Point3D C, Vector3D normal = Vector3D()) : _vtA(A), _vtB(B), _vtC(C), _normal(normal) {};
    
    bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
			const Matrix4x4& modelToWorld );
    Vector3D _normal;
    Point3D _vtA;
    Point3D _vtB;
    Point3D _vtC;
private:

};

class TriangleMesh : public SceneObject {
public:
    TriangleMesh(const char *file_name);
    bool intersect( Ray3D& ray, const Matrix4x4& worldToModel,
			const Matrix4x4& modelToWorld );
private:
    std::vector<Triangle *> _triangles;
    const char *_file_name;
    void loadMeshFromFile();
};