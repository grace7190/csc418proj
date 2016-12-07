/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

void Raytracer::DOFSampling( Point3D& eye, Vector3D& view, Point3D focus ) {
	double x_change = (rand() % 20 - 10) /40.0;
	double y_change = (rand() % 20 - 10) /40.0;
	double z_change = (rand() % 20 - 10) /40.0;

	Matrix4x4 translation;
	translation[0][3] = x_change;
	translation[1][3] = y_change;
	translation[2][3] = z_change;
	eye = translation*eye;
	view = focus - eye;
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}
void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
    traverseScene(node,ray,_modelToWorld,_worldToModel);
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray, const Matrix4x4& modelToWorld, const Matrix4x4& worldToModel ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	Matrix4x4 myModelToWorld = modelToWorld*node->trans;
	Matrix4x4 myWorldToModel = node->invtrans*worldToModel;
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, myWorldToModel, myModelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray, myModelToWorld,myWorldToModel);
		childPtr = childPtr->next;
	}

}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	int numLights = 0;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.

		Point3D p = ray.intersection.point; // point on surface

		int i;
		for (i = 0; i < 20; i++) {
            double x_change = (rand() % 20 - 10)/5.0;
            double y_change = (rand() % 20 - 10)/5.0;
            double z_change = (rand() % 20 - 10)/5.0;

			Point3D offset = Point3D(
				curLight->light->get_position()[0]+x_change/10.0,
				curLight->light->get_position()[1]+y_change/10.0,
				curLight->light->get_position()[2]+z_change/10.0);

			// Check for intersection between point and light (shadow if none)
			Vector3D s = offset - p; // vector towards light
			s.normalize();
			p = p + 0.0001*s;
			Ray3D light_ray = Ray3D(p, s);
			traverseScene(_root, light_ray);
			if (light_ray.intersection.none) {
				ray.col = ray.col + 0.05*curLight->light->shade(ray);
			} else {
        		ray.col = ray.col + 0.05*curLight->light->shadow();
        	}
        }
		curLight = curLight->next;
		numLights++;
	}
	//ray.col = (1.0/numLights)*ray.col;
	ray.col.clamp();
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}

Colour Raytracer::shadeRay( Ray3D& ray, int max_depth) {
    if (max_depth <= 0) {
        return Colour(0.0, 0.0, 0.0); 
    }
	Colour col(0.0, 0.0, 0.0); 
	traverseScene(_root, ray); 
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray);
		if (ray.intersection.mat->refraction > 0) {
			col = ray.intersection.mat->alpha*ray.col;
		} else {
			col = ray.col;
		}
        
	// You'll want to call shadeRay recursively (with a different ray, 
	// of course) here to implement reflection/refraction effects. 
		if (ray.intersection.mat->reflection > 0) {
			Vector3D reflectV = -2*(ray.intersection.normal.dot(ray.dir))*ray.intersection.normal + ray.dir;
			reflectV.normalize();
			Ray3D reflectedRay = Ray3D(ray.intersection.point+0.001*reflectV, reflectV);
			col = col + ray.intersection.mat->reflection*shadeRay(reflectedRay, max_depth-1);
			col.clamp();
		}
		// REFRACTION
		if (ray.intersection.mat->refraction > 0) {
			Vector3D N = ray.intersection.normal;
			double NdotRay = N.dot(ray.dir);
			double refIdx = ray.intersection.mat->refraction;
			if (NdotRay < 0) {
				// outside surface, want positive cos(theta)
				NdotRay = -NdotRay;
			} else {
				// inside surface, reverse normal, invert refraction index
				N = -N;
				refIdx = 1.0/ray.intersection.mat->refraction;
			}
			double k = 1 - refIdx * refIdx * (1 - NdotRay*NdotRay);
			Vector3D refractV = refIdx * ray.dir + (refIdx * NdotRay - sqrtf(k)) * N;
			refractV.normalize();
			Ray3D refractedRay = Ray3D(ray.intersection.point+0.001*refractV, refractV);
			col = col + shadeRay(refractedRay, max_depth-1);
			col.clamp();
		}
	}

	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	double i_off[] = {0.0, 0.2, 0.15, -0.31, 0.42, 0.34, 0.22, -0.11, 0.05};
	double j_off[] = {0.0, -0.2, 0.3, -0.3, -0.4, -0.12, 0.41, -0.1, 0.15};
    // double x_change = rand() % 4 - 2;
    // double y_change = rand() % 4 - 2;
    
	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			// Sets up ray origin and direction in view space, 
			// image plane is at z = -1.
			Point3D origin(0, 0, 0);
			Colour averageColour = Colour(0,0,0);

			for (int offset = 0; offset < sizeof(i_off)/sizeof(i_off[0]); offset++){
                double i_change = (rand() % 100 - 50)/100.0;
                double j_change = (rand() % 100 - 50)/100.0;
				Point3D imagePlane;
				imagePlane[0] = (-double(width)/2 + 0.5 + j + j_off[offset])/factor;
				imagePlane[1] = (-double(height)/2 + 0.5 + i + i_off[offset])/factor;
				imagePlane[2] = -1;

				Ray3D ray;
				ray.origin = viewToWorld*imagePlane;
				ray.dir = ray.origin - viewToWorld*origin;
				ray.dir.normalize();

				averageColour = averageColour + shadeRay(ray, 3);
			}

			averageColour = (1.0/9.0) * averageColour;
			averageColour.clamp();

			_rbuffer[i*width+j] = int(averageColour[0]*255);
			_gbuffer[i*width+j] = int(averageColour[1]*255);
			_bbuffer[i*width+j] = int(averageColour[2]*255);
		}
	}

	flushPixelBuffer(fileName);
}

void Raytracer::averageImage(unsigned char* rbuffer2, unsigned char* gbuffer2, unsigned char* bbuffer2,
	Point3D focal, int width, int height, Vector3D up, double fov){

	for (int i = 0; i<24; i++){
		std::cout << i << "\n";
        Point3D eye(0, 0, 1);
        Vector3D view(0.3, 0, -1);
		DOFSampling(eye, view, focal);
        char filename[] = " ";
		render(width, height, eye, view, up, fov, filename);
		for (int j=0; j<( height*width-1 ); j++){
			rbuffer2[j] += _rbuffer[j]/24.0;
			gbuffer2[j] += _gbuffer[j]/24.0;
			bbuffer2[j] += _bbuffer[j]/24.0;
		}
	}
	for (int j=0; j<( height*width-1 ); j++){
		_rbuffer[j] = int(rbuffer2[j]);
		_gbuffer[j] = int(gbuffer2[j]);
		_bbuffer[j] = int(bbuffer2[j]);
	}
    char imfilename[] = "avView2.bmp";
	flushPixelBuffer(imfilename);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 380; 
	int height = 240;

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	Point3D eye(0, 0, 1);
	Vector3D view(0.3, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 90;

	// Defines a material for shading.
	Material gold( Colour(0.3, 0.3, 0.3), Colour(0.7516, 0.60648, 0.22648), 
			Colour(0.628281, 0.555802, 0.366065), 
			51.2, 0.0, 1.3, 0.3 );
	Material jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63), 
			Colour(0.316228, 0.316228, 0.316228), 
			12.8, 1.0, 0.0, 1.0 );
            
	Material ultrav( Colour(0, 0, 0), Colour(59/255.0, 23/255.0, 99/255.0), 
			Colour(224/255.0, 118/255.0, 255/255.0), 
			12.8, 0.0, 0.0, 1.0);
            
    Material stark( Colour(0, 0, 0), Colour(31/255.0, 29/255.0, 82/255.0), 
			Colour(96/255.0, 208/255.0, 255/255.0), 
			12.8, 0.0, 1.3, 0.3);
            
	Material bade( Colour(0, 0, 0), Colour(0.24, 0.14, 0.73), 
		Colour(0.206228, 0.506228, 0.306228), 
		19.8, 0.0, 0.0, 1.0 );
    
    Material pantone(Colour(27/255.0,34/255.0,46/255.0),
                    Colour(145/255.0,167/255.0,208/255.0),
                    Colour(246/255.0,202/255.0,204/255.0),
                    160.0, 0.02, 1.35, 0.5); 
                    //specexp reflection refraction alpha
                    
    Material flat_pantone(Colour(27/255.0,34/255.0,46/255.0),
                    Colour(145/255.0,167/255.0,208/255.0),
                    Colour(246/255.0,202/255.0,204/255.0),
                    0.001, 0.0, 0.0, 1.0); 
                    
    Material gemini(Colour(23/255.0,24/255.0,32/255.0),
                    Colour(51/255.0,53/255.0,67/255.0),
                    Colour(116/255.0,202/255.0,212/255.0),
                    160.0, 0.00, 0.0, 1.0); 
                    
    Material high(Colour(23/255.0,24/255.0,32/255.0),
                    Colour(192/255.0,237/255.0,221/255.0),
                    Colour(192/255.0,237/255.0,221/255.0),
                    160.0, 0.7, 0.0, 1.0); 
        
	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(-2, -2.2, 0), 
				Colour(0.6, 0.6, 0.9) ) );
    raytracer.addLightSource( new PointLight(Point3D(1.2, 2.1, -1), 
	 			Colour(0.99, 0.92, 0.9) ) );

	// Add a unit square into the scene with material mat.
	SceneDagNode* sphere = raytracer.addObject( new UnitSphere(), &gold );
	// SceneDagNode* sphere2 = raytracer.addObject( new UnitSphere(), &pantone );
    // SceneDagNode* sphere3 = raytracer.addObject( new UnitSphere(), &pantone );
    // SceneDagNode* sphere4 = raytracer.addObject( new UnitSphere(), &pantone );
	SceneDagNode* plane = raytracer.addObject( new UnitSquare(), &jade );
    
	// Apply some transformations to the unit square.
	double factor1[3] = { 1.5, 1.5, 1.5 };
	double factor2[3] = { 300.0, 300.0, 300.0 };
	double factor3[3] = { 0.3, 0.3, 0.3 };

	//double factor3[3] = { 40.0, 40.0, 40.0 };
    
	raytracer.translate(sphere, Vector3D(1.0, 3.5, -3.0));	
	raytracer.scale(sphere, Point3D(0, 0, 0), factor1);

	// raytracer.translate(sphere2, Vector3D(2, 1.5, -3.4));	
	// raytracer.scale(sphere2, Point3D(0, 0, 0), factor3);	
    
	// raytracer.translate(sphere3, Vector3D(-2, 0.5, -3));	
	// raytracer.scale(sphere3, Point3D(0, 0, 0), factor3);
    
	// raytracer.translate(sphere4, Vector3D(1, -2.2, -3));	
	// raytracer.scale(sphere4, Point3D(0, 0, 0), factor3);

	raytracer.translate(plane, Vector3D(0, 0, -6.5));	
    raytracer.rotate(plane, 'y', 21);
    raytracer.rotate(plane, 'x', -45);
	raytracer.scale(plane, Point3D(0, 0, 0), factor2);
    
    
    // Point3D p1 = Point3D(2.0,1.0,-5.0);
    // Point3D p2 = Point3D(2.0,3.0,-5.0);
    // Point3D p3 = Point3D(-0.0,1.0,-5.0);
    
    // Point3D p1 = Point3D(1.0,1.0,0.0);
    // Point3D p2 = Point3D(1.0,-1.0,0.0);
    // Point3D p3 = Point3D(0.0,0.0,3.0);
    
    // SceneDagNode* triangle = raytracer.addObject( new Triangle(p1, p2, p3), &ultrav );
	// raytracer.translate(triangle, Vector3D(0, 0, -4.5));
    
    // p1 = Point3D(1.0,-1.0,0.0);
    // p2 = Point3D(-1.0,-1.0,0.0);
    // p3 = Point3D(0.0,0.0,3.0);
    
    // SceneDagNode* triangle2 = raytracer.addObject( new Triangle(p1, p2, p3), &ultrav );
	// raytracer.translate(triangle2, Vector3D(0, 0, -4.5));
    
    // double breaded[3] = { 3.0, 3.0, 3.0 };
    // TriangleMesh* tMesh = new TriangleMesh("Bread.obj");
    // SceneDagNode* cube = raytracer.addObject(tMesh, &ultrav);
    // raytracer.translate(cube, Vector3D(-0.0, -0.0, -3.5));
    // raytracer.rotate(cube, 'y', -45);
    // raytracer.rotate(cube, 'x', 15);
    // raytracer.scale(cube, Point3D(0, 0, 0), breaded);
    
    //double deerFactor[3] = { 0.05, 0.05, 0.05 };
    // TriangleMesh* deerMesh = new TriangleMesh("wurf.obj");
    // SceneDagNode* der = raytracer.addObject(deerMesh, &high);
    // raytracer.translate(der, Vector3D(2.0, 1.0, -3.5));
    // raytracer.rotate(der, 'x', 45);
    // raytracer.scale(der, Point3D(0, 0, 0), factor1);
    
	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	// raytracer.render(width, height, eye, view, up, fov, "view1.bmp");
	
	// Render it from a different point of view.
	Point3D eye2(4, 2, -0.2);
	Vector3D view2(-4, -1.8, -5.2);
	Point3D focal (0, 0, -3);

	unsigned char* rbuffer2;
	unsigned char* gbuffer2;
	unsigned char* bbuffer2;
	int numbytes = width * height * sizeof(unsigned char);
	rbuffer2 = new unsigned char[numbytes];
	gbuffer2 = new unsigned char[numbytes];
	bbuffer2 = new unsigned char[numbytes];
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			rbuffer2[i*width+j] = 0;
			gbuffer2[i*width+j] = 0;
			bbuffer2[i*width+j] = 0;
		}
	}

	//raytracer.averageImage(rbuffer2, gbuffer2, bbuffer2, focal, width, height, up, fov);
    
    char file2[] = "view2.bmp";
	raytracer.render(width, height, eye2, view2, up, fov, file2);
	//char file1[] = "view1.bmp";
    // raytracer.render(width, height, eye, view, up, fov, file1);

	return 0;
}

// TODO:
// 	. under-reflection??
// 	. fix shadows (only ambient instead of black)
//  . fix shadows for see-thru objects!!
// ----------------------	
// 	. ray marching..??
// 	. particle effects!
//  . bounding box??
