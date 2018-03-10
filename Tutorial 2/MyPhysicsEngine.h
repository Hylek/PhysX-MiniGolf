#pragma once

#include "BasicActors.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	//pyramid vertices
	static PxVec3 pyramid_verts[] = {PxVec3(0,1,0), PxVec3(1,0,0), PxVec3(-1,0,0), PxVec3(0,0,1), PxVec3(0,0,-1)};
	//pyramid triangles: a list of three vertices for each triangle e.g. the first triangle consists of vertices 1, 4 and 0
	//vertices have to be specified in a counter-clockwise order to assure the correct shading in rendering
	static PxU32 pyramid_trigs[] = {1, 4, 0, 3, 1, 0, 2, 3, 0, 4, 2, 0, 3, 2, 1, 2, 4, 1};

	class Pyramid : public ConvexMesh
	{
	public:
		Pyramid(PxTransform pose=PxTransform(PxIdentity), PxReal density=1.f) :
			ConvexMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), pose, density)
		{
		}
	};

	class PyramidStatic : public TriangleMesh
	{
	public:
		PyramidStatic(PxTransform pose=PxTransform(PxIdentity)) :
			TriangleMesh(vector<PxVec3>(begin(pyramid_verts),end(pyramid_verts)), vector<PxU32>(begin(pyramid_trigs),end(pyramid_trigs)), pose)
		{
		}
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Box* box;
		Box* box2;
		Box* box3;
		Sphere* ball;
		PxMaterial* box1Mat;
		PxMaterial* box2Mat;
		PxMaterial* box3Mat;
		PxPhysics* physics = GetPhysics();



	public:
		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			//GetMaterial()->setDynamicFriction(.2f);
			//box1Mat->setRestitution(.5f);
			//box2Mat->setRestitution(1.f);
			//box3Mat->setRestitution(.25f);

			plane = new Plane();
			plane->Color(PxVec3(50.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);

			//box = new Box(PxTransform(PxVec3(.0f, 10.f, .0f)));
			//box->Color(PxVec3(255.f / 255.f, 132.f / 255.f, 13.f / 255.f));
			//box->Material(box1Mat);
			//Add(box);

			//box2 = new Box(PxTransform(PxVec3(2.f, 10.f, .0f)));
			//box2->Color(PxVec3(232.f / 255.f, 12.f / 255.f, 198.f / 255.f));
			//box->Material(box2Mat);
			//Add(box2);

			//box3 = new Box(PxTransform(PxVec3(-2.f, 10.f, .0f)));
			//box3->Color(PxVec3(164.f / 255.f, 13.f / 255.f, 255.f / 255.f));
			//box->Material(box3Mat);
			//Add(box3);

			for (int i = 0; i < 10; i++)
			{
				ball = new Sphere(PxTransform(PxVec3(.0f, i + 50.f, .0f)));
				ball->Color(color_palette[0]);
				//ball->Material(box2Mat);
				Add(ball);
			}

			//PxClothParticle vertices[] = {
			//	PxClothParticle(PxVec3(0.0f, 0.0f, 0.0f), 0.0f),
			//	PxClothParticle(PxVec3(0.0f, 1.0f, 0.0f), 1.0f),
			//	PxClothParticle(PxVec3(1.0f, 0.0f, 0.0f), 1.0f),
			//	PxClothParticle(PxVec3(1.0f, 1.0f, 0.0f), 1.0f)
			//};

			//PxU32 primitives[] = { 0, 1, 3, 2 };

			//PxClothMeshDesc meshDesc;
			//meshDesc.points.data = vertices;
			//meshDesc.points.count = 4;
			//meshDesc.points.stride = sizeof(PxClothParticle);

			//meshDesc.invMasses.data = &vertices->invWeight;
			//meshDesc.invMasses.count = 4;
			//meshDesc.invMasses.stride = sizeof(PxClothParticle);

			//meshDesc.quads.data = primitives;
			//meshDesc.quads.count = 1;
			//meshDesc.quads.stride = sizeof(PxU32) * 4;

			//PxClothFabric* fabric = PxClothFabricCreate(*physics, meshDesc, PxVec3(0, -1, 0));
			//PxTransform pose = PxTransform(PxIdentity);
			//PxCloth* cloth = physics->createCloth(pose, *fabric, vertices, PxClothFlags());
			////Add(cloth);
			//
			//PxClothCollisionSphere spheres[2] =
			//{
			//	PxClothCollisionSphere(PxVec3(-1.0f, 0.0f, 0.0f), 0.5f),
			//	PxClothCollisionSphere(PxVec3(1.0f, 0.0f, 0.0f), 0.25f)
			//};

			//cloth->setCollisionSpheres(spheres, 2);
			//cloth->addCollisionCapsule(0, 1);
			//cloth->addCollisionPlane(PxClothCollisionPlane(PxVec3(0.0f, 1.0f, 0.0f), 0.0f));
			//cloth->addCollisionConvex(1 << 0); // Convex references the first plane

			//PxClothCollisionTriangle triangles[4] = {
			//	PxClothCollisionTriangle(PxVec3(0.0f, 0.0f, 0.0f),
			//	PxVec3(1.0f, 0.0f, 0.0f),
			//	PxVec3(0.0f, 1.0f, 0.0f)),
			//	PxClothCollisionTriangle(PxVec3(1.0f, 0.0f, 0.0f),
			//	PxVec3(0.0f, 0.0f, 1.0f),
			//	PxVec3(0.0f, 1.0f, 0.0f)),
			//	PxClothCollisionTriangle(PxVec3(0.0f, 0.0f, 1.0f),
			//	PxVec3(0.0f, 0.0f, 0.0f),
			//	PxVec3(0.0f, 1.0f, 0.0f)),
			//	PxClothCollisionTriangle(PxVec3(0.0f, 0.0f, 0.0f),
			//	PxVec3(0.0f, 0.0f, 1.0f),
			//	PxVec3(1.0f, 0.0f, 0.0f)),
			//};

			//static PxVec3 weights[] =
			//{
			//	PxVec3(1.0f / 3, 1.0f / 3, 1.0f / 3), // center point
			//	PxVec3(4.0f / 6, 1.0f / 6, 1.0f / 6), // off-center point
			//};

			//PxU32 numFaces = meshDesc.triangles.count;
			//assert(meshDesc.flags & PxMeshFlag::e16_BIT_INDICES);
			//PxU8* triangles = (PxU8*)meshDesc.triangles.data;

			//PxU32 indices[] = new PxU32[4 * 4 * numFaces];
			//for (PxU32 i = 0, *it = indices; i < numFaces; i++)
			//{
			//	PxU16* triangle = (PxU16*)triangles;
			//	PxU32 v0 = triangle[0];
			//	PxU32 v1 = triangle[1];
			//	PxU32 v2 = triangle[2];

			//	// center
			//	*it++ = v0; *it++ = v1; *it++ = v2; *it++ = 0;

			//	// off centers
			//	*it++ = v0; *it++ = v1; *it++ = v2; *it++ = 1;
			//	*it++ = v1; *it++ = v2; *it++ = v0; *it++ = 1;
			//	*it++ = v2; *it++ = v0; *it++ = v1; *it++ = 1;

			//	triangles += meshDesc.triangles.stride;
			//}

			//cloth->setVirtualParticles(numFaces * 4, indices, 2, weights);
			//delete[] indices;
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			PxVec3 up = PxVec3(.0f, 1.f, .0f);
			PxVec3 vel = ((PxRigidBody*)ball->Get())->getLinearVelocity();
			PxReal dir = vel.normalize();

			PxVec3 crs = vel.cross(up);
			PxQuat rot = PxQuat();
		}
	};
}
