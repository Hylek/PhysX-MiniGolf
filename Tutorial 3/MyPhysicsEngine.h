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

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2),
			GOLFBALL    = (1 << 3),
			FLAGPOLE    = (1 << 4),
			PUTTER      = (1 << 5)
		};
	};

	///An example class showing the use of springs (distance joints).
	class Trampoline
	{
		vector<DistanceJoint*> springs;
		Box *bottom, *top;

	public:
		Trampoline(const PxVec3& dimensions=PxVec3(1.f,1.f,1.f), PxReal stiffness=1.f, PxReal damping=1.f)
		{
			PxReal thickness = .1f;
			bottom = new Box(PxTransform(PxVec3(0.f,thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			top = new Box(PxTransform(PxVec3(0.f,dimensions.y+thickness,0.f)),PxVec3(dimensions.x,thickness,dimensions.z));
			springs.resize(4);
			springs[0] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,dimensions.z)));
			springs[1] = new DistanceJoint(bottom, PxTransform(PxVec3(dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(dimensions.x,-dimensions.y,-dimensions.z)));
			springs[2] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,dimensions.z)));
			springs[3] = new DistanceJoint(bottom, PxTransform(PxVec3(-dimensions.x,thickness,-dimensions.z)), top, PxTransform(PxVec3(-dimensions.x,-dimensions.y,-dimensions.z)));

			for (unsigned int i = 0; i < springs.size(); i++)
			{
				springs[i]->Stiffness(stiffness);
				springs[i]->Damping(damping);
			}
		}

		void AddToScene(Scene* scene) const
		{
			scene->Add(bottom);
			scene->Add(top);
		}

		~Trampoline()
		{
			for (unsigned int i = 0; i < springs.size(); i++)
				delete springs[i];
		}
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;

						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
					
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}

				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_CCD)
				{
					cerr << "onContact::eNOTIFY_TOUCH_CCD" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_CCD;

			// Return ekill to ignore the collision between the golfball and flagpole DISABLE WHEN TESTING
			return PxFilterFlag::eKILL | PxFilterFlag::eNOTIFY;
		}

		return PxFilterFlags();
	};

	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Box* box, *box2, *box3;
		Box* putterJoint;
		MySimulationEventCallback* my_callback;
		Putter* putter;
		WindmillBase* windmillBase;
		WindmillBlades* windmillBlades;
		SandBank* sandBank;
		Box* windmillCenter;
		StaticBox* holeTrigger;
		StaticBox* flagPole;
		Sphere* ball;
		PxMaterial* barrierMaterial;
		PxMaterial* planeMaterial;

		PxMaterial* concrete;
		PxMaterial* ballMaterial;
		PxMaterial* sand;

		RevoluteJoint* pJoint;
		RevoluteJoint* windmillJoint;
		TeeBox* teeBox;
		Cloth* poleFlag;


		PxTransform pJointLocation;

		PxReal speed = 0;
		PxReal bladeSpeed = 1;

		//bool switchTriggers = false;
		bool swingBack = false;
		bool hasWon = false;


		// Course objects
		CoursePlanes* planes;
		CourseBarriers* barriers;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene(CustomFilterShader) {};

		///A custom scene class
		void SetVisualisation() const
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_HORIZONTAL, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_VERTICAL, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_BENDING, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_SHEARING, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit()
		{
			SetVisualisation();

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			planeMaterial = GetPhysics()->createMaterial(.4f, .3f, .1f);
			barrierMaterial = GetPhysics()->createMaterial(0.f, 0.f, .6f);
			concrete = GetPhysics()->createMaterial(.4f, .6f, .4f);
			sand = GetPhysics()->createMaterial(5.f, 8.f, .0f);
			ballMaterial = GetPhysics()->createMaterial(.4f, .15f, .4f);
			ballMaterial->setFrictionCombineMode(PxCombineMode::eAVERAGE);

			// THESE COMMENTS WERE FOR TESTING
			//PxReal offset = 0;
			//for (int i = 0; i < 1; i++)
			//{
				ball = new Sphere(PxTransform(PxVec3(.0f /*+ offset*/, 10.f, -2.5f)), .2f);
				ball->Color(PxVec3(210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
				ball->Name("GolfBall");
				((PxRigidBody*)ball->Get())->setMass(0.045f);
				((PxRigidBody*)ball->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
				((PxRigidDynamic*)ball->Get())->setLinearDamping(.25f);
				ball->Material(ballMaterial);

				// filter the golfball 
				ball->SetupFiltering(FilterGroup::GOLFBALL, FilterGroup::FLAGPOLE);
				Add(ball);


				teeBox = new TeeBox(PxTransform(PxVec3(0.f /*+ offset*/, 0.f, -2.5f)));
				teeBox->Color(PxVec3(60.f / 255.f, 60.f / 255.f, 60.f / 255.f));
				teeBox->Material(concrete);
				Add(teeBox);

				putterJoint = new Box(PxTransform(PxVec3(0.f /*+ offset*/, 10.f, 0.f)));
				putterJoint->SetKinematic(true);
				Add(putterJoint);

				putter = new Putter(PxTransform(PxVec3(.1f /*+ offset*/, 5.f, .0f)));
				putter->Color(color_palette[2]);
				putter->Name("Putter");
				((PxRigidBody*)putter->Get())->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
				putter->SetupFiltering(FilterGroup::PUTTER, FilterGroup::GOLFBALL);
				((PxRigidBody*)putter->Get())->setMass(0.355f);
				Add(putter);

				pJoint = new RevoluteJoint(putterJoint, PxTransform(PxVec3(0.f, -4.f, 0.f), PxQuat(PxPi / 2, PxVec3(1.f, 0.f, 0.f))), putter, PxTransform(PxVec3(0.f, 5.f, 0.f)));
				pJoint->SetLimits(-PxPi / 1.4f, -PxPi / 3.5f);

				planes = new CoursePlanes(PxTransform(PxVec3(.0f /*+ offset*/, .0f, .0f)));
				barriers = new CourseBarriers(PxTransform(PxVec3(.0f /*+ offset*/, 1.f, .0f)));
				sandBank = new SandBank(PxTransform(PxVec3(.0f /*+ offset*/, .0f, .0f)));

				planes->Color(PxVec3(0.f / 255.f, 160.f / 255.f, 20.f / 255.f));
				barriers->Color(PxVec3(178.f / 255.f, 70.f / 255.f, 34.f / 255.f));
				sandBank->Color(PxVec3(235.f / 235.f, 255.f / 255.f, 0.f / 255.f));
				sandBank->Material(sand);
				planes->Material(planeMaterial);
				Add(planes);
				Add(barriers);
				Add(sandBank);

				windmillBase = new WindmillBase(PxTransform(PxVec3(.0f /*+ offset*/, .0f, .0f)));
				windmillBase->Color(PxVec3(255.f / 255.f, 0 / 255.f, 0 / 255.f));
				Add(windmillBase);


				windmillBlades = new WindmillBlades(PxTransform(PxVec3(.0f /*+ offset*/, 6.5f, -19.5f)));
				Add(windmillBlades);

				windmillCenter = new Box(PxTransform(PxVec3(0.f /*+ offset*/, 6.5f, -19.5f)));
				windmillCenter->SetKinematic(true);
				Add(windmillCenter);

				windmillJoint = new RevoluteJoint(windmillCenter, PxTransform(PxVec3(0.f, 0.f, 0.f), PxQuat(PxPi / 2, PxVec3(0.f, 1.f, 0.f))), windmillBlades, PxTransform(PxVec3(0.f, 0.f, 0.f)));
				windmillJoint->DriveVelocity(1);

				holeTrigger = new StaticBox(PxTransform(PxVec3(0 /*+ offset*/, -2.f, -45.5f)), PxVec3(2.f, .1f, 2.f));
				holeTrigger->SetTrigger(true);
				Add(holeTrigger);

				flagPole = new StaticBox(PxTransform(PxVec3(.0f /*+ offset*/, 4.4f, -45.5f)), PxVec3(.1f, 4.f, .1f));
				flagPole->SetupFiltering(FilterGroup::FLAGPOLE, FilterGroup::GOLFBALL);
				flagPole->Name("FlagPole");
				Add(flagPole);

				poleFlag = new Cloth(PxTransform(PxVec3(0.1f /*+ offset*/, 6.5f, -45.5f)), PxVec2(2.f, 1.5f), 40, 40, true);
				poleFlag->Color(PxVec3(255.f / 255.f, 255.f / 255.f, 0.f / 255.f));
				((PxCloth*)poleFlag->Get())->setExternalAcceleration(PxVec3(11.f, 1.f, .5f));
				Add(poleFlag);

		/*		offset += 10;*/
			//}


			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
		}

		virtual void CustomUpdate() 		
		{
			pJointLocation = ((PxRigidBody*)putterJoint->Get())->getGlobalPose();

			// Display speed
			cerr << speed << endl;

			((PxRigidDynamic*)putter->Get())->wakeUp();

			//cerr << "x= " << ((PxRigidBody*)ball->Get())->getLinearVelocity().x << endl;
		    //cerr << "y= " << ((PxRigidBody*)ball->Get())->getLinearVelocity().y << endl;
			//cerr << "z= " << ((PxRigidBody*)ball->Get())->getLinearVelocity().z << endl;

			PxTransform ballPose = ((PxRigidBody*)ball->Get())->getGlobalPose();
			if (ballPose.p.y <= -5 && !hasWon)
			{
				ResetBall();
			}

			if (pJointLocation.p.y <= 9.4f)
			{
				pJointLocation.p.y = 9.4f;
			}

			if (swingBack)
			{
				((PxRigidBody*)putter->Get())->addForce(PxVec3(.0f, .0f, 1.f), PxForceMode::eIMPULSE);
			}

			if(my_callback->trigger)
			{
				cerr << "YOU WIN" << endl;
				hasWon = true;
			}

			if (hasWon)
			{
				windmillBase->Color(PxVec3(0.f / 255.f, 255.f / 255.f, 0.f / 255.f));
				windmillJoint->DriveVelocity(5);
			}
			else
			{
				windmillBase->Color(PxVec3(255.f / 255.f, 0.f / 255.f, 0.f / 255.f));
			}

		}
			
		void ResetBall()
		{
			((PxRigidBody*)ball->Get())->setGlobalPose(PxTransform(PxVec3(.0f, 4.f, -2.5f)));
			((PxRigidBody*)ball->Get())->setLinearVelocity(PxVec3(.0f, .0f, .0f));
			((PxRigidBody*)ball->Get())->setAngularVelocity(PxVec3(.0f, .0f, .0f));
			hasWon = false;
			windmillJoint->DriveVelocity(1);
		}

		void Fire()
		{
			swingBack = false;
			((PxRigidBody*)putter->Get())->addForce(PxVec3(.0f, .0f, -speed), PxForceMode::eIMPULSE);
		}

		void SwingBack()
		{
			((PxRigidBody*)putter->Get())->addForce(PxVec3(.0f, .0f, 1.f), PxForceMode::eIMPULSE);
			swingBack = true;
			speed = 0;
		}

		void ReleaseMotor() const
		{
			((PxRigidBody*)putter->Get())->addForce(PxVec3(.0f, .0f, -10.f), PxForceMode::eIMPULSE);
		}

		void SetSpeed()
		{
			if (speed <= 20.f)
			{
				speed++;
			}
			else
			{
				speed = 5;
			}
		}

		void MovePutterForward() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y, pJointLocation.p.z - .05f), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void MovePutterBack() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y, pJointLocation.p.z + .05f), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void MovePutterLeft() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x - .05f, pJointLocation.p.y, pJointLocation.p.z), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void MovePutterRight() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x + .05f, pJointLocation.p.y, pJointLocation.p.z), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void MovePutterUp() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y + .005f, pJointLocation.p.z), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void MovePutterDown() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y - .005f, pJointLocation.p.z), PxQuat(pJointLocation.q.x, pJointLocation.q.y, pJointLocation.q.z, pJointLocation.q.w)));
		}

		void RotatePutterLeft() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y, pJointLocation.p.z), PxQuat(pJointLocation.q.y + .45f, PxVec3(0.f, 1.f, .0f))));
		}

		void RotatePutterRight() const
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y, pJointLocation.p.z), PxQuat(pJointLocation.q.y + -.45f, PxVec3(0.f, 1.f, .0f))));
		}

		void ResetRotation()
		{
			((PxRigidBody*)putterJoint->Get())->setGlobalPose(PxTransform(PxVec3(pJointLocation.p.x, pJointLocation.p.y, pJointLocation.p.z), PxQuat(0.f, PxVec3(0.f, 1.f, .0f))));
		}

		/// An example use of key release handling
		void ExampleKeyReleaseHandler() const
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler() const
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
