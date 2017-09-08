#include "VRWindow.h"

#include <iostream>

using namespace glm;
using namespace std;

#include "Drawable.h"
#include "SimpleGeometry.h"
#include "SimpleShader.h"
#include "ColorMat.h"
#include "TrackballCamera.h"
#include "SimpleTexManager.h"
#include "simpleTexShader.h"
#include "TextureCreation.h"
#include "TextureMat.h"
#include "MeshInfoLoader.h"
#include "ShadedMat.h"
#include "TorranceSparrowShader.h"
#include "Framebuffer.h"
#include "VRController.h"
#include "ModelLoader.h"

#include "HeatParticleShader.h"
#include "HeatParticleGeometry.h"
#include "HeatParticleMat.h"
#include "HeatParticleSystem.h"
#include "BlendShader.h"


#include <glm/gtc/matrix_transform.hpp>

const float PI = 3.14159265358979323846;

using namespace renderlib;

TrackballCamera cam(
	vec3(0, 0, -1), vec3(0, 0, 1),
	//	glm::perspective(90.f*3.14159f/180.f, 1.f, 0.1f, 3.f));
	mat4(1.f));

void cursorPositionCallback(GLFWwindow *window, double xpos, double ypos) {
	static vec2 lastPos = vec2(0.f, 0.f);
	
	int vp[4];
	glGetIntegerv(GL_VIEWPORT, vp);
	vec2 mousePos = vec2(float(xpos) / float(vp[2]), 
		float(-ypos) / float(vp[3]))*2.f
		- vec2(1.f, 1.f);

	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
		vec2 diff = mousePos - lastPos;
		cam.trackballRight(-diff.x*3.14159f);
		cam.trackballUp(-diff.y*3.14159f);
	}

	lastPos = mousePos;
}

WindowManager::WindowManager() :
window_width(800), window_height(800)
{
	glfwInit();
	window = createWindow(window_width, window_height, 
		"You really should rename this");
	initGlad();
	vrDisplay = initVR();

	glfwSwapInterval(0);

	initGL();
}

WindowManager::WindowManager(int width, int height, std::string name, glm::vec4 color) :
	window_width(width), window_height(height) 
{
	glfwInit();
	window = createWindow(window_width, window_height, name);
	initGlad();
	vrDisplay = initVR();

	glfwSwapInterval(0);

	initGL();
}

void printMat4(const mat4 &m) {
	printf("[%.2f, %.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f, %.2f]\n[%.2f, %.2f, %.2f, %.2f]\n\n",
		m[0][0], m[0][1], m[0][2], m[0][3],
		m[1][0], m[1][1], m[1][2], m[1][3],
		m[2][0], m[2][1], m[2][2], m[2][3],
		m[3][0], m[3][1], m[3][2], m[3][3]);
}

//Temporary testing
void WindowManager::mainLoop() {
	glfwSetCursorPosCallback(window, cursorPositionCallback);

	vec3 points[6] = {
		//First triangle
		vec3(-0.5f, 0.5f, 0.f)*2.f,
		vec3(0.5f, 0.5f, 0.f)*2.f,
		vec3(0.5f, -0.5f, 0.f)*2.f,
		//Second triangle
		vec3(0.5f, -0.5f, 0.f)*2.f,
		vec3(-0.5f, -0.5f, 0.f)*2.f,
		vec3(-0.5f, 0.5f, 0.f)*2.f
	};

	vec2 coords[6] = {
		//First triangle
		vec2(1, 0.f),
		vec2(0.f, 0.f),
		vec2(0.f, 1.f),
		//Second triangle
		vec2(0.f, 1.f),
		vec2(1.f, 1.f),
		vec2(1.f, 0.f)
	};

	SimpleTexManager tm;

	if (vrDisplay == nullptr) {
		vr::VR_Shutdown();
		glfwTerminate();
		return;
	}

	Framebuffer fbWindow(window_width, window_height);
	unsigned int TEX_WIDTH = 800;
	unsigned int TEX_HEIGHT = 800;
	vrDisplay->GetRecommendedRenderTargetSize(&TEX_WIDTH, &TEX_HEIGHT);

	vec2 icoords[6] = {
		vec2(0, TEX_HEIGHT),
		vec2(TEX_WIDTH, TEX_HEIGHT),
		vec2(TEX_WIDTH, 0),
		//Second triangle
		vec2(TEX_WIDTH, 0),
		vec2(0, 0),
		vec2(0, TEX_HEIGHT)
	};

	Framebuffer fbLeftEyeDraw = createNewFramebuffer(TEX_WIDTH, TEX_HEIGHT);
	Framebuffer fbRightEyeDraw = createNewFramebuffer(TEX_WIDTH, TEX_HEIGHT);
	Framebuffer fbLeftEyeRead = createNewFramebuffer(TEX_WIDTH, TEX_HEIGHT);
	Framebuffer fbRightEyeRead = createNewFramebuffer(TEX_WIDTH, TEX_HEIGHT);


	const int NUM_SAMPLES = 4;

	if (!fbLeftEyeDraw.addTexture(
		createTexture2DMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES),
		GL_COLOR_ATTACHMENT0) ||
		!fbLeftEyeDraw.addTexture(
		createTexture2DMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES),
		GL_COLOR_ATTACHMENT1) ||
		!fbLeftEyeDraw.addTexture(
			createDepthTextureMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES), GL_DEPTH_ATTACHMENT))
	{
		std::cout << "FBO creation failed" << endl;
	}
	if (!fbRightEyeDraw.addTexture(
		createTexture2DMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES),
		GL_COLOR_ATTACHMENT0) || 
		!fbRightEyeDraw.addTexture(
			createTexture2DMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES),
			GL_COLOR_ATTACHMENT1) ||
		!fbRightEyeDraw.addTexture(
			createDepthTextureMulti(TEX_WIDTH, TEX_HEIGHT, &tm, NUM_SAMPLES), GL_DEPTH_ATTACHMENT))
	{
		std::cout << "FBO creation failed" << endl;
	}

	if (!fbLeftEyeRead.addTexture(
		createTexture2D(TEX_WIDTH, TEX_HEIGHT, &tm), GL_COLOR_ATTACHMENT0)) {
		std::cout << "FBO creation failed" << endl;
	}
	else if (!fbRightEyeRead.addTexture(
		createTexture2D(TEX_WIDTH, TEX_HEIGHT, &tm), GL_COLOR_ATTACHMENT0)) {
		std::cout << "FBO creation failed" << endl;
	}

	Viewport leftEyeView(window_width / 2, window_height);
	Viewport rightEyeView(window_width / 2, window_height, window_width / 2);

	//Parse tracked devices
	int headsetIndex = 0;
	vector<VRController> controllers;
	vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];

	for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
		vr::TrackedDeviceClass deviceClass = vrDisplay->GetTrackedDeviceClass(i);

		switch (deviceClass) {
		case vr::TrackedDeviceClass_HMD:
			headsetIndex = i;
			break;
		case vr::TrackedDeviceClass_Controller:
			controllers.push_back(VRController(i, vrDisplay, poses[i], &tm));
			break;
		}
	}

	VRCameraController vrCam(&poses[headsetIndex], vrDisplay);

	//Squares for left and right views
	Drawable leftSquare(
		new TextureMat(fbLeftEyeRead.getTexture(GL_COLOR_ATTACHMENT0)),
		new SimpleTexGeometry(points, coords, 6, GL_TRIANGLES));

	Drawable rightSquare(
		new TextureMat(fbRightEyeRead.getTexture(GL_COLOR_ATTACHMENT0)),
		new SimpleTexGeometry(points, coords, 6, GL_TRIANGLES));
	
	SimpleTexShader texShader;
	TorranceSparrowShader tsShader;
	TorranceSparrowShader tsTexShader({{ GL_FRAGMENT_SHADER, "#define USING_TEXTURE\n" }
	});
	HeatParticleShader thrusterShader;
	BlendShader blendShader(NUM_SAMPLES);

	Disk particleSpawners[2];
	for (int i = 0; i < 2; i++) {
		particleSpawners[i] = Disk(0.0125f, vec3(0.f, 0.f, -2.f), vec3(0.f, 1.f, 0.f));
	}
	HeatParticleSystem pSystem;
	HeatParticleGeometry pGeometry;
	HeatParticleMat pMat(0.07);
	Drawable blenderLeft(
		new TextureMat(fbLeftEyeDraw.getTexture(GL_COLOR_ATTACHMENT0)),
		new SimpleTexGeometry(points, icoords, 6, GL_TRIANGLES));
	blenderLeft.addMaterial(
		new TextureMat(fbLeftEyeDraw.getTexture(GL_COLOR_ATTACHMENT1), 
			TextureMat::TRANSLUCENT));

	Drawable blenderRight(
		new TextureMat(fbRightEyeDraw.getTexture(GL_COLOR_ATTACHMENT0)),
		new SimpleTexGeometry(points, icoords, 6, GL_TRIANGLES));
	blenderRight.addMaterial(
		new TextureMat(fbRightEyeDraw.getTexture(GL_COLOR_ATTACHMENT1),
			TextureMat::TRANSLUCENT));

	Drawable pDrawable(&pMat, &pGeometry);

	float initialVelocity = 8.0f;
	float lifespan = 0.1f;
	float heat = 0.1f;
	float divergenceAngle = PI / 8.f;
	const int particlesPerStep = 250;

	for (int i = 0; i < 500; i++) {
		pSystem.addParticleFromDisk(particleSpawners[i%2], initialVelocity,
			heat, lifespan, divergenceAngle);
	}

	float timeElapsed = 0.f;

	float thrust[2];

	/*ElementGeometry dragonGeom = objToElementGeometry("models/dragon.obj");
	Drawable dragon(
		new ColorMat(vec3(0.75f, 0.1f, 0.3f)),
		&dragonGeom);
	dragon.addMaterial(new ShadedMat(0.2f, 0.5f, 0.3f, 10.f));

	dragon.setPosition(vec3(0.f, 0, -1.f));
	dragon.setOrientation(angleAxis(-PI*0.5f, vec3(0.f, 1.f, 0.f)));*/

	vec3 lightPos(-100.f, 100.f, 100.f);

	fbWindow.use();

	vector<Drawable> drawables;
	loadWavefront("untrackedmodels/OrganodronCity/", "OrganodronCity", &drawables, &tm);
//	loadWavefront("untrackedmodels/SciFiCenter/CenterCity/", "scificity", &drawables, &tm);

	for (int i = 0; i < drawables.size(); i++) {
		drawables[i].setPosition(vec3(0, 0, -2.f));
		drawables[i].setScale(vec3(1.f));
	}
	
	vector<vec3> controllerPositions(controllers.size());

	//Velocity
	vec3 linearVelocity(0.f);
	quat angularVelocity = quat();

	while (!glfwWindowShouldClose(window)) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Get pose
		vr::VRCompositor()->WaitGetPoses(poses, vr::k_unMaxTrackedDeviceCount, 
			NULL, 0);

		//Update camera
		vrCam.update();
		vrCam.setProjection(vrDisplay, 0.2f, 400.f);

		//Update controllers
		vector<int> triggersPressed;
		for (int i = 0; i < controllers.size(); i++) {
			vr::VRControllerState_t state;
			vr::TrackedDevicePose_t pose;
			if (!vrDisplay->GetControllerStateWithPose(
				vr::VRCompositor()->GetTrackingSpace(),
				controllers[i].index,
				&state,
				sizeof(vr::VRControllerState_t),
				&pose)) 
			{
				printf("Error reading controller state\n");
			}

			controllers[i].updatePose(poses[controllers[i].index]);
			controllers[i].updateState(state);
			if (controllers[i].axes[VRController::TRIGGER].x > 0.5f)
				triggersPressed.push_back(i);
		}

		vec3 positionTransform(0.f);
		quat orientationTransform;
		
		bool updatePositions = true;

		/*switch (triggersPressed.size()) {
		case 1:
		{
			vec3 lastPos = controllerPositions[triggersPressed[0]];
			positionTransform = controllers[triggersPressed[0]].getPos() - lastPos;
			linearVelocity = positionTransform;
			break;
		}
		case 2:
		{
			vec3 axisA = normalize(controllerPositions[triggersPressed[0]]
				- controllerPositions[triggersPressed[1]]);
			vec3 axisB = normalize(controllers[triggersPressed[0]].getPos()
				- controllers[triggersPressed[1]].getPos());
			vec3 rotAxis = cross(axisA, axisB);
			if (length(rotAxis) > 0.0001) {
				float angle = asin(length(rotAxis));
				orientationTransform = angleAxis(angle, normalize(rotAxis));
				angularVelocity = orientationTransform;
			}
			else
				updatePositions = false;
			break;

		}
		default:
			for (int i = 0; i < drawables.size(); i++) {
				quat orientation = drawables[i].getOrientationQuat();
				drawables[i].setOrientation(normalize(angularVelocity*orientation));
				drawables[i].setPosition(drawables[i].getPos() + linearVelocity);
			}

			angularVelocity = slerp(angularVelocity, quat(), 0.01f);
			linearVelocity *= 0.99f;
		}

		if (updatePositions) {
			for (int i = 0; i < controllerPositions.size(); i++) {
				controllerPositions[i] = controllers[i].getPos();
			}
		}
		

		//Update model
		for (int i = 0; i < drawables.size(); i++) {
			quat orientation = drawables[i].getOrientationQuat();
			drawables[i].setOrientation(normalize(orientationTransform*orientation));
			drawables[i].setPosition(drawables[i].getPos() + positionTransform);
			
		}*/

		////////////////////
		//	PARTICLE SIMULATION
		////////////////////
		float currentTime = glfwGetTime();
		float dt = currentTime - timeElapsed;

		for (int j = 0; j < 2; j++) {
			thrust[j] = controllers[j].axes[VRController::TRIGGER].x;
			vec4 newDiskPosition = controllers[j].getTransform()*vec4(0, 0, 0, 1);
			vec4 newDiskNormal = controllers[j].getTransform()*vec4(0, -1, 0, 0);
			particleSpawners[j].origin = vec3(newDiskPosition.x, newDiskPosition.y, newDiskPosition.z);
			particleSpawners[j].normal = vec3(newDiskNormal.x, newDiskNormal.y, newDiskNormal.z);
			particleSpawners[j].generatePerpendicular();

			float timeOffset = 0.f;
			for (int i = 0; i<int((0.f + 2.f*thrust[j])*float(particlesPerStep) / 2.f); i++) {
				float newHeat = heat*(float(rand()) / float(RAND_MAX))*(0.75f + thrust[j] / 4.f);
				pSystem.addParticleFromDisk(particleSpawners[j], initialVelocity*(0.5f + thrust[j] / 2.f),
					newHeat, lifespan, divergenceAngle*(1.f - thrust[j])*2.f, timeOffset);
				timeOffset += (dt) / float(particlesPerStep);
			}
		}

		pSystem.runSimulation((currentTime - timeElapsed)*0.5f);

		pGeometry.loadParticles(pSystem.particles.data(), pSystem.particles.size());

		timeElapsed = currentTime;

		////////////////////
		//	APPLY THRUST
		///////////////////
		static bool gravityOn = false;

		vec3 linearAcceleration;
		float maxAcceleration = 8.f;

		if (thrust[0] + thrust[1] > 0.0001f)
			gravityOn = true;

		float DAMPING = 0.2f;
		/*vec3 GRAVITY = (gravityOn) ? vec3(0.f, 9.81f, 0.f): vec3(0.f, 0.f, 0.f);

		vec3 acceleration(0.f);
		acceleration += thrust[0] * particleSpawners[0].normal*maxAcceleration;
		acceleration += thrust[1] * particleSpawners[1].normal*maxAcceleration;

		linearVelocity += acceleration*dt - DAMPING*linearVelocity*dt + GRAVITY*dt;
		vec3 positionOffset = linearVelocity*dt;
		*/

		vec3 centerOfMass = (0.5f*vrCam.leftEye.getPosition() + 
			0.5f*vrCam.rightEye.getPosition());
		centerOfMass.y *= 0.6;

		if (gravityOn) {

			vec3 fA = thrust[0] * particleSpawners[0].normal*maxAcceleration;
			vec3 fB = thrust[1] * particleSpawners[1].normal*maxAcceleration;
			vec3 locA = particleSpawners[0].origin - centerOfMass;
			vec3 locB = particleSpawners[1].origin - centerOfMass;

			fA = vrCam.modelToWorld(vec4(fA, 0.f));
			fB = vrCam.modelToWorld(vec4(fB, 0.f));
			locA = vrCam.modelToWorld(vec4(locA, 1.f));
			locB = vrCam.modelToWorld(vec4(locB, 1.f));

			vrCam.addLinearForceOnly(-fA);
			vrCam.addLinearForceOnly(-fB);
			vrCam.addTorqueOnly(-fA*1.f, locA);
			vrCam.addTorqueOnly(-fB*1.f, locB);

			vrCam.addGravityForces();
			vrCam.addDampingForces(DAMPING);

			//Correct deviation from up vector
			vec3 worldUp(0.f, 1.f, 0.f);
			vec3 playerUp = normalize(vrCam.modelToWorld(vec4(0.f, 1.f, 0.f, 0.f)));
			vec3 rotAxis = cross(playerUp, worldUp);
			if (length(rotAxis) > 0.00001f)
				rotAxis = normalize(rotAxis);
			else
				rotAxis = vec3(0.f);
			float correctionWeight = (1.f - dot(playerUp, worldUp))*30.f;
			vrCam.torque += correctionWeight*rotAxis;

			vrCam.resolveForces(dt);
		}

		//Update submatrices
		mat4 subMatrix = inverse(vrCam.getOrientationMat4())*translateMatrix(-vrCam.getPos());	//vrCam.getTransform();

		for (int i = 0; i < drawables.size(); i++) {
//			drawables[i].setPosition(drawables[i].getPos() + positionOffset);
			//drawables[i].setPosition(-vrCam.getPos());
			//drawables[i].setOrientation(inverse(vrCam.getOrientationQuat()));
		}

		////////////////////
		//	DRAW
		////////////////////
		glEnable(GL_MULTISAMPLE);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		glClearColor(0.f, 0.f, 0.f, 0.f);

		//Draw left eye
		fbLeftEyeDraw.use();
		glDrawBuffer(GL_COLOR_ATTACHMENT0);	//Draw opaque
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		tsShader.draw(vrCam.leftEye, lightPos, dragon);
		for (int i = 0; i < controllers.size(); i++)
			tsTexShader.draw(vrCam.leftEye, lightPos, controllers[i]);

		for (int i = 0; i < drawables.size(); i++) {
			vrCam.leftEye.subMatrix = subMatrix;
			vrCam.rightEye.subMatrix = subMatrix;
			if (drawables[i].getMaterial(TextureMat::id) != nullptr) {
				tsTexShader.draw(vrCam.leftEye, lightPos, drawables[i]);
			}
			else
				tsShader.draw(vrCam.leftEye, lightPos, drawables[i]);
			vrCam.leftEye.subMatrix = mat4(1.f);
			vrCam.rightEye.subMatrix = mat4(1.f);
		}
		glDrawBuffer(GL_COLOR_ATTACHMENT1);	//Draw translucent
		glClear(GL_COLOR_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		glDepthMask(GL_FALSE);
		thrusterShader.draw(vrCam.leftEye, pDrawable);

		//Draw right eye
		glEnable(GL_MULTISAMPLE);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		fbRightEyeDraw.use();
		glDrawBuffer(GL_COLOR_ATTACHMENT0);	//Draw opaque
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//		tsShader.draw(vrCam.rightEye, lightPos, dragon);
		for (int i = 0; i < controllers.size(); i++)
			tsTexShader.draw(vrCam.rightEye, lightPos, controllers[i]);
		for (int i = 0; i < drawables.size(); i++) {
			vrCam.leftEye.subMatrix = subMatrix;
			vrCam.rightEye.subMatrix = subMatrix;
			if (drawables[i].getMaterial(TextureMat::id) != nullptr) {
				tsTexShader.draw(vrCam.rightEye, lightPos, drawables[i]);
			}
			else
				tsShader.draw(vrCam.rightEye, lightPos, drawables[i]);
			vrCam.leftEye.subMatrix = mat4(1.f);
			vrCam.rightEye.subMatrix = mat4(1.f);
		}
		glDrawBuffer(GL_COLOR_ATTACHMENT1);	//Draw translucent
		glClear(GL_COLOR_BUFFER_BIT);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
		glDepthMask(GL_FALSE);
		thrusterShader.draw(vrCam.rightEye, pDrawable);

		//Composite together into read buffers
//		blit(fbLeftEyeDraw, fbLeftEyeRead);
//		blit(fbRightEyeDraw, fbRightEyeRead);
		glDisable(GL_BLEND);
		fbLeftEyeRead.use();
		glDrawBuffer(GL_COLOR_ATTACHMENT0);
		glClear(GL_COLOR_BUFFER_BIT);
		blendShader.draw(blenderLeft);
		fbRightEyeRead.use();
		glDrawBuffer(GL_COLOR_ATTACHMENT0);
		glClear(GL_COLOR_BUFFER_BIT);
		blendShader.draw(blenderRight);

		glDisable(GL_MULTISAMPLE);

		//Draw window
		glEnable(GL_MULTISAMPLE);
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
		fbWindow.use();
		leftEyeView.use();
		glClearColor(1.0, 1.0, 1.0, 1.0);
		texShader.draw(cam, leftSquare);

		rightEyeView.use();
		glClearColor(1.0, 1.0, 1.0, 1.0);
		texShader.draw(cam, rightSquare);

		//Draw headset
		if (vrDisplay) {
			vr::Texture_t leftTexture = { 
				(void*)(uintptr_t)fbLeftEyeRead.getTexture(GL_COLOR_ATTACHMENT0).getID(), 
				vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
			vr::Texture_t rightTexture = {
				(void*)(uintptr_t)fbRightEyeRead.getTexture(GL_COLOR_ATTACHMENT0).getID(),
				vr::TextureType_OpenGL, vr::ColorSpace_Gamma };

			vr::VRCompositor()->Submit(vr::Eye_Left, &leftTexture);
			vr::VRCompositor()->Submit(vr::Eye_Right, &rightTexture);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}


	delete leftSquare.getMaterial(TextureMat::id);
	delete leftSquare.getGeometryPtr();

	fbLeftEyeDraw.deleteFramebuffer();
	fbLeftEyeDraw.deleteTextures();
	fbRightEyeDraw.deleteFramebuffer();
	fbRightEyeDraw.deleteTextures();

	glfwTerminate();
	vr::VR_Shutdown();
}

void initGlad() {
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "GLAD initialization failed" << std::endl;
	}
}

vr::IVRSystem *initVR() {

	vr::EVRInitError error = vr::VRInitError_None;
	vr::IVRSystem *vrDisplay = vr::VR_Init(&error, vr::VRApplication_Scene);

	if (error != vr::VRInitError_None)
	{
		vrDisplay = nullptr;
		std::cout << "[Error]" << VR_GetVRInitErrorAsSymbol(error) << std::endl;
	}
	else if (!vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &error)){
		printf("Failed to obtain render models\n");
		vrDisplay = nullptr;
	}
	else if (!vr::VRCompositor()) {
		printf("Compositor failed\n");
		vrDisplay = nullptr;
	}

	return vrDisplay;
}

void WindowManager::initGL() {
	glClearColor(1.f, 1.f, 1.f, 1.f);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);

	glViewport(0, 0, window_width, window_height);
}

GLFWwindow *createWindow(int width, int height, std::string name)
{
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	
	GLFWwindow *window = glfwCreateWindow(
		width, height, name.c_str(), nullptr, nullptr);
	
	if (window == NULL) {
		glfwTerminate();
		return nullptr;
	}
	else {
		glfwMakeContextCurrent(window);
		return window;
	}
}

