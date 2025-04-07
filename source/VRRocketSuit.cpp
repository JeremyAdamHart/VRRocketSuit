// OpenVRTest.cpp : Defines the entry point for the console application.
//

#include "VRWindow.h"

using namespace glm;

int main()
{
	//Dimensions 0.5 x 2 x 0.5
	RigidBody body(1.f, glm::mat3(0.354167f, 0, 0, 0, 0.0416667, 0, 0, 0, 0.354167));
	body.position = vec3(0, -3.11926, 0);
	body.v = vec3(0, -0.789059, 0);
	quat rx = angleAxis(0.785398f, vec3(1, 0, 0));
	quat ry = angleAxis(0.785398f, vec3(0, 1, 0));
	body.orientation = ry*rx;
//	body.torque = vec3(0.375, 0, -0.375);
	body.addForce(vec3(0, 0.5, 0), vec3(-0.551777, -4.00315, -0.198223));
	body.addForce(vec3(0, 0.5, 0), vec3(-0.198223, -4.00315, -0.551777));

	body.resolveForces(1.f / 64.f);
	//Force constant up to height of -4

	WindowManager wm(800, 400, "OpenVR Test");
	wm.mainLoop();
}