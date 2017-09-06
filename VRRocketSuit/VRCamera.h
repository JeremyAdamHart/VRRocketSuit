#pragma once

#include <Camera.h>
#include <glmSupport.h>
#include <openvr.h>

class VRCamera : public renderlib::Camera {

	glm::mat4 camMatrix;

public:
	VRCamera();

	void setCameraMatrix(glm::mat4 newCamMatrix);

	virtual glm::mat4 getCameraMatrix() const;
	virtual glm::vec3 getPosition() const;
};

class VRCameraController {
	vr::TrackedDevicePose_t *headsetPose;
	glm::mat4 leftEyeTransform;
	glm::mat4 rightEyeTransform;
public:
	VRCamera leftEye;
	VRCamera rightEye;

	VRCameraController(vr::TrackedDevicePose_t *headsetPose, 
		vr::IVRSystem *vrDisplay);

	void setEyeTransforms(vr::IVRSystem *vrDisplay);
	void setProjection(vr::IVRSystem *vrDisplay, 
		float near=0.01f, float far=10.f);
	void update();		//Updates camera orientations
};