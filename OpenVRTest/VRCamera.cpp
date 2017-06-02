#include "VRCamera.h"

using namespace glm;

VRCamera::VRCamera(): camMatrix(1.f){}

void VRCamera::setCameraMatrix(mat4 newCamMatrix) {
	camMatrix = newCamMatrix;
}

mat4 VRCamera::getCameraMatrix() const {
	return camMatrix;
}

vec3 VRCamera::getPosition() const {
	vec4 position = inverse(camMatrix)*vec4(0, 0, 0, 1);
	return vec3(position.x, position.y, position.z);
}

VRCameraController::VRCameraController(vr::TrackedDevicePose_t *headsetPose, 
	vr::IVRSystem *vrDisplay) :
headsetPose(headsetPose), leftEyeTransform(1.f), rightEyeTransform(1.f)
{
	setEyeTransforms(vrDisplay);
	setProjection(vrDisplay);
}

mat4 toMat4(const vr::HmdMatrix44_t &hmdMat) {
	return {
		hmdMat.m[0][0], hmdMat.m[1][0], hmdMat.m[2][0], hmdMat.m[3][0],
		hmdMat.m[0][1], hmdMat.m[1][1], hmdMat.m[2][1], hmdMat.m[3][1],
		hmdMat.m[0][2], hmdMat.m[1][2], hmdMat.m[2][2], hmdMat.m[3][2],
		hmdMat.m[0][3], hmdMat.m[1][3], hmdMat.m[2][3], hmdMat.m[3][3]};
}

void VRCameraController::setProjection(vr::IVRSystem *vrDisplay, float near, float far) {

	mat4 leftProjection = toMat4(
		vrDisplay->GetProjectionMatrix(vr::Eye_Left, near, far));
	leftEye.setProjectionMatrix(leftProjection);
	mat4 rightProjection = toMat4(
		vrDisplay->GetProjectionMatrix(vr::Eye_Right, near, far));
	rightEye.setProjectionMatrix(rightProjection);
}

mat4 toMat4(const vr::HmdMatrix34_t &hmdMat) {
	return {
		hmdMat.m[0][0], hmdMat.m[1][0], hmdMat.m[2][0], 0.f,
		hmdMat.m[0][1], hmdMat.m[1][1], hmdMat.m[2][1], 0.f,
		hmdMat.m[0][2], hmdMat.m[1][2], hmdMat.m[2][2], 0.f,
		hmdMat.m[0][3], hmdMat.m[1][3], hmdMat.m[2][3], 1.f };
}

void VRCameraController::setEyeTransforms(vr::IVRSystem *vrDisplay) {
	leftEyeTransform = toMat4(vrDisplay->GetEyeToHeadTransform(vr::Eye_Left));
	rightEyeTransform = toMat4(vrDisplay->GetEyeToHeadTransform(vr::Eye_Right));
}

void VRCameraController::update() {
	mat4 headTransform = inverse(toMat4(headsetPose->mDeviceToAbsoluteTracking));

	leftEye.setCameraMatrix(headTransform*leftEyeTransform);
	rightEye.setCameraMatrix(headTransform*rightEyeTransform);
}