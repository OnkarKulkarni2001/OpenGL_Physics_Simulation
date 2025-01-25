#include "cBasicFlyCamera.h"
#include <glm/gtc/matrix_transform.hpp> // glm::translate, glm::rotate, glm::scale, glm::perspective



cBasicFlyCamera::cBasicFlyCamera()
{
//	this->m_eye = glm::vec3(0.0f, 0.0f, 0.0f);

	this->m_eye = cBasicFlyCamera::m_FRONT_OF_CAMERA;

	this->m_target = glm::vec3(0.0f, 0.0f, 0.0f);

	this->m_Yaw_Y_axis_rotation = 0.0f;
	this->m_Pitch_X_axis_rotation = 0.0f;

	const float DEFAULT_CAMERA_MOVE_SPEED = 0.1f;
	// At 60Hz (FPS) we'll move 1 degree per second
//	const float DEFAULT_CAMERA_TURN_ANGLE_SPEED_SCALING = 1.0f/60.0f;
	const float DEFAULT_CAMERA_TURN_ANGLE_SPEED_SCALING = 1.0f/600.0f;

	this->m_movementSpeed = DEFAULT_CAMERA_MOVE_SPEED;
	this->m_turnSpeedScaling = DEFAULT_CAMERA_TURN_ANGLE_SPEED_SCALING;

}

void cBasicFlyCamera::setEyeLocation(glm::vec3 newEyeLocation)
{
	this->m_eye = newEyeLocation;

	// When we update the eye (location), we will also need to update the "target"
	// But we are going to do this when we call getTargetLocation()
	return;
}

void cBasicFlyCamera::lookAt(const glm::vec3& targetPosition, float offsetX, float offsetY)
{
	// Calculate the adjusted target position with the XY offset
	glm::vec3 adjustedTargetPosition = targetPosition + glm::vec3(offsetX, offsetY, 0.0f);

	// Calculate the forward direction
	glm::vec3 forward = glm::normalize(adjustedTargetPosition - this->m_eye);

	// Ensure the forward vector is valid
	if (glm::length(forward) < 0.0001f)
	{
		// Target is too close; ignore
		return;
	}

	// World up vector (assumes Y is up)
	const glm::vec3 UP = glm::vec3(0.0f, 1.0f, 0.0f);

	// Right vector (perpendicular to both UP and Forward)
	glm::vec3 right = glm::normalize(glm::cross(UP, forward));

	// Adjust the UP vector to ensure it is perpendicular to the Forward vector
	glm::vec3 adjustedUp = glm::cross(forward, right);

	// Build a view matrix to align the camera
	glm::mat4 lookAtMatrix = glm::lookAt(this->m_eye, adjustedTargetPosition, adjustedUp);

	// Extract yaw (Y-axis rotation) and pitch (X-axis rotation) from the matrix
	this->m_Yaw_Y_axis_rotation = atan2(-lookAtMatrix[2][0], lookAtMatrix[2][2]);
	this->m_Pitch_X_axis_rotation = asin(lookAtMatrix[2][1]);

	// Flip the yaw by 180 degrees to correct the inversion
	this->m_Yaw_Y_axis_rotation += glm::pi<float>();

	// Update the internal target direction
	this->m_target = forward;
}

void cBasicFlyCamera::setEyeLocation(float newX, float newY, float newZ)
{
	this->setEyeLocation(glm::vec3(newX, newY, newZ));

	return;
}

float cBasicFlyCamera::getYaw()
{
	return this->m_Yaw_Y_axis_rotation;
}

float cBasicFlyCamera::getPitch()
{
	return this->m_Pitch_X_axis_rotation;
}

glm::vec3 cBasicFlyCamera::getRightVector(void)
{

	glm::mat4 mat_cameraYaw_Y_Axis = glm::rotate(glm::mat4(1.0f),
		this->m_Yaw_Y_axis_rotation,
		glm::vec3(0.0f, 1.0f, 0.0f));

	glm::vec3 someVector = glm::mat3(mat_cameraYaw_Y_Axis) * cBasicFlyCamera::m_RIGHT_SIDE_OF_CAMERA;


	return someVector;
}


glm::vec3 cBasicFlyCamera::getEyeLocation(void)
{
	return this->m_eye;
}

glm::vec3 cBasicFlyCamera::getTargetRelativeToCamera(void)
{
	return glm::normalize(this->m_target);
}

glm::vec3 cBasicFlyCamera::getTargetLocation(void)
{
	// For now, this is simple, but will get more complicated

//	this->m_target = this->m_eye + cBasicFlyCamera::m_FRONT_OF_CAMERA;
	
	// Calcualte where the target will be, given these angles of rotation:
	// float m_Yaw_Y_axis_rotation;
	// float m_Pitch_X_axis_rotation;

// These lines are taken from the DrawObject, whic is using a 4x4 matrix (mat4 or mat4x4)
//	glm::mat4 matRotateX = glm::rotate(glm::mat4(1.0f),
//									   pCurrentMesh->orientation.x,
//									   glm::vec3(1.0f, 0.0f, 0.0f));
//
//	glm::mat4 matRotateY = glm::rotate(glm::mat4(1.0f),
//									   pCurrentMesh->orientation.y,
//									   glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 mat_cameraYaw_Y_Axis = glm::rotate(glm::mat4(1.0f),
												 this->m_Yaw_Y_axis_rotation,
												 glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 mat_cameraPitch_X_Axis = glm::rotate(glm::mat4(1.0f),
												   this->m_Pitch_X_axis_rotation,
												   glm::vec3(1.0f, 0.0f, 0.0f));

//	glm::mat3 matFinalOrienation = glm::mat3(1.0f);
	glm::mat3 matFinalOrienation = glm::mat3(mat_cameraYaw_Y_Axis) * glm::mat3(mat_cameraPitch_X_Axis);


//	From the vertex shader
// 	uniform mat4 mModel;
// 	...
//	vertexWorldPosition = mModel * vec4(vPosition.xyz, 1.0f);

	this->m_target = matFinalOrienation * cBasicFlyCamera::m_FRONT_OF_CAMERA;

	glm::vec3 targetInWorldXYZ = this->m_eye + this->m_target;

	return targetInWorldXYZ;
}

void cBasicFlyCamera::rotateLeftRight_Yaw(float yAngleAdjust)
{
	this->m_Yaw_Y_axis_rotation += (this->m_turnSpeedScaling * yAngleAdjust);
}


void cBasicFlyCamera::rotateLeftRight_Yaw_NoScaling(float yAngleAdjust)
{
	this->m_Yaw_Y_axis_rotation += yAngleAdjust;
}

// positive (+ve) is "looking up"
// negative (-ve) is "looking down"
void cBasicFlyCamera::pitchUpDown(float xAngleAdjust)
{
	this->m_Pitch_X_axis_rotation += (this->m_turnSpeedScaling * xAngleAdjust);

	// Clamp the pitch to +89 or -89 degrees
	// When we use the lookAt() transform, if the target == up, then
	//	we get a NaN (divide by zero)
	// AND if we keep going, then the pitch reverses because of the up vector
	if ( this->m_Pitch_X_axis_rotation <= glm::radians(-89.0f) )
	{
		this->m_Pitch_X_axis_rotation = glm::radians(-89.0f);
	}
	if ( this->m_Pitch_X_axis_rotation >= glm::radians( 89.0f ) )
	{
		this->m_Pitch_X_axis_rotation = glm::radians(89.0f);
	}

	return;
}


// The mouse will pass in integer values, passing "1" per "tick" of the mouse wheel.
// This will likely need to be adjusted per mouse, but who knows?
void cBasicFlyCamera::adjustMovementSpeed(float adjustment)
{
	this->m_movementSpeed += adjustment;

	// See if it's too high or low
	if ( this->m_movementSpeed < cBasicFlyCamera::MIN_MOVEMENT_SPEED )
	{
		this->m_movementSpeed = cBasicFlyCamera::MIN_MOVEMENT_SPEED;
	}
	if (this->m_movementSpeed > cBasicFlyCamera::MAX_MOVEMENT_SPEED)
	{
		this->m_movementSpeed = cBasicFlyCamera::MAX_MOVEMENT_SPEED;
	}
	return;
}

float cBasicFlyCamera::getMovementSpeed(void)
{
	return this->m_movementSpeed;
}


void cBasicFlyCamera::moveForward(float distanceToMove)
{
	float scaledDistance = distanceToMove * this->m_movementSpeed;

//	this->m_eye += glm::vec3(0.0f, 0.0f, scaledDistance);

	// Now we move it in the direction of the target
	// i.e. we are moving "toward" the target
	this->m_eye += (this->m_target * scaledDistance);

	return;
}


void cBasicFlyCamera::moveLeftRight(float distanceToMove)
{
	float scaledDistance = distanceToMove * this->m_movementSpeed;

	// HACK:
//	this->m_eye += glm::vec3(scaledDistance, 0.0f, 0.0f);

	// Similar to the "forward" and "back" 
	// but we are moving in a direction along the x axis
	// (perpendicular to the "forward" or "target"
	// 

	glm::mat4 mat_cameraYaw_Y_Axis = glm::rotate(glm::mat4(1.0f),
												 this->m_Yaw_Y_axis_rotation,
												 glm::vec3(0.0f, 1.0f, 0.0f));

	glm::vec3 moveSideWaysDirection  = glm::mat3(mat_cameraYaw_Y_Axis) * cBasicFlyCamera::m_RIGHT_SIDE_OF_CAMERA;

	// Like this, but "sideways" this->m_eye += (this->m_target * scaledDistance);
	this->m_eye += (moveSideWaysDirection * scaledDistance);


	return;
}

void cBasicFlyCamera::moveUpDown(float distanceToMove)
{
	float scaledDistance = distanceToMove * this->m_movementSpeed;

	// HACK:
	this->m_eye += glm::vec3(0.0f, scaledDistance, 0.0f);
	return;
}
