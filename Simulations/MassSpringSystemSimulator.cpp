#include "MassSpringSystemSimulator.h"
#include <sstream>
#include <iostream>
#include <cmath>

/**
 * This function is the constructor of the MassSpringSystemSimulator class. It initializes the point
 * size, damping, external force and the integrator.
 */
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fPointSize = 0.05f;
	m_fDamping = 1.0f;
	applyExternalForce(Vec3(0, 0, 0));
	setIntegrator(EULER);
}

/**
 * This function sets up a system of two mass points connected by a spring
 */
void MassSpringSystemSimulator::setupTwoPoints() {
	reset();
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);
}


/**
 * It creates a system of 10 points connected by springs
*/
void MassSpringSystemSimulator::setupTenPoints() {
	reset();
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	m_bGravity = false;
	applyExternalForce(Vec3(0, 0, 0));
	// front 4
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	int p1 = addMassPoint(Vec3(0.2, 0, 0), Vec3(0, 0, 0), false);
	addSpring(p0, p1, 0.2);
	int p2 = addMassPoint(Vec3(0, 0.2, 0), Vec3(0, 0, 0), false);
	addSpring(p0, p2, 0.2);
	int p3 = addMassPoint(Vec3(0.2, 0.2, 0), Vec3(0, 0, 0), false);
	addSpring(p2, p3, 0.2);
	addSpring(p3, p1, 0.2);
	// back 4
	int p4 = addMassPoint(Vec3(0, 0, -0.2), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(0.2, 0, -0.2), Vec3(0, 0, 0), false);
	addSpring(p4, p5, 0.2);
	int p6 = addMassPoint(Vec3(0, 0.2, -0.2), Vec3(0, 0, 0), false);
	addSpring(p4, p6, 0.2);
	int p7 = addMassPoint(Vec3(0.2, 0.2, -0.2), Vec3(0, 0, 0), false);
	addSpring(p6, p7, 0.2);
	addSpring(p7, p5, 0.2);
	// connect
	addSpring(p0, p4, 0.2);
	addSpring(p1, p5, 0.2);
	addSpring(p2, p6, 0.2);
	addSpring(p3, p7, 0.2);
	// the sides
	int p8 = addMassPoint(Vec3(-0.1, 0.1, -0.1), Vec3(0, 0, 0), false);
	int p9 = addMassPoint(Vec3(0.3, 0.1, -0.1), Vec3(0, 0, 0), false);
	addSpring(p8, p0, 0.2);
	addSpring(p8, p2, 0.2);
	addSpring(p8, p4, 0.2);
	addSpring(p8, p6, 0.2);
	addSpring(p9, p1, 0.2);
	addSpring(p9, p3, 0.2);
	addSpring(p9, p5, 0.2);
	addSpring(p9, p7, 0.2);
	if (m_iIntegrator == LEAPFROG) {
		setupLeapFrogMethod();
	}
	if (m_iIntegrator == EULER) {
		eulerMethodIntergration(0.1f);
	}
	if (m_iIntegrator == MIDPOINT) {
		midPointMethodIntergration(0.1f);
	}

}

/**
 * This function sets up a cube-like structure with 8 mass points and 12 springs
 */
void MassSpringSystemSimulator::setupCubelike() {
	reset();
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(10.0f);
	m_bGravity = false;
	applyExternalForce(Vec3(0, 0, 0));

	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
	int p1 = addMassPoint(Vec3(1, 0, 0), Vec3(0, 0, 0), false);
	int p2 = addMassPoint(Vec3(1, 1, 0), Vec3(0, 0, 0), false);
	int p3 = addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), false);
	int p4 = addMassPoint(Vec3(0, 0, 1), Vec3(0, 0, 0), false);
	int p5 = addMassPoint(Vec3(1, 0, 1), Vec3(0, 0, 0), false);
	int p6 = addMassPoint(Vec3(1, 1, 1), Vec3(0, 0, 0), false);
	int p7 = addMassPoint(Vec3(0, 1, 1), Vec3(0, 0, 0), false);


	addSpring(p0, p1, 0.5f);
	addSpring(p1, p2, 0.5f);
	addSpring(p2, p3, 0.5f);
	addSpring(p3, p0, 0.5f);

	addSpring(p4, p5, 0.5f);
	addSpring(p5, p6, 0.5f);
	addSpring(p6, p7, 0.5f);
	addSpring(p7, p4, 0.5f);

	addSpring(p0, p4, 0.5f);
	addSpring(p1, p5, 0.5f);
	addSpring(p2, p6, 0.5f);
	addSpring(p3, p7, 0.5f);

	/*addSpring(p0, p6, sqrtf(3));
	addSpring(p1, p7, sqrtf(3));
	addSpring(p2, p4, sqrtf(3));
	addSpring(p3, p5, sqrtf(3));*/

	if (m_iIntegrator == LEAPFROG) {
		setupLeapFrogMethod();
	}
}

/**
 * This function is used to run the demo1, which is a two point system.
 */
void MassSpringSystemSimulator::Demo1() {
	setupTwoPoints();
	eulerMethodIntergration(0.1);
	std::cout << std::endl;
	std::cout << "After an Euler step: " << std::endl;
	printResults(0);
	std::cout << std::endl;

	setupTwoPoints();
	midPointMethodIntergration(0.1);
	std::cout << "After a midpoint step: " << std::endl;
	printResults(0);
	std::cout << std::endl;


	setupTwoPoints();
	setupLeapFrogMethod();
	leapfrogMethodIntegration(0.1);
	std::cout << "After a leapfrog step: " << std::endl;
	printResults(0);
	std::cout << std::endl;
}

/**
 * This function sets up the system with two points and sets the integrator to Euler
 */
void MassSpringSystemSimulator::Demo2() {
	setIntegrator(EULER);
	setupTwoPoints();
}

/*
* This function sets up the system with two points and sets the integrator to Midpoint
*/
void MassSpringSystemSimulator::Demo3() {
	setIntegrator(MIDPOINT);
	setupTwoPoints();
}

/*
* This function sets up the system to be a mass - spring system with 10 points
*/
void MassSpringSystemSimulator::Demo4() {
	setupTenPoints();
	//setupCubelike();
}

/* It returns a string containing the names of the test cases
*
*  @return The string "Demo1, Demo2, Demo3, Demo4"
*/
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Demo1, Demo2, Demo3, Demo4";
}

/* It returns a string containing the names of the integration cases
*
*  @return The string "Euler, LeapFrog, Midpoint"
*/
const char* MassSpringSystemSimulator::getIntegratorsStr() {
	return "Euler, LeapFrog, Midpoint";
}

/**
 * This function is called once when the UI is initialized. It is used to add variables to the tweak
 * bar
 *
 * the parameter DUC is a pointer to the DrawingUtilitiesClass object.
 */
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	if (m_iCurrentDemo != 0) {
		TwType TW_TYPE_INTEGRATOR = TwDefineEnumFromString("Integrator", getIntegratorsStr());
		TwAddVarRW(DUC->g_pTweakBar, "Integrator", TW_TYPE_INTEGRATOR, &m_iIntegrator, "");
		TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_fPointSize, "min=0.01 step=0.01");
		TwAddVarRW(DUC->g_pTweakBar, "Apply Gravity", TW_TYPE_BOOLCPP, &m_bGravity, "");
		TwAddVarRW(DUC->g_pTweakBar, "Damping", TW_TYPE_FLOAT, &m_fDamping, "min=0.0 step=0.2");
		TwAddVarRW(DUC->g_pTweakBar, "Stiffness", TW_TYPE_FLOAT, &m_fStiffness, "min=0.0 step=10.0");
	}
}

/**
 * It resets the current scene
 */
void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_externalForce = ZERO;
	pointArr.erase(pointArr.begin(), pointArr.end());
	springArr.erase(springArr.begin(), springArr.end());
}

/**
 * It draws the points and the lines between them
 */
void MassSpringSystemSimulator::drawPoints() {
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(0.0f, 1.0f);
	std::uniform_real_distribution<float> randPos(-0.5f, 0.5f);
	// m_iNumPoints
	for (int i = 0; i < pointArr.size(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(pointArr[i].Position, Vec3(m_fPointSize, m_fPointSize, m_fPointSize));
	}
	for (int i = 0; i < springArr.size(); i++) {
		Spring spring = springArr.at(i);
		Point p1 = pointArr.at(spring.p1);
		Point p2 = pointArr.at(spring.p2);
		bool springIsInitialSize = spring.initialLength < sqrtf(p1.Position.squaredDistanceTo(p2.Position)) + 0.0001;
		DUC->beginLine();
		DUC->drawLine(p1.Position, Vec3(255.0, 255.0, 255.0), p2.Position, Vec3(125.0, 125.0, 125.0));
		DUC->endLine();
	}
}

/**
 * This function is called every frame, and it's responsible for drawing the points of the mass-spring
 * system
 * 
 */
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (m_iIntegrator != m_iOldIntegrator) {
		m_bLeapFrogIsSetup = false;
		m_iOldIntegrator = m_iIntegrator;
	}
	if (m_iCurrentDemo != 0) {
		drawPoints();
	}
}

/**
 * This function is called when the user changes the test case (the demo)

 */
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iCurrentDemo = testCase;
	m_bLeapFrogIsSetup = false;
	switch (m_iCurrentDemo)
	{
	case 0:
		std::cout << "Demo1: " << std::endl;
		std::cout << "Results: " << std::endl;
		Demo1();
		break;
	case 1:
		std::cout << "Demo2: " << std::endl;
		Demo2();
		break;
	case 2:
		std::cout << "Demo3: " << std::endl;
		Demo3();
		break;
	case 3:
		std::cout << "Demo4: " << std::endl;
		Demo4();
		break;
	default:
		cout << "Test Empty\n";
		break;
	}
}

/*
The function is responsible for applying the mouse deltas to the movable object's position
*/
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.5f;
		inputWorld = inputWorld * inputScale;
		applyExternalForce(inputWorld);
	}
}

/**
 * This function is used to simulate the time step of the system
 */
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	m_fLastTimeStep = timeStep;
	switch (m_iIntegrator) {
	case EULER:
		eulerMethodIntergration(timeStep);
		return;
	case LEAPFROG:
		leapfrogMethodIntegration(timeStep);
		return;
	case MIDPOINT:
		midPointMethodIntergration(timeStep);
		return;
	}
}

/**
 * This function is called when the user presses the left mouse button down. It sets the m_trackmouse.x
 * and m_trackmouse.y to the current mouse position
 *
 * The parameter x: is the x coordinate of the mouse click
 * The parameter y: is the y coordinate of the mouse click.
 */
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

/**
 *
 *
 * The parameter mass: is the mass of the particle.
 */
void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

/**
 * The parameter stiffness: is the stiffness of the springs
 */
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

/**
 * This function sets the damping factor of the system
 *
 * The parameter damping: is the damping factor of the system.
 */
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

/**
 * This function adds a mass point to the system
 *
 * The parameter position: is the position of the mass point.
 * The parameter Velocity: is the initial velocity of the mass point.
 * The parameter isFixed:  if true, the mass point will not move.
 *
 * @return The index of the last mass point added to the system.
 */
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	Point p = Point(position, Velocity, ZERO, isFixed);

	pointArr.push_back(p);

	return getNumberOfMassPoints() - 1;
}

/**
 * The parameter masspoint1: is the index of the first mass point in the mass point array.
 * The parameter masspoint2: is the index of the second mass point.
 * The parameter initialLength: is the initial length of the spring
 */
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring(masspoint1, masspoint2, initialLength);
	springArr.push_back(s);
}

/**

 * @return The number of mass points in the system.
 */
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointArr.size();
}

/**
 * @return The number of springs in the system.
 */
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springArr.size();
}

/**
 * The parameter index: is the index of the mass point
 *
 * The parameter Position: is the position of the mass point.
 */
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return pointArr[index].Position;
}

/**
 *
 *
 * The parameter index: the index of the mass point
 *
 * @return The velocity of the mass point.
 */
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return pointArr[index].Velocity;
}

/**
 * This function adds the force to the external force
 *
 *The parameter force: is the force to be applied to the system.
 */
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce = force;
	m_iexternalForceLifetime = 10;
}

/**
 * If the y-coordinate of a mass point is less than -1, then set the y-coordinate to -1 and reverse the
 * y-velocity
 */
Vec3 MassSpringSystemSimulator::getCollisonAcceleration(int index, float timeStep) {
	if (pointArr[index].Position.y < -1) {
		return (Vec3(pointArr[index].Velocity.x, std::abs(pointArr[index].Velocity.y) * 0.7, pointArr[index].Velocity.z) - pointArr[index].Velocity) / timeStep;
	}
	return ZERO;
}

/**
 * The function calculates the euler method integration by first calculating the force of each spring and adds it to the force of the mass points. Then
 * it calculates the acceleration of each mass point and updates the position and velocity of each mass
 * point
.
 */
void MassSpringSystemSimulator::eulerMethodIntergration(float timeStep) {
	Vec3 gravity = ZERO;
	if (m_bGravity) {
		gravity = Vec3(0.0, -10.0f, 0.0);
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		Point p1 = pointArr[springArr[i].p1];
		Point p2 = pointArr[springArr[i].p2];

		Vec3 dist = p1.Position - p2.Position;
		float length = sqrtf(powf(dist.x, 2) + powf(dist.y, 2) + powf(dist.z, 2));
		Vec3 dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -1 * m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}
		pointArr[i].Acceleration = (pointArr[i].Force + m_externalForce) / m_fMass + getCollisonAcceleration(i, timeStep) + gravity - m_fDamping * pointArr[i].Velocity;

		pointArr[i].Position = pointArr[i].Position + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = pointArr[i].Velocity + (timeStep * pointArr[i].Acceleration);
	}

	if (m_iexternalForceLifetime >= 0) {
		m_iexternalForceLifetime--;
	}
	else {
		m_externalForce = ZERO;
	}
}

/**
 * The function calculates the midpoint method of integration by first calculating the euler method of
 * integration for half the time step, then calculating the forces and acceleration for the new
 * position and velocity, and finally calculating the new position and velocity

 */
void MassSpringSystemSimulator::midPointMethodIntergration(float timeStep)
{
	Point p1 = Point(ZERO, ZERO, ZERO, false);
	Point p2 = Point(ZERO, ZERO, ZERO, false);
	Vec3 gravity, dist, dist_norm, dist_m, dist_m_norm,
		p1vm, p2vm, p1fm, p2fm,
		p1am, p2am, p1xm, p2xm = ZERO;
	float length, length_mid = 0;

	std::vector<Vec3> oldPosArr;
	std::vector<Vec3> oldVelArr;

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		oldPosArr.push_back(pointArr[i].Position);
		oldVelArr.push_back(pointArr[i].Velocity);
	}

	eulerMethodIntergration(timeStep / 2);

	if (m_bGravity) {
		gravity = Vec3(0.0, -10.0f, 0.0);
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		p1 = pointArr[springArr[i].p1];
		p2 = pointArr[springArr[i].p2];

		dist = p1.Position - p2.Position;
		length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}


	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}
		pointArr[i].Acceleration = (pointArr[i].Force + m_externalForce) / m_fMass + getCollisonAcceleration(i, timeStep) + gravity - m_fDamping * pointArr[i].Velocity;

		pointArr[i].Position = oldPosArr[i] + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = oldVelArr[i] + (timeStep * pointArr[i].Acceleration);
	}


	if (m_iexternalForceLifetime >= 0) {
		m_iexternalForceLifetime--;
	}
	else {
		m_externalForce = ZERO;
	}
}

/**
 * The function sets up the environment for the leapfrog integration by stepping each masspoint's velocity back half a timestep
 */
void MassSpringSystemSimulator::setupLeapFrogMethod() {
	Vec3 gravity = ZERO;
	if (m_bGravity) {
		gravity = Vec3(0.0, -10.0f, 0.0);
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		Point p1 = pointArr[springArr[i].p1];
		Point p2 = pointArr[springArr[i].p2];

		Vec3 dist = p1.Position - p2.Position;
		float length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		Vec3 dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}
		pointArr[i].Acceleration = (pointArr[i].Force + m_externalForce) / m_fMass + getCollisonAcceleration(i, m_fLastTimeStep) + gravity - m_fDamping * pointArr[i].Velocity;
		Vec3 midpointVelocity = pointArr[i].Velocity + (m_fLastTimeStep / 2 * pointArr[i].Acceleration);
		pointArr[i].Velocity = -(pointArr[i].Velocity - (pointArr[i].Velocity - midpointVelocity));
	}
	m_bLeapFrogIsSetup = true;
}

/**
 * The function implements the leapfrog integration

 */
void MassSpringSystemSimulator::leapfrogMethodIntegration(float timeStep) {
	if (!m_bLeapFrogIsSetup) {
		setupLeapFrogMethod();
	}

	Vec3 gravity = ZERO;
	if (m_bGravity) {
		gravity = Vec3(0.0, -10.0f, 0.0);
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		Point p1 = pointArr[springArr[i].p1];
		Point p2 = pointArr[springArr[i].p2];

		Vec3 dist = p1.Position - p2.Position;
		float length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		Vec3 dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].isFixed) {
			continue;
		}
		pointArr[i].Acceleration = (pointArr[i].Force + m_externalForce) / m_fMass + getCollisonAcceleration(i, timeStep) + gravity - m_fDamping * pointArr[i].Velocity;
		pointArr[i].Velocity = pointArr[i].Velocity + (timeStep * pointArr[i].Acceleration);

		pointArr[i].Position = pointArr[i].Position + (timeStep * pointArr[i].Velocity);
	}

	if (m_iexternalForceLifetime >= 0) {
		m_iexternalForceLifetime--;
	}
	else {
		m_externalForce = ZERO;
	}
}

/**
 * This function prints the position, velocity, and force of the two points connected by the spring
 *
 * The parameter springIndex: is the index of the spring in the spring array.
 */
void MassSpringSystemSimulator::printResults(int springIndex)
{
	std::cout << "Spring #" << springIndex << std::endl;
	std::cout << std::fixed;
	std::cout << std::setprecision(4);
	Vec3 p1 = pointArr[springArr[springIndex].p1].Position;
	Vec3 p2 = pointArr[springArr[springIndex].p2].Position;
	Vec3 v1 = pointArr[springArr[springIndex].p1].Velocity;
	Vec3 v2 = pointArr[springArr[springIndex].p2].Velocity;
	Vec3 f1 = pointArr[springArr[springIndex].p1].Force;
	Vec3 f2 = pointArr[springArr[springIndex].p2].Force;

	std::cout << "Position p1 " << p1.x << " " << p1.y << " " << p1.z << " " << std::endl;
	std::cout << "Position p2 " << p2.x << " " << p2.y << " " << p2.z << " " << std::endl;
	std::cout << "Velocity p1 " << v1.x << " " << v1.y << " " << v1.z << " " << std::endl;
	std::cout << "Velocity p2 " << v2.x << " " << v2.y << " " << v2.z << " " << std::endl;
	std::cout << "Force p1 " << f1.x << " " << f1.y << " " << f1.z << " " << std::endl;
	std::cout << "Force p2 " << f2.x << " " << f2.y << " " << f2.z << " " << std::endl;
}

