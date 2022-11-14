#include "MassSpringSystemSimulator.h"
#include <sstream>
#include <iostream>
#include <cmath>

/*  */
#define FIXED_FLOAT(x) std::fixed <<std::setprecision(4)<<(x)
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
#define ZERO Vec3(0.0, 0.0, 0.0)
//--------------------------------------------------------------------------------------
int previousDemo = -1;
int demo = 1;
Vec3 gravity = ZERO;
//---------------------------------------------------------------------------------------


/**
 * This function is the constructor of the MassSpringSystemSimulator class. It initializes the point
 * size, damping, external force and the integrator.
 */
MassSpringSystemSimulator::MassSpringSystemSimulator() {
	m_fPointSize = 0.05f;
	m_fDamping = 0.0f;
	applyExternalForce(Vec3(0, 0, 0));
	setIntegrator(EULER);
}
//--------------------------------------------------------------------------------------
/**
 * The function takes a Vec3 and a string as input, and outputs the string and the Vec3 to the console
 *
 * @param v The vector to be printed.
 * @param s The name of the vector.
 */
void Vec3dToString(Vec3 v, string s) {
	std::cout << std::fixed;
	std::cout << std::setprecision(4); // Decimal precision is set to determine how floating-point numbers will be formatted during output operations.
	std::cout << "------------------" << std::endl;
	std::cout << s << std::endl;
	std::cout << v.x << " "
		<< v.y << " "
		<< v.z << " "
		<< std::endl;
}
//--------------------------------------------------------------------------------------
/**
 * This function sets up a system of two mass points connected by a spring
 */
void MassSpringSystemSimulator::setupTwoPoints() {
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0.0, 0.0f, 0), Vec3(-1.0, 0.0f, 0), false);
	int p1 = addMassPoint(Vec3(0.0, 2.0f, 0), Vec3(1.0, 0.0f, 0), false);
	addSpring(p0, p1, 1.0);
}
//--------------------------------------------------------------------------------------
/**
 * This function is used to set up the demo1, which is a two point system.
 */
void MassSpringSystemSimulator::Demo1() {
	previousDemo = 1;
	setupTwoPoints();
	m_bGravity = false;
	if (m_iIntegrator == EULER) {
		eulerMethodIntergration(0.1);
		checkCollison();
	}
	else if (m_iIntegrator == MIDPOINT) {
		midPointMethodIntergration(0.1);
	}


}
//--------------------------------------------------------------------------------------
/**
 * This function sets up the system with two points and sets the integrator to Euler. It then calls the
 * eulerMethodIntegration function with a time step of 0.005. It then checks for collisions and sets
 * the previousDemo to 2
 */
void MassSpringSystemSimulator::Demo2() {
	setupTwoPoints();
	setIntegrator(EULER);
	eulerMethodIntergration(0.005);
	checkCollison();
	previousDemo = 2;
}
//--------------------------------------------------------------------------------------
/**
 * This function sets up the system with two points and sets the integrator to Midpoint. It then calls the
 * midPointMethodIntergration function with a time step of 0.1, Then sets the previousDemo to 3.
 */
void MassSpringSystemSimulator::Demo3() {
	setupTwoPoints();
	setIntegrator(MIDPOINT);
	//applyMidpointStep(0.005);
	midPointMethodIntergration(0.1);
	previousDemo = 3;
}
//--------------------------------------------------------------------------------------
/**
 * It creates a system of 10 points connected by springs
 */
void MassSpringSystemSimulator::setupTenPoints() {
	setMass(10.0f);
	setDampingFactor(0.0f);
	setStiffness(40.0f);
	m_bGravity = true;
	setIntegrator(EULER);
	applyExternalForce(Vec3(0, 0, 0));
	int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(0, 0.1, 0), false);
	int p1 = addMassPoint(Vec3(0.2, 0.8f, 0), Vec3(0, -.1, 0), false);
	addSpring(p0, p1, 1.0f);
	int p2 = addMassPoint(Vec3(0.1, 0.3, 0.5), Vec3(0, .1, 0), false);
	addSpring(p0, p2, 0.5f);
	int p3 = addMassPoint(Vec3(0.3, 0.5f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p2, p3, 0.4);
	int p4 = addMassPoint(Vec3(0.5, 0.2f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p3, p4, 0.6);
	int p5 = addMassPoint(Vec3(0.5, 0.4f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p4, p5, 0.8);
	int p6 = addMassPoint(Vec3(0.4, 0.0f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p5, p6, 0.1);
	int p7 = addMassPoint(Vec3(0.6, 0.3f, 0), Vec3(0, 0.1f, 0), false);
	addSpring(p6, p7, 0.2);
	int p8 = addMassPoint(Vec3(0.1, 0.4f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p7, p8, 0.6);
	int p9 = addMassPoint(Vec3(0.7, 0.2f, 0), Vec3(0, 0.0f, 0), false);
	addSpring(p8, p9, 0.8);
	addSpring(p9, p0, 0.5);
}
//--------------------------------------------------------------------------------------

/*This function sets up the system to be a mass - spring system with 10 points, sets the integrator to
* be the leapfrog method, and then integrates the system using the leapfrog method with a time step of
* 0.1
*/
void MassSpringSystemSimulator::Demo4() {
	setupTenPoints();
	setIntegrator(LEAPFROG);
	leapFrogMethodIntegration(0.1);
	previousDemo = 4;
}
//--------------------------------------------------------------------------------------

 /* It returns a string containing the names of the test cases
 *
 * @return The string "Euler, Leapfrog, Midpoint"
 */
const char* MassSpringSystemSimulator::getTestCasesStr() {
	return "Euler, Leapfrog, Midpoint";
}
//--------------------------------------------------------------------------------------
/**
 * This function is called once when the UI is initialized. It is used to add variables to the tweak
 * bar
 *
 * @param DUC A pointer to the DrawingUtilitiesClass object.
 */
void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(DUC->g_pTweakBar, "Demo no.", TW_TYPE_INT32, &demo, "min=1 step=1 max=4");
	TwAddVarRW(DUC->g_pTweakBar, "Point Size", TW_TYPE_FLOAT, &m_fPointSize, "min=0.01 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Apply Gravity", TW_TYPE_BOOLCPP, &m_bGravity, "");
}
//--------------------------------------------------------------------------------------
/**
 * It resets the mouse position, the trackmouse position, the oldtrackmouse position, the external
 * force, the point array, and the spring array
 */
void MassSpringSystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
	m_externalForce = ZERO;
	pointArr.erase(pointArr.begin(), pointArr.end());
	//std::cout << "POINT ARR SIZE " << pointArr.size() << std::endl;
	springArr.erase(springArr.begin(), springArr.end());
	//std::cout << "SPRING ARR SIZE " << springArr.size() << std::endl;
}
//--------------------------------------------------------------------------------------
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
		if (i != 0) {
			if (i == pointArr.size() - 1) {
				DUC->beginLine();
				DUC->drawLine(pointArr[i].Position, Vec3(255.0, 255.0, 255.0), pointArr[0].Position, Vec3(125.0, 125.0, 125.0));
				DUC->endLine();
			}
			else {
				DUC->beginLine();
				DUC->drawLine(pointArr[i].Position, Vec3(255.0, 255.0, 255.0), pointArr[i - 1].Position, Vec3(125.0, 125.0, 125.0));
				DUC->endLine();
			}
		}
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(randCol(eng), randCol(eng), randCol(eng)));
		DUC->drawSphere(pointArr[i].Position, Vec3(m_fPointSize, m_fPointSize, m_fPointSize));

	}
}
//--------------------------------------------------------------------------------------
/**
 * This function is called every frame, and it's responsible for drawing the points of the mass-spring
 * system
 *
 * @param pd3dImmediateContext The immediate context of the device.
 */
void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	if (previousDemo != demo) {
		reset();
		switch (demo) {
		case 1:
			Demo1();
			break;
		case 2:
			Demo2();
			break;
		case 3:
			Demo3();
			break;
		case 4:
			Demo4();
			break;
		}
	}
	drawPoints();


}
//--------------------------------------------------------------------------------------
/**
 * This function is called when the user changes the test case. It sets the integrator to the one
 * selected by the user
 *
 * @param testCase The test case number that you want to run.
 */
void MassSpringSystemSimulator::notifyCaseChanged(int testCase)
{
	m_iIntegrator = testCase;
	setIntegrator(m_iIntegrator);
	switch (m_iIntegrator)
	{
	case 0:
		cout << "Euler was used\n";
		break;
	case 1:
		cout << "Leapfrog was used\n";
		break;
	case 2:
		cout << "Midpoint was used\n";
		break;
	default:
		cout << "Test Empty\n";
		break;
	}
}
//--------------------------------------------------------------------------------------
/**
 * The function is responsible for applying the mouse deltas to the movable object's position
 *
 * @param timeElapsed The time elapsed since the last time this function was called.
 */
void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
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
		float inputScale = 0.001f;
		inputWorld = inputWorld * inputScale;
		// TODO: FIX User interaction
		for (int i = 0; i < getNumberOfMassPoints(); i++)
		{
			pointArr[i].Position += inputScale * (float)m_trackmouse.x * inputWorld;
			pointArr[i].Position += -inputScale * (float)m_trackmouse.y * inputWorld;
		}
	}
}
//--------------------------------------------------------------------------------------
/**
 * This function is used to simulate the time step of the system
 *
 * @param timeStep The time step to simulate.
 */
void MassSpringSystemSimulator::simulateTimestep(float timeStep)
{
	// update current setup for each frame
	switch (m_iIntegrator)
	{// handling different cases
	case EULER:
		eulerMethodIntergration(timeStep);
		checkCollison();
		break;
	case LEAPFROG:
		/*applyLeapfrogStep(timeStep);*/
		break;
	case MIDPOINT:
		midPointMethodIntergration(timeStep);
		break;
	default:
		break;
	}
}
//--------------------------------------------------------------------------------------
/**
 * This function is called when the user presses the left mouse button down. It sets the m_trackmouse.x
 * and m_trackmouse.y to the current mouse position
 *
 * @param x x coordinate of the mouse click
 * @param y The y coordinate of the mouse click.
 */
void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
//--------------------------------------------------------------------------------------
/**
 * *|MARKER_CURSOR|*
 *
 * @param x x-coordinate of the mouse
 * @param y The y coordinate of the mouse.
 */
void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}
//--------------------------------------------------------------------------------------
/**
 *
 *
 * @param mass The mass of the particle.
 */
void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}
//--------------------------------------------------------------------------------------
/**
 * *|MARKER_CURSOR|*
 *
 * @param stiffness the stiffness of the springs
 */
void MassSpringSystemSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}
//--------------------------------------------------------------------------------------
/**
 * This function sets the damping factor of the system
 *
 * @param damping The damping factor of the system.
 */
void MassSpringSystemSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}
//--------------------------------------------------------------------------------------
/**
 * This function adds a mass point to the system
 *
 * @param position The position of the mass point.
 * @param Velocity The initial velocity of the mass point.
 * @param isFixed if true, the mass point will not move.
 *
 * @return The index of the last mass point added to the system.
 */
int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	Point p = Point(position, Velocity, ZERO, isFixed);

	pointArr.push_back(p);

	return getNumberOfMassPoints() - 1;
}
//--------------------------------------------------------------------------------------
/**
 * *|MARKER_CURSOR|*
 *
 * @param masspoint1 The index of the first mass point in the mass point array.
 * @param masspoint2 The index of the second mass point.
 * @param initialLength the initial length of the spring
 */
void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	Spring s = Spring(masspoint1, masspoint2, initialLength);
	springArr.push_back(s);
}
//--------------------------------------------------------------------------------------
/**
 *
 *
 * @return The number of mass points in the system.
 */
int MassSpringSystemSimulator::getNumberOfMassPoints()
{
	return pointArr.size();
}
//--------------------------------------------------------------------------------------
/**
 *
 *
 * @return The number of springs in the system.
 */
int MassSpringSystemSimulator::getNumberOfSprings()
{
	return springArr.size();
}
//--------------------------------------------------------------------------------------
/**
 * *|CURSOR_MARCADOR|*
 *
 * @param index the index of the mass point
 *
 * @return The position of the mass point.
 */
Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index)
{
	return pointArr[index].Position;
}
//--------------------------------------------------------------------------------------
/**
 *
 *
 * @param index the index of the mass point
 *
 * @return The velocity of the mass point.
 */
Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index)
{
	return pointArr[index].Velocity;
}
//--------------------------------------------------------------------------------------
/**
 * This function adds the force to the external force
 *
 * @param force The force to be applied to the system.
 */
void MassSpringSystemSimulator::applyExternalForce(Vec3 force)
{
	m_externalForce += force;
}
//--------------------------------------------------------------------------------------
/**
 * If the y-coordinate of a mass point is less than -1, then set the y-coordinate to -1 and reverse the
 * y-velocity
 */
void MassSpringSystemSimulator::checkCollison() {
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		if (pointArr[i].Position.y < -1) {
			pointArr[i].Position.y = -1.0f;
			pointArr[i].Velocity.y = std::abs(pointArr[i].Velocity.y);
		}
	}
}
//--------------------------------------------------------------------------------------
/**
 * The function calculates the euler method integration by first calculating the force of each spring and adds it to the force of the mass points. Then
 * it calculates the acceleration of each mass point and updates the position and velocity of each mass
 * point
 *
 * @param timeStep The time step of the simulation.
 */
void MassSpringSystemSimulator::eulerMethodIntergration(float timeStep) {
	Point p1 = Point(ZERO, ZERO, ZERO, false);
	Point p2 = Point(ZERO, ZERO, ZERO, false);
	Vec3 dist, dist_norm = ZERO;
	float length = 0;
	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
		if (m_bGravity) {
			gravity = Vec3(0.0, -10.0f, 0.0);
		}
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		p1 = pointArr[springArr[i].p1];
		p2 = pointArr[springArr[i].p2];

		dist = p1.Position - p2.Position;
		length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
		printResults(i);
	}

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].Acceleration = (pointArr[i].Force / m_fMass) + gravity + m_externalForce - m_fDamping * pointArr[i].Velocity;

		pointArr[i].Position = pointArr[i].Position + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = pointArr[i].Velocity + (timeStep * pointArr[i].Acceleration);
	}

}
//--------------------------------------------------------------------------------------
/**
 * The function calculates the midpoint method of integration by first calculating the euler method of
 * integration for half the time step, then calculating the forces and acceleration for the new
 * position and velocity, and finally calculating the new position and velocity
 *
 * @param timeStep the time step
 */
void MassSpringSystemSimulator::midPointMethodIntergration(float timeStep)
{
	Point p1 = Point(ZERO, ZERO, ZERO, false);
	Point p2 = Point(ZERO, ZERO, ZERO, false);
	Vec3 dist, dist_norm, dist_m, dist_m_norm,
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

	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].clearForce();
		if (m_bGravity) {
			gravity = Vec3(0.0, -10.0f, 0.0);
		}
	}

	for (int i = 0; i < getNumberOfSprings(); i++) {
		p1 = pointArr[springArr[i].p1];
		p2 = pointArr[springArr[i].p2];

		dist = p1.Position - p2.Position;
		length = std::sqrt(p1.Position.squaredDistanceTo(p2.Position));
		dist_norm = dist / length;

		pointArr[springArr[i].p1].Force += -m_fStiffness * (length - springArr[i].initialLength) * dist_norm;
		pointArr[springArr[i].p2].Force += -1 * pointArr[springArr[i].p1].Force;
		printResults(i);
	}


	for (int i = 0; i < getNumberOfMassPoints(); i++) {
		pointArr[i].Acceleration = (pointArr[i].Force / m_fMass) + gravity + m_externalForce - m_fDamping * pointArr[i].Velocity;;

		pointArr[i].Position = oldPosArr[i] + (timeStep * pointArr[i].Velocity);

		pointArr[i].Velocity = oldVelArr[i] + (timeStep * pointArr[i].Acceleration);
	}

	checkCollison();
}
//--------------------------------------------------------------------------------------
void MassSpringSystemSimulator::leapFrogMethodIntegration(double h)
{

}
//--------------------------------------------------------------------------------------
/**
 * This function prints the position, velocity, and force of the two points connected by the spring
 *
 * @param springIndex The index of the spring in the spring array.
 */
void MassSpringSystemSimulator::printResults(int springIndex)
{
	std::cout << "\nSpring #" << springIndex << std::endl;
	std::cout << std::fixed;
	std::cout << std::setprecision(4);
	Vec3 p1 = pointArr[springArr[springIndex].p1].Position;
	Vec3 p2 = pointArr[springArr[springIndex].p2].Position;
	Vec3 v1 = pointArr[springArr[springIndex].p1].Velocity;
	Vec3 v2 = pointArr[springArr[springIndex].p2].Velocity;
	Vec3 f1 = pointArr[springArr[springIndex].p1].Force;
	Vec3 f2 = pointArr[springArr[springIndex].p2].Force;
	//Vec3dToString(pointArr[springArr[springIndex].p1].Position, "Position p1");
	//Vec3dToString(pointArr[springArr[springIndex].p2].Position, "Position p2");
	//Vec3dToString(pointArr[springArr[springIndex].p1].Velocity, "Velocity p1");
	//Vec3dToString(pointArr[springArr[springIndex].p2].Velocity, "Velocity p2");

	//std::cout << (pointArr[springArr[springIndex].p2].Velocity, "Velocity p2") << std::endl;
	//std::cout << (pointArr[springArr[springIndex].p1].Position, "Position p1") << std::endl;
	std::cout << "Position p1 " << p1.x << " " << p1.y << " " << p1.z << " " << std::endl;
	std::cout << "Position p2 " << p2.x << " " << p2.y << " " << p2.z << " " << std::endl;
	std::cout << "Velocity p1 " << v1.x << " " << v1.y << " " << v1.z << " " << std::endl;
	std::cout << "Velocity p2 " << v2.x << " " << v2.y << " " << v2.z << " " << std::endl;
	std::cout << "Force p1 " << f1.x << " " << f1.y << " " << f1.z << " " << std::endl;
	std::cout << "Force p2 " << f2.x << " " << f2.y << " " << f2.z << " " << std::endl;
	std::cout << "*----------------------------------------------------------------------------------------*" <<
		std::endl;

}

