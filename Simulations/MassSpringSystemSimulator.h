#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change
#define ZERO Vec3(0.0, 0.0, 0.0)

class Point {
public:
	Vec3 Position = ZERO;
	Vec3 Velocity = ZERO;
	Vec3 Force = ZERO;
	Vec3 Acceleration = ZERO;
	bool isFixed = false;

	Point(Vec3 pos, Vec3 vel, Vec3 force, bool fixed) {
		Position = pos;
		Velocity = vel;
		Force = force;
		isFixed = fixed;
	}

	void clearForce() {
		Force = ZERO;
		Acceleration = ZERO;
	}

	std::vector<Point> m_point;

};

class Spring {
public:
	int p1;
	int p2;
	float initialLength;
	Spring(int point1, int point2, float initialLen) {
		p1 = point1;
		p2 = point2;
		initialLength = initialLen;
	}
};

class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);
	void eulerMethodIntergration(float timeStep);
	void midPointMethodIntergration(float timeStep);
	void leapFrogMethodIntegration(double h);
	//void calculateHookesLaw();
	//void halfEuler(double h, std::vector<Vec3>& oldPosArr, std::vector<Vec3>& oldVelArr);
	void printResults(int springIndex);
	void drawPoints();
	void checkCollison();

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

	//void calculateHookesLaw(Point massPoint1, Point massPoint2, Vec3 dist, Vec3 dist_norm, float length);



private:
	// Data Attributes
	std::vector<Point> pointArr;
	std::vector<Spring> springArr;
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	// UI Attributesprint3dVec
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	int   m_iNumPoints;
	float m_fPointSize;
	int   m_iNumSprings;
	bool m_bGravity = false;
	// functions
	void setupTwoPoints();
	void Demo1();
	void Demo2();
	void Demo3();
	void setupTenPoints();
	void Demo4();
};
#endif