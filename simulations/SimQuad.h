#pragma once

#include <windows.h>
#include <math.h>
#include <stdio.h>

const double DEG_TO_RAD = 0.01745329252;
const double RAD_TO_DEG = 57.29577951308;
const double PI = 3.14159265359;
const int numParticles = 40;
//const double mass_weight = 0.0064;	//6.4 gr o 0.0064 kg por cada eslabón de la cuerda (0.0064 * 39 = 0.250 kg)	
//const double mass_weight = 0.0033;	//3.3 gr o 0.0033 kg por cada eslabón de la cuerda (0.0033 * 39 = 0.1287 kg)
const double mass_weight = 0.001;	//1 gr o 0.001 kg por cada eslabón de la cuerda (0.001 * 40 = 0.04 kg)
//0.004 diameter rope ->http://www.samsonropecatalogs.com/lg_display.cfm/catalog/Industrial_Product_Guide_2017/page/6
const double epsilon = 0.001; //0.001;
const int SHARED_NAME_SIZE = 256;
const double maxPossible_dt = 0.00025; //0.0005;		// Maximum Possible dt is (0.5 ms) 0.0005 Seconds
											// This Is Needed To Prevent Pass Over Of A Non-Precise dt Value
const int maxNumQuads = 10;
const int maxNumRopes = 10;

const int maxNumObs = 10;

const int n_mostrar = 1000;

//% Setear la longitud más grande de un barrido laser
const int LaserMaxRange = 20;
const int MinDist = 1;							//Distancia para fuerza máxima
const int res_ang1 = 1;							//%resolución de 1 grado en XY
const int res_ang2 = 2;							//%resolución de 2 grado en Z
const int reg_ang1 = 360;						//%región de 360 grados en XY
const int reg_ang2 = 60;						//%región de 60 grados en Z
const float theta1 = res_ang1 * (float)DEG_TO_RAD;
const float theta2 = res_ang2 * (float)DEG_TO_RAD;

///ESTRUCTURAS QUE FORMAN LA MEMORIA VIRTUAL//////
#define M_COMPARTIDA_QUAD "SIM_QUAD_COPTER_1"
#define M_COMPARTIDA_LASER "SIM_QUAD_LASER_1"
#define M_COMPARTIDA_COM  "SIM_QUAD_COM_1"
#define M_COMPARTIDA_ROPE "SIM_QUAD_ROPE_1"
#define M_COMPARTIDA_SIM "SIM_QUAD_SIM_1"
#define M_COMPARTIDA_LOAD "SIM_QUAD_LOAD_1"

// A better solution for C++
/*class Locker
{
public:
	Locker(CRITICAL_SECTION& cs) : m_cs(cs)
	{
		EnterCriticalSection(&m_cs);
	}
	~Locker()
	{
		LeaveCriticalSection(&m_cs);
	}
private:
	CRITICAL_SECTION&  m_cs;
};*/

// class Vector3D		---> An object to represent a 3D vector or a 3D point in space
class Vector3D
{
public:
	double x;									// the x value of this Vector3D
	double y;									// the y value of this Vector3D
	double z;									// the z value of this Vector3D

	Vector3D()									// Constructor to set x = y = z = 0
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
	}

	Vector3D(double x, double y, double z)			// Constructor that initializes this Vector3D to the intended values of x, y and z
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Vector3D& operator= (Vector3D v)			// operator= sets values of v to this Vector3D. example: v1 = v2 means that values of v2 are set onto v1
	{
		x = v.x;
		y = v.y;
		z = v.z;
		return *this;
	}

	Vector3D operator+ (Vector3D v)				// operator+ is used to add two Vector3D's. operator+ returns a new Vector3D
	{
		return Vector3D(x + v.x, y + v.y, z + v.z);
	}

	Vector3D operator- (Vector3D v)				// operator- is used to take difference of two Vector3D's. operator- returns a new Vector3D
	{
		return Vector3D(x - v.x, y - v.y, z - v.z);
	}

	Vector3D operator* (double value)			// operator* is used to scale a Vector3D by a value. This value multiplies the Vector3D's x, y and z.
	{
		return Vector3D(x * value, y * value, z * value);
	}

	Vector3D operator/ (double value)			// operator/ is used to scale a Vector3D by a value. This value divides the Vector3D's x, y and z.
	{
		return Vector3D(x / value, y / value, z / value);
	}

	Vector3D& operator+= (Vector3D v)			// operator+= is used to add another Vector3D to this Vector3D.
	{
		x += v.x;
		y += v.y;
		z += v.z;
		return *this;
	}

	Vector3D& operator-= (Vector3D v)			// operator-= is used to subtract another Vector3D from this Vector3D.
	{
		x -= v.x;
		y -= v.y;
		z -= v.z;
		return *this;
	}

	Vector3D& operator*= (double value)			// operator*= is used to scale this Vector3D by a value.
	{
		x *= value;
		y *= value;
		z *= value;
		return *this;
	}

	Vector3D& operator/= (double value)			// operator/= is used to scale this Vector3D by a value.
	{
		x /= value;
		y /= value;
		z /= value;
		return *this;
	}


	Vector3D operator- ()						// operator- is used to set this Vector3D's x, y, and z to the negative of them.
	{
		return Vector3D(-x, -y, -z);
	}

	double length()								// length() returns the length of this Vector3D
	{
		return sqrt(x*x + y * y + z * z);
	};

	void unitize()								// unitize() normalizes this Vector3D that its direction remains the same but its length is 1.
	{
		double length = this->length();

		if (length == 0)
			return;

		x /= length;
		y /= length;
		z /= length;
	}

	Vector3D unit()								// unit() returns a new Vector3D. The returned value is a unitized version of this Vector3D.
	{
		double length = this->length();

		if (length == 0)
			return *this;

		return Vector3D(x / length, y / length, z / length);
	}

};

// class Mass			---> An object to represent a mass
class Mass
{
public:
	double m;									// The mass value
	Vector3D pos;								// Position in space
	Vector3D vel;								// Velocity
	Vector3D force;								// Force applied on this mass at an instance

	Mass(double m)								// Constructor
	{
		this->m = m;
	}

	/*
	void applyForce(Vector3D force) method is used to add external force to the mass.
	At an instance in time, several sources of force might affect the mass. The vector sum
	of these forces make up the net force applied to the mass at the instance.
	*/
	void applyForce(Vector3D force)
	{
		this->force += force;					// The external force is added to the force of the mass
	}

	/*
	void init() method sets the force values to zero
	*/
	void init()
	{
		force.x = 0.0;
		force.y = 0.0;
		force.z = 0.0;
	}

	/*
	void simulate(double dt) method calculates the new velocity and new position of
	the mass according to change in time (dt). Here, a simulation method called
	"The Euler Method" is used. The Euler Method is not always accurate, but it is
	simple. It is suitable for most of physical simulations that we know in common
	computer and video games.
	*/
	void simulate(double dt)
	{
		vel += (force / m) * dt;				// Change in velocity is added to the velocity.
												// The change is proportinal with the acceleration (force / m) and change in time

		pos += vel * dt;						// Change in position is added to the position.
												// Change in position is velocity times the change in time
	}

};

class Spring											//An object to represent a spring with inner friction binding two masses. The spring 
														//has a normal length (the length that the spring does not exert any force)
{
public:
	Mass* mass1;										//The first mass at one tip of the spring
	Mass* mass2;										//The second mass at the other tip of the spring

	double springConstant;								//A constant to represent the stiffness of the spring
	double springLength;									//The length that the spring does not exert any force
	double frictionConstant;								//A constant to be used for the inner friction of the spring

	Spring(Mass* mass1, Mass* mass2,
		double springConstant, double springLength, double frictionConstant)		//Constructor
	{
		this->springConstant = springConstant;									//set the springConstant
		this->springLength = springLength;										//set the springLength
		this->frictionConstant = frictionConstant;								//set the frictionConstant

		this->mass1 = mass1;													//set mass1
		this->mass2 = mass2;													//set mass2
	}

	void solve()																	//solve() method: the method where forces can be applied
	{
		Vector3D springVector = mass1->pos - mass2->pos;							//vector between the two masses

		double r = springVector.length();											//distance between the two masses

		Vector3D force;																//force initially has a zero value

		if (r != 0.0)																	//to avoid a division by zero check if r is zero
			force += (springVector / r) * (r - springLength) * (-springConstant);	//the spring force is added to the force

		force += -(mass1->vel - mass2->vel) * frictionConstant;						//the friction force is added to the force
																					//with this addition we obtain the net force of the spring

		mass1->applyForce(force);													//force is applied to mass1
		mass2->applyForce(-force);													//the opposite of force is applied to mass2
	}

};

// class Simulation		---> A container object for simulating masses
class Simulation
{
public:
	int numOfMasses;								// number of masses in this container
	Mass** masses[maxNumRopes];						// masses are held by pointer to pointer. (Here Mass** represents a 1 dimensional array)

	Mass* fmass;

	int numOfRopes;								// number of ropes in this container

	Simulation(int nOfMasses, double m, double fm, int nRopes)			// Constructor creates nRopes groups of masses with mass values m and fm
	{
		this->numOfMasses = nOfMasses - 1;
		this->numOfRopes = nRopes;

		for (int a = 0; a < numOfRopes; ++a)
			masses[a] = new Mass*[numOfMasses];			// Create an array of pointers

		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)		// We will step to every pointer in the array
				masses[a][b] = new Mass(m);				// Create a Mass as a pointer and put it in the array

		fmass = new Mass(fm);				// Create a Mass as a pointer and put it in the array
	}

	virtual void release()							// delete the masses created
	{
		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)		// we will delete all of them
			{
				delete(masses[a][b]);
				masses[a][b] = NULL;
			}

		for (int a = 0; a < numOfRopes; ++a)
		{
			delete(masses[a]);
			masses[a] = NULL;
		}

		delete(fmass);
		fmass = NULL;
	}

	/*Mass* getMass(int nRope, int index) CHEQUEAR BIEN PORQUE ALGO ANDA MAL
	{
		if (index < 0 || index >= numOfMasses)		// if the index is not in the array
			return NULL;							// then return NULL

		if (nRope == 0)
			return fmass;							// get the mass at the index
		else
			return masses[nRope-1][index];			// get the mass at the index

		return NULL;
	}*/

	Vector3D getMassForce(int nRope, int index)
	{
		if (index < 0 || index >= numOfMasses)		// if the index is not in the array
			return Vector3D();							// then return NULL

		if (nRope == 0)
			return fmass->force;					// get the mass at the index
		else
			return masses[nRope - 1][index]->force;	// get the mass at the index

		return Vector3D();
	}

	virtual void init()								// this method will call the init() method of every mass
	{
		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)		// We will init() every mass
				masses[a][b]->init();						// call init() method of the mass

		fmass->init();
	}

	virtual void solve()							// no implementation because no forces are wanted in this basic container
	{
		// in advanced containers, this method will be overrided and some forces will act on masses
	}

	virtual void simulate(double dt)					// Iterate the masses by the change in time
	{
		/*static int mostrar;
		if (mostrar > n_mostrar)
			mostrar = 0;
		else
			mostrar++;

		if (mostrar > n_mostrar-1)
			std::cout << "fmass->force ANTES: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";*/

		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)
				masses[a][b]->simulate(dt);				// Iterate the mass and obtain new position and new velocity

		fmass->simulate(dt);

		//if (mostrar > n_mostrar-1)
			//std::cout << "fmass->force DESPUES: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
	}

	virtual void operate(double dt)					// The complete procedure of simulation
	{
		init();										// Step 1: reset forces to zero
		solve();									// Step 2: apply forces
		simulate(dt);								// Step 3: iterate the masses by the change in time
	}

};

/*
class RopeSimulation is derived from class Simulation (see Physics1.h). It simulates a rope with
point-like particles binded with springs. The springs have inner friction and normal length. One tip of
the rope is stabilized at a point in space called "Vector3D ropeConnectionPos". This point can be
moved externally by a method "void setRopeConnectionVel(Vector3D ropeConnectionVel)". RopeSimulation
creates air friction and a planer surface (or ground) with a normal in +y direction. RopeSimulation
implements the force applied by this surface. In the code, the surface is refered as "ground".
*/
class RopeSimulation : public Simulation				//An object to simulate a rope interacting with a planer surface and air
{
public:
	Spring** springs[maxNumRopes];;						//Springs binding the masses of ropes (there shall be [numOfMasse s - 1] of them for each rope)

	Vector3D gravitation;								//gravitational acceleration (gravity will be applied to all masses)

	Vector3D windForceRope[maxNumRopes];				//wind force calculated from velocity and direction of wind
	Vector3D windForceLoad;

	double Cd_rope = 1.0f;								//Drag coefficient of rope = 1.0–1.3	wires and cables
	double Cd_load = 1.05f;								//Placa plana perpendicular al flujo (1.28)
														//Cubo -> 1.05 viento de frente, 0.8 viento en una esquina
														//https://en.wikipedia.org/wiki/Drag_coefficient
														//https://www.grc.nasa.gov/WWW/k-12/airplane/shaped.html
														//https://www.grc.nasa.gov/www/k-12/airplane/drageq.html
	double A_rope;										//Lateral area of each rope link
	double A_load;										//Lateral area of load

	double loadHeight;									//Height of Load

	double rho;											//air density
	Vector3D ropeConnectionPos[maxNumRopes];							//A point in space that is used to set the position of the 
															//first mass in the system (mass with index 0)
	Vector3D ropeConnectionVel[maxNumRopes];							//a variable to move the ropeConnectionPos (by this, we can swing the rope)

	Vector3D posInicial[maxNumRopes];								//6. position of first mass for each rope 

	double groundRepulsionConstant;						//a constant to represent how much the ground shall repel the masses

	double groundFrictionConstant;						//a constant of friction applied to masses by the ground
														//(used for the sliding of rope on the ground)

	double groundAbsorptionConstant;						//a constant of absorption friction applied to masses by the ground
														//(used for vertical collisions of the rope with the ground)

	double groundHeight;									//a value to represent the y position value of the ground
														//(the ground is a planer surface facing +y direction)

	double springLength;								//the length that a spring does not exert any force

	double airFrictionConstant;							//a constant of air friction applied to masses

	int numOfIterations;

	RopeSimulation(										//a long long constructor with 16 parameters starts here
		int nOfMasses,									//1. the number of masses
		int nRopes,										//2. the number of ropes
		double m,										//3. weight of each mass
		double fm,										//4. weight of final mass
		double loadSize,								//5. the loadSize (side length)
		double springConstant,							//6. how stiff the springs are
		double springLength,							//7. the length that a spring does not exert any force
		double springFrictionConstant,					//8. inner friction constant of spring
		Vector3D position[],							//9. position of first mass for each rope
		double rho,										//10. air density			
		Vector3D gravitation,							//11. gravitational acceleration
		double airFrictionConstant,						//12. air friction constant
		double groundRepulsionConstant,					//13. ground repulsion constant
		double groundFrictionConstant,					//14. ground friction constant
		double groundAbsorptionConstant,				//15. ground absorption constant
		double groundHeight								//16. height of the ground (y position)
	) : Simulation(nOfMasses, m, fm, nRopes)			//The super class creates masses with weights m of each
	{
		this->gravitation = gravitation;

		this->airFrictionConstant = airFrictionConstant;

		this->groundFrictionConstant = groundFrictionConstant;
		this->groundRepulsionConstant = groundRepulsionConstant;
		this->groundAbsorptionConstant = groundAbsorptionConstant;
		this->groundHeight = groundHeight;
		this->springLength = springLength;
		this->rho = rho;

		for (int a = 0; a < numOfRopes; ++a)
			this->posInicial[a] = position[a];

		this->loadHeight = loadSize;
		//A_rope = PI * 0.005 * springLength;	//0.01 diameter rope
		//A_rope = PI * 0.002 * springLength;	//0.004 diameter rope
		A_rope = PI * 0.003 * springLength;	//0.006 diameter rope ->http://www.samsonropecatalogs.com/lg_display.cfm/catalog/Industrial_Product_Guide_2017/page/6
		A_load = loadSize * loadSize;
		//A_load = A_rope;

		for (int a = 0; a < numOfRopes; ++a)
			windForceRope[a] = Vector3D();
		windForceLoad = Vector3D();

		Vector3D PuntoMedio = Vector3D();
		//std::cout << "PuntoMedio.x = " << PuntoMedio.x << " PuntoMedio.y = " << PuntoMedio.y << " PuntoMedio.z = " << PuntoMedio.z << "            \n";
		for (int a = 0; a < numOfRopes; ++a)
		{
			PuntoMedio = PuntoMedio + position[a];
			//std::cout << "PuntoMedio.x = " << PuntoMedio.x << " PuntoMedio.y = " << PuntoMedio.y << " PuntoMedio.z = " << PuntoMedio.z << "            \n";
		}
		PuntoMedio = PuntoMedio / (double)numOfRopes;

		//std::cout << "PuntoMedio.x = " << PuntoMedio.x << " PuntoMedio.y = " << PuntoMedio.y << " PuntoMedio.z = " << PuntoMedio.z << "            \n";

		for (int a = 0; a < numOfRopes; ++a)
		{
			masses[a][0]->pos.x = PuntoMedio.x + 0.01;		//Set x position of masses[a][0] 
			masses[a][0]->pos.y = PuntoMedio.y + numOfMasses * springLength + 10.0;	//Set y position + 10 m de margen
			masses[a][0]->pos.z = PuntoMedio.z;
		}

		//std::cout << "masses[0][0]->pos: X = " << masses[0][0]->pos.x << " Y = " << masses[0][0]->pos.y << " Z = " << masses[0][0]->pos.z << "            \n";


		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 1; b < numOfMasses; ++b)			//To set the initial positions of masses loop with for(;;)
			{
				masses[a][b]->pos.x = masses[a][b - 1]->pos.x;					//Set x position of masses[a] with springLength distance to its neighbor
				masses[a][b]->pos.y = masses[a][b - 1]->pos.y - springLength;	//Set y position as 0 so that it stand horizontal with respect to the ground
				masses[a][b]->pos.z = masses[a][b - 1]->pos.z;					//Set z position as 0 so that it looks simple
			}

		//std::cout << "masses[0][10]->pos: X = " << masses[0][10]->pos.x << " Y = " << masses[0][10]->pos.y << " Z = " << masses[0][10]->pos.z << "            \n";

		fmass->pos.x = masses[0][numOfMasses - 1]->pos.x;		//Set x position of masses[a] with springLength distance to its neighbor
		fmass->pos.y = masses[0][numOfMasses - 1]->pos.y - springLength;		//Set y position as 0 so that it stand horizontal with respect to the ground
		fmass->pos.z = masses[0][numOfMasses - 1]->pos.z;

		//std::cout << "fmass->pos: X = " << fmass->pos.x << " Y = " << fmass->pos.y << " Z = " << fmass->pos.z << "            \n";

		for (int a = 0; a < numOfRopes; ++a)
			ropeConnectionPos[a] = masses[a][0]->pos;

		for (int a = 0; a < numOfRopes; ++a)
			springs[a] = new Spring*[numOfMasses];		//create [numOfMasses - 1] pointers for springs
														//([numOfMasses - 1] springs are necessary for numOfMasses)
		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses - 1; ++b)	//to create each spring, start a loop, Create the spring with index "b" by the mass with index "b" and another mass with index "b + 1".
				springs[a][b] = new Spring(masses[a][b], masses[a][b + 1], springConstant, springLength, springFrictionConstant);

		for (int a = 0; a < numOfRopes; ++a)
			springs[a][numOfMasses - 1] = new Spring(masses[a][numOfMasses - 1], fmass, springConstant, springLength, springFrictionConstant);
	}

	void release()										//release() is overriden because we have springs to delete
	{
		Simulation::release();							//Have the super class release itself


		for (int a = 0; a < numOfRopes; ++a)			//to delete all springs, start a loop
			for (int b = 0; b < numOfMasses; ++b)
			{
				delete(springs[a][b]);
				springs[a][b] = NULL;
			}

		for (int a = 0; a < numOfRopes; ++a)
		{
			delete(springs[a]);
			springs[a] = NULL;
		}
	}

	void solve()										//solve() is overriden because we have forces to be applied
	{
		static int mostrar;
		if (mostrar > n_mostrar)
			mostrar = 0;
		else
			mostrar++;

		//if (mostrar > n_mostrar - 1)
			//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
			//std::cout << "fmass->m = " << fmass->m << "            \n";

		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)		//apply force of all springs
			{
				/*if ((mostrar > n_mostrar-1) && (b == numOfMasses - 1))
				//if ((mostrar > n_mostrar-1) && (b < numOfMasses - 35))
				{
					std::cout << "masses[" << a << "][" << b << "]->pos: X = " << masses[a][b]->pos.x << " Y = " << masses[a][b]->pos.y << " Z = " << masses[a][b]->pos.z << "            \n";
					std::cout << "fmass->pos: X = " << fmass->pos.x << " Y = " << fmass->pos.y << " Z = " << fmass->pos.z << "            \n";

					std::cout << "masses[" << a << "][" << b << "]->vel: X = " << masses[a][b]->vel.x << " Y = " << masses[a][b]->vel.y << " Z = " << masses[a][b]->vel.z << "            \n";
					std::cout << "fmass->vel: X = " << fmass->vel.x << " Y = " << fmass->vel.y << " Z = " << fmass->vel.z << "            \n";
				}*/

				springs[a][b]->solve();					//Spring with index "b" should apply its force

				/*if ((mostrar > n_mostrar-1) && (b == numOfMasses - 1))
				//if ((mostrar > n_mostrar-1) && (b < numOfMasses - 35))
				{
					std::cout << "springs[" << a << "][" << b << "]->mass1->force: X = " << springs[a][b]->mass1->force.x << " Y = " << springs[a][b]->mass1->force.y << " Z = " << springs[a][b]->mass1->force.z << "            \n";
					std::cout << "springs[" << a << "][" << b << "]->mass2->force: X = " << springs[a][b]->mass2->force.x << " Y = " << springs[a][b]->mass2->force.y << " Z = " << springs[a][b]->mass2->force.z << "            \n";
				}*/
			}

		//if (mostrar > n_mostrar - 1)
			//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";

		for (int a = 0; a < numOfRopes; ++a)
			for (int b = 0; b < numOfMasses; ++b)				//Start a loop to apply forces which are common for all masses
			{
				masses[a][b]->applyForce(gravitation * masses[a][b]->m);			//The gravitational force
				masses[a][b]->applyForce(-masses[a][b]->vel * airFrictionConstant);	//The air friction
				masses[a][b]->applyForce(windForceRope[a]);		//The wind force (drag)

				if (masses[a][b]->pos.y < groundHeight)		//Forces from the ground are applied if a mass collides with the ground
				{
					Vector3D v;								//A temporary Vector3D

					v = masses[a][b]->vel;					//get the velocity
					v.y = 0.0;								//omit the velocity component in y direction
															//The velocity in y direction is omited because we will apply a friction force to create 
															//a sliding effect. Sliding is parallel to the ground. Velocity in y direction will be used
															//in the absorption effect.
					masses[a][b]->applyForce(-v * groundFrictionConstant);		//ground friction force is applied
					v = masses[a][b]->vel;					//get the velocity
					v.x = 0.0;								//omit the x and z components of the velocity
					v.z = 0.0;								//we will use v in the absorption effect
															//above, we obtained a velocity which is vertical to the ground and it will be used in 
															//the absorption force
					if (v.y < 0.0)							//let's absorb energy only when a mass collides towards the ground
						masses[a][b]->applyForce(-v * groundAbsorptionConstant);		//the absorption force is applied

																					//The ground shall repel a mass like a spring. 
																					//By "Vector3D(0, groundRepulsionConstant, 0)" we create a vector in the plane normal direction 
																					//with a magnitude of groundRepulsionConstant.
																					//By (groundHeight - masses[a]->pos.y) we repel a mass as much as it crashes into the ground.
					Vector3D force = Vector3D(0.0, groundRepulsionConstant, 0.0) *
						(groundHeight - masses[a][b]->pos.y);

					masses[a][b]->applyForce(force);			//The ground repulsion force is applied
				}
			}

		fmass->applyForce(gravitation * fmass->m);				//The gravitational force
		//if (mostrar > n_mostrar-1)
			//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
			//std::cout << "gravitation * fmass->m: X = " << (gravitation * fmass->m).x << " Y = " << (gravitation * fmass->m).y << " Z = " << (gravitation * fmass->m).z << "            \n";

		fmass->applyForce(-fmass->vel * airFrictionConstant*fmass->m / masses[0][0]->m);	//The load air friction (n times the rope link air friction)

		//if (mostrar > n_mostrar-1)
			//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
			//std::cout << "-fmass->vel * airFrictionConstant*10.0: X = " << (-fmass->vel * airFrictionConstant*10.0).x << " Y = " << (-fmass->vel * airFrictionConstant*10.0).y << " Z = " << (-fmass->vel * airFrictionConstant*10.0).z << "            \n";

		Vector3D force;
		if (fmass->vel.x > 0)
			force.x = Cd_load * 0.5 * rho * A_load * fmass->vel.x * fmass->vel.x;
		else
			force.x = -Cd_load * 0.5 * rho * A_load * fmass->vel.x * fmass->vel.x;
		if (fmass->vel.y > 0)
			force.y = Cd_load * 0.5 * rho * A_load * fmass->vel.y * fmass->vel.y;
		else
			force.y = -Cd_load * 0.5 * rho * A_load * fmass->vel.y * fmass->vel.y;
		if (fmass->vel.z)
			force.z = Cd_load * 0.5 * rho * A_load * fmass->vel.z * fmass->vel.z;
		else
			force.z = -Cd_load * 0.5 * rho * A_load * fmass->vel.z * fmass->vel.z;
		fmass->applyForce(-force);

		//if (mostrar > n_mostrar-1)
			//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
			//std::cout << "-force: X = " << (-force).x << " Y = " << (-force).y << " Z = " << (-force).z << "            \n";

		fmass->applyForce(windForceLoad);						//The wind force (drag)

		if (fmass->pos.y < groundHeight)		//Forces from the ground are applied if a mass collides with the ground
		{
			Vector3D v;								//A temporary Vector3D

			v = fmass->vel;						//get the velocity
			v.y = 0.0;								//omit the velocity component in y direction

													//The velocity in y direction is omited because we will apply a friction force to create 
													//a sliding effect. Sliding is parallel to the ground. Velocity in y direction will be used
													//in the absorption effect.
			fmass->applyForce(-v * groundFrictionConstant*fmass->m / masses[0][0]->m);		//ground friction force is applied

			//if (mostrar > n_mostrar-1)
				//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
				//std::cout << "-v * 1.5 * groundFrictionConstant*fmass->m / masses[0][0]->m: X = " << (-v * 1.5 * groundFrictionConstant*fmass->m / masses[0][0]->m).x << " Y = " << (-v * 1.5 * groundFrictionConstant*fmass->m / masses[0][0]->m).y << " Z = " << (-v * 1.5 * groundFrictionConstant*fmass->m / masses[0][0]->m).z << "            \n";

			v = fmass->vel;						//get the velocity
			v.x = 0.0;								//omit the x and z components of the velocity
			v.z = 0.0;								//we will use v in the absorption effect

													//above, we obtained a velocity which is vertical to the ground and it will be used in 
													//the absorption force

			if (v.y < 0.0)							//let's absorb energy only when a mass collides towards the ground
				fmass->applyForce(-v * groundAbsorptionConstant*fmass->m / masses[0][0]->m);		//the absorption force is applied

			//if (mostrar > n_mostrar-1)
				//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
				//std::cout << "-v * groundAbsorptionConstant*fmass->m / masses[0][0]->m: X = " << (-v * groundAbsorptionConstant*fmass->m / masses[0][0]->m).x << " Y = " << (-v * groundAbsorptionConstant*fmass->m / masses[0][0]->m).y << " Z = " << (-v * groundAbsorptionConstant*fmass->m / masses[0][0]->m).z << "            \n";

																		//The ground shall repel a mass like a spring. 
																		//By "Vector3D(0, groundRepulsionConstant, 0)" we create a vector in the plane normal direction 
																		//with a magnitude of groundRepulsionConstant.
																		//By (groundHeight - masses[a]->pos.y) we repel a mass as much as it crashes into the ground.
			Vector3D force = Vector3D(0.0, groundRepulsionConstant*fmass->m / masses[0][0]->m, 0.0) *
				(groundHeight - fmass->pos.y);

			fmass->applyForce(force);			//The ground repulsion force is applied

		   /*if (mostrar > n_mostrar-1)
		   {
			   //std::cout << "force: X = " << (force).x << " Y = " << (force).y << " Z = " << (force).z << "            \n";
			   std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n\n";
			   //std::cout << "fmass->pos.y = " << fmass->pos.y << "            \n";
		   }*/

		}
	}

	void simulate(double dt)								//simulate(double dt) is overriden because we want to simulate 
														//the motion of the ropeConnectionPos
	{
		Simulation::simulate(dt);						//the super class shall simulate the masses

		for (int a = 0; a < numOfRopes; ++a)
		{
			ropeConnectionPos[a] += ropeConnectionVel[a] * dt;	//iterate the positon of ropeConnectionPos

			if (ropeConnectionPos[a].y < groundHeight)			//ropeConnectionPos shall not go under the ground
			{
				ropeConnectionPos[a].y = groundHeight;
				ropeConnectionVel[a].y = 0.0;
			}
			masses[a][0]->pos = ropeConnectionPos[a];				//mass with index "0" shall position at ropeConnectionPos
			masses[a][0]->vel = ropeConnectionVel[a];				//the mass's velocity is set to be equal to ropeConnectionVel
		}
	}

	void setRopeConnectionVel(Vector3D ropeConVel[])	//the method to set ropeConnectionVel
	{
		for (int a = 0; a < numOfRopes; ++a)
			this->ropeConnectionVel[a] = ropeConVel[a];
	}

	void setRopeConnectionPos()
	{
		Vector3D aux[maxNumRopes];
		Vector3D distancia[maxNumRopes];
		double distanciaTotal;

		for (int a = 0; a < numOfRopes; ++a)
		{
			aux[a] = (posInicial[a] - ropeConnectionPos[a]);
			//std::cout << "posInicial[a]: X = " << posInicial[a].x << " Y = " << posInicial[a].y << " Z = " << posInicial[a].z << "            \n";
			//std::cout << "ropeConnectionPos[a]: X = " << ropeConnectionPos[a].x << " Y = " << ropeConnectionPos[a].y << " Z = " << ropeConnectionPos[a].z << "            \n";
			//std::cout << "aux[a]: X = " << aux[a].x << " Y = " << aux[a].y << " Z = " << aux[a].z << "            \n";
			aux[a].unitize();
			//aux[a] /= 2;
			//aux[a] /= 4;
			aux[a] /= 5;
			//std::cout << "aux[a].unitize(): X = " << aux[a].x << " Y = " << aux[a].y << " Z = " << aux[a].z << "            \n";
		}

		setRopeConnectionVel(aux);

		distanciaTotal = 1.0;
		//int mostrar = 0;
		while (distanciaTotal > (double)numOfRopes*epsilon)
		{
			static int mostrar;
			if (mostrar > n_mostrar)
				mostrar = 0;
			else
				mostrar++;


			operate(maxPossible_dt);

			//if (mostrar > n_mostrar-1)
				//std::cout << "fmass->force ANTES: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";


			distanciaTotal = 0.0;
			for (int a = 0; a < numOfRopes; ++a)
			{
				distancia[a] = posInicial[a] - ropeConnectionPos[a];
				if (distancia[a].length() < epsilon)
				{
					aux[a] = Vector3D();
					setRopeConnectionVel(aux);
					//std::cout << "Velocidad Cero a aux =  " << a << "\n";
				}
				distanciaTotal = distanciaTotal + distancia[a].length();

				/*if (mostrar > n_mostrar-1)
				{
					std::cout << "distanciaTotal = " << distanciaTotal << "            \n\n";
				}*/

				/*if (mostrar > 300)
				{
					//std::cout << "masses[0][0]->pos: X = " << masses[0][0]->pos.x << " Y = " << masses[0][0]->pos.y << " Z = " << masses[0][0]->pos.z << "            \n";
					//std::cout << "fmass->pos: X = " << fmass->pos.x << " Y = " << fmass->pos.y << " Z = " << fmass->pos.z << "            \n";
					//std::cout << "fmass->vel: X = " << masses[0][0]->vel.x << " Y = " << fmass->vel.y << " Z = " << fmass->vel.z << "            \n";
					//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
					//std::cout << "groundHeight - fmass->pos.y = " << (groundHeight - fmass->pos.y) << "            \n";
					//std::cout << "groundRepulsionConstant = " << groundRepulsionConstant << "            \n";
					//std::cout << "fmass->m = " << fmass->m << "            \n";
					//std::cout << "fmass->m: = " << fmass->m << "            \n";
					//std::cout << "distanciaTotal = " << distanciaTotal << "            \n";
					//std::cout << "fmass->force: X = " << fmass->force.x << " Y = " << fmass->force.y << " Z = " << fmass->force.z << "            \n";
					mostrar = 0;
				}
				else
					mostrar++;*/
			}
		}

		for (int a = 0; a < numOfRopes; ++a)
			aux[a] = Vector3D();
		setRopeConnectionVel(aux);

		for (int i = 0; i < (int)(20.0 / maxPossible_dt); ++i)  //simulo 20 seg para asegurarme que la cuerda esté estática al comienzo
			operate(maxPossible_dt);
	}

	void setRopeConnectionPos(Vector3D ropePosDest[], double deltat)
	{
		Vector3D aux[maxNumRopes];
		for (int a = 0; a < numOfRopes; ++a)
			aux[a] = (ropePosDest[a] - ropeConnectionPos[a]) / deltat;

		setRopeConnectionVel(aux);

		/*numOfIterations = 0;

		for (int a = 0; a < numOfRopes; ++a)
		{
			while ((ropeConnectionPos[a] - ropePosDest[a]).length() > epsilon)
			{
				operate(maxPossible_dt);
				numOfIterations++;
				//printf("\n numOfIterations++ = %d                            \n", numOfIterations);
				if (numOfIterations > dt / maxPossible_dt)
					goto stop;
			}
		}
		stop:*/

		double dt;
		int numOfIterations = (int)(deltat / maxPossible_dt) + 1;					// Calculate Number Of Iterations To Be Made At This Update Depending On maxPossible_dt And dt
		if (numOfIterations != 0)												// Avoid Division By Zero
			dt = deltat / numOfIterations;											// dt Should Be Updated According To numOfIterations

		/*if (numOfIterations > ropeSimulation->numOfIterations)
		{
			for (int a = 0; a < (numOfIterations - ropeSimulation->numOfIterations); ++a)								// We Need To Iterate Simulations "numOfIterations" Times
			ropeSimulation->operate(dt);
		}*/

		for (int a = 0; a < numOfIterations; ++a)								// We Need To Iterate Simulations "numOfIterations" Times
			operate(dt);
	}

	void setRopeWindForce(int nRope, Vector3D wind)
	{
		if (wind.x > 0)
			windForceRope[nRope].x = Cd_rope * 0.5 * rho * A_rope * wind.x * wind.x;
		else
			windForceRope[nRope].x = -Cd_rope * 0.5 * rho * A_rope * wind.x * wind.x;

		if (wind.z > 0)
			windForceRope[nRope].y = Cd_rope * 0.5 * rho * A_rope * wind.z * wind.z;
		else
			windForceRope[nRope].y = -Cd_rope * 0.5 * rho * A_rope * wind.z * wind.z;

		if (wind.y > 0)
			windForceRope[nRope].z = -Cd_rope * 0.5 * rho * A_rope * wind.y * wind.y;
		else
			windForceRope[nRope].z = Cd_rope * 0.5 * rho * A_rope * wind.y * wind.y;
	}

	void setLoadWindForce(Vector3D wind)
	{
		if (wind.x > 0)
			windForceLoad.x = Cd_load * 0.5 * rho * A_load * wind.x * wind.x;
		else
			windForceLoad.x = -Cd_load * 0.5 * rho * A_load * wind.x * wind.x;

		if (wind.z > 0)
			windForceLoad.y = Cd_load * 0.5 * rho * A_load * wind.z * wind.z;
		else
			windForceLoad.y = -Cd_load * 0.5 * rho * A_load * wind.z * wind.z;

		if (wind.y > 0)
			windForceLoad.z = -Cd_load * 0.5 * rho * A_load * wind.y * wind.y;
		else
			windForceLoad.z = Cd_load * 0.5 * rho * A_load * wind.y * wind.y;
	}
};

template <class StructType>
class CSharedStruct
{
private:
	HANDLE m_hFileMapping;
	char   m_hSharedName[SHARED_NAME_SIZE];
	DWORD  m_dwMaxDataSize;
	StructType *m_pvData;

	BOOL m_bCreated;
	BOOL m_bNamed;

public:
	CSharedStruct();
	CSharedStruct(char *Name);
	VOID Name(char *Name);
	VOID Release();
	BOOL Acquire();

	StructType *operator->();
};


template <class StructType>
StructType *CSharedStruct<StructType>::operator->()
{
	return m_pvData;
}

template <class StructType>
CSharedStruct<StructType>::CSharedStruct()
{
	m_hFileMapping = NULL;
	m_pvData = NULL;
	m_bNamed = false;
	m_dwMaxDataSize = sizeof(StructType);
}

template <class StructType>
CSharedStruct<StructType>::CSharedStruct(char *Name)
{
	m_hFileMapping = NULL;
	m_pvData = NULL;
	strncpy_s(m_hSharedName, Name, SHARED_NAME_SIZE - 1);
	m_bNamed = true;
	m_dwMaxDataSize = sizeof(StructType);
}

template <class StructType>
VOID CSharedStruct<StructType>::Name(char *Name)
{
	strncpy_s(m_hSharedName, Name, SHARED_NAME_SIZE - 1);
	m_bNamed = true;
}

template <class StructType>
VOID CSharedStruct<StructType>::Release()
{
	if (m_pvData)
	{
		UnmapViewOfFile(m_pvData);
	}

	if (m_hFileMapping)
	{
		CloseHandle(m_hFileMapping);
	}
}

template <class StructType>
BOOL CSharedStruct<StructType>::Acquire()
{
	m_dwMaxDataSize = 0;
	if (m_bNamed)
	{
		m_hFileMapping = CreateFileMapping(INVALID_HANDLE_VALUE, NULL,
			PAGE_READWRITE, 0, sizeof(StructType), m_hSharedName);
	}
	else
		return FALSE;

	if (m_hFileMapping == NULL)
	{
		return FALSE;
	}

	m_pvData = (StructType *)MapViewOfFile(m_hFileMapping,
		FILE_MAP_READ | FILE_MAP_WRITE, 0, 0, 0);

	if (m_pvData == NULL)
	{
		CloseHandle(m_hFileMapping);
		return FALSE;
	}

	return TRUE;
}

typedef struct tag_Quad {
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double u;
	double v;
	double w;
	double p;
	double q;
	double r;
	double roll;
	double pitch;
	double yaw;
	double omega1;
	double omega2;
	double omega3;
	double omega4;
	double vxh;
	double vyh;
	double vzh;
	double wzh;
	double tiempoSim;
}Quad;

typedef struct tag_Quads {
	Quad quadcopter[maxNumQuads];
	int simCount;
}Quads;

typedef struct tag_Comandos {
	double v_xh[maxNumQuads];		//m/s en Coordenadas Locales (pero no Coordenadas del Body)
	double v_yh[maxNumQuads];		//m/s
	double v_zh[maxNumQuads];		//m/s
	double w_zh[maxNumQuads];		//m/s
}Comandos;

typedef struct tag_RopePos {
	Vector3D rPoints[maxNumRopes][numParticles];
	Vector3D forceRope[maxNumRopes];
	double tiempoSim;
	int simCount;
}RopePos;

typedef struct tag_VirtualLoad {
	Vector3D position;
	double orientation;
}VirtualLoad;

typedef struct tag_LaserData {
	float laserData[maxNumQuads][reg_ang1 / res_ang1][(reg_ang2 / res_ang2) + 1];
	Vector3D fuerzaFicticia[maxNumQuads];
	int simCount;
}LaserData;

typedef struct tag_ConfigSim {
	bool running;
	int number_quads;			// Number of Quads/Ropes
	double time;				// Time of simulation
	double weight_load;			// this includes 0.25kg for each rope
	double height_load;			// Height of load
	double rope_length;			// Each rope length in [m]
	double spring_friction;		// Spring Inner Friction Constant (0.2 default)
	double spring_constant;		// Spring Constant In The Rope (10000.0 default)
	double air_friction;		// Air Friction Constant (0.02 default)
	double ground_repel;		// Ground Repel Constant (100 default)
	double ground_slide;		// Ground Slide Friction Constant (0.2 default)
	double ground_absorption;	// Ground Absoption Constant (2.0 default)
	double Xo[maxNumQuads];		// Initial X Position of each quad
	double Yo[maxNumQuads];		// Initial Y Position of each quad
	double Zo[maxNumQuads];		// Initial Z Position of each quad
	double Tho[maxNumQuads];	// Initial Theta of each quad in degrees
	bool laser[maxNumQuads];	// true for each quad with laser
	int number_obs;				// Number of Obs
	Vector3D Obs[maxNumObs];	// Obstacles origin (x,y,z)
	Vector3D dObs[maxNumObs];	// Size of Obstacles (dx,dy,dz) 
	Vector3D RopeWind[maxNumRopes];	// Wind velocity in x,y,z [m/s] 
	Vector3D LoadWind;			// Wind velocity in x,y,z [m/s]
	Vector3D QuadWind[maxNumQuads];	// Wind velocity in x,y,z [m/s] 
}ConfigSim;
