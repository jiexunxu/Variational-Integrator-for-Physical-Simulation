#include "Mesh.h"
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;

class Dynamics{
public:
	int N; // N is the total number of vertices of the system
	double hk; // hk is the step size at step k
	double kD; // kD is the coefficient in front of gradW(C(qk1, qk))
	double density; // density of the mesh

	vector<double> pk; // pk is the momentum at step k
	vector<double> qk; // qk is the position at step k
	vector<double> qk1; // qk1 the is position at step k-1
	vector<double> vk; // vk is the velocity at step k
	vector<double> q; // rest position of the mesh
	
	vector<double> Fext; // external force
	vector<double> Fenv; // Environment force
	vector<double> M; // M is the mass matrix
	vector<double> invM; // invM is the inverse mass matrix
	double mass_total; // total mass of the system

	double lambda_k;
	vector<bool> fix_vertices;
	vector<double> fixed_points;

	double dampFactor;

	int environment;
	double env_force_multiplier;

	vector<double> CM_rest; // center of mass at rest
	vector<double> shapematch_qi;

	int energy_model; // describes which potential energy model to use

	double kinetic_energy;
	double potential_energy;
	double momentum[3];

	// needed for laplacian model
	SparseMatrix<double, RowMajor>* laplacian; 
	// needed for neo-hookean and mooney-rivlin models
	vector<Tet>* tetrahedrens; 
	vector<double> tetVolumes;
	vector<Matrix3f> invVs;
	double a1;
	double b1;

	Dynamics(Mesh* m);
	Dynamics(Mesh* m, int em);

	void set_b1(double new_b1);
	void update_input_position(double qx, double qy, double qz, int idx);
	void update_input_momentum(double qx, double qy, double qz, int idx);
	void update_input_external_force(double fx, double fy, double fz, int idx);
	void update_vertex_constraints(double fix_x, double fix_y, double fix_z, int vidx);

	void compute_mass_matrix(Mesh* m);
	double compute_energy_density(const VectorXd &u, int tet_id, int v1, int v2, int v3, int v4, double dx[4][3]);
	void compute_cauchy_gradient(const VectorXd &u, int tet_id, int v1, int v2, int v3, int v4, double grad_output[4][3]);
	void compute_Wdamp_gradient(VectorXd &gradWdamp);
	void compute_W_gradient(const VectorXd &u, VectorXd &gradW);
	void compute_environment_forces();
	void update_velocity_explicit();
	void update_mometum_explicit();
	void update_position();
	void advance_system_explicit();
	void estimate_displacement_vector(VectorXd &u);
	void reset();	
	void compute_total_momentum(double total_momentum[3]);
	double compute_kinetic_energy();
	double compute_potential_energy(const VectorXd &u);
};