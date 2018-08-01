#ifndef EIGEN2_SUPPORT
#define EIGEN2_SUPPORT
#endif

#include "math.h"
#include "TriMesh.h"
#include <set>
#include <vector>
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include<Eigen/LU>

using namespace std;
using namespace Eigen;

struct Vec3d{
	double x;
	double y;
	double z;
	double operator[](int i) 
	{ 
		if(i==0)
			return x;
		else if(i==1)
			return y;
		else if(i==2)
			return z;
		else
			return 0;
	}
};

struct Vertex{
	double x;
	double y;
	double z;
	double operator[](int i) 
	{ 
		if(i==0)
			return x;
		else if(i==1)
			return y;
		else if(i==2)
			return z;
		else
			cout<<"Vertex index out of bounds!";
	}
};

struct Triangle{
	int v1;
	int v2;
	int v3;
};

struct Tet{
	int v[4];
	inline double operator[](int i){ return v[i]; }
};

class Mesh{
public:
	int N; // number of vertices;
	int M; // number of triangles;
	double density; // material density
	double boundingSphere[4];
	bool isTet;

	vector<Vertex> vertices; // rest coordinates of each vertex
	vector<Triangle> triangles; // used to store faces for a tet mesh
	vector<Tet> tetrahedrens;
	vector<vector<int> > vtx_tri_nbrs; // neighboring triangles of each vertex
	vector<vector<int> > vtx_vtx_nbrs; // neighboring vertices of each vertex
	vector<Vec3d> vtx_normals;
	vector<Vec3d> face_normals;

	void add_cube(double x, double y, double z, double l, vector<int> &cube_vertices);
	Mesh(int builtin_id);
	Mesh(TriMesh* input_mesh, double d);
	Mesh(int num_vertices, double d, bool isTet=false);
	Mesh(double d, bool isTetMesh);
	void update_Fext();
	void add_vertex(double x, double y, double z);
	void add_triangle(int v1, int v2, int v3);
	void add_tet(int v1, int v2, int v3, int v4);
	double triangle_area(int tri);
	double triangle_area(Triangle tri);
	double tetrahedren_volume(int tet_id);
	double compute_tetrahedren_volume_and_invV(int tet_id, Matrix3f &invV);
	void compute_laplacian_matrix(SparseMatrix<double, RowMajor>* laplacian);
	bool triangleMatched(int v0, int v1, int v2, int tid);
	void recompute_face_normals();
	void recompute_vertex_normals();
	void compute_bounding_sphere();	

	inline double length(Vertex v1, Vertex v2){ return sqrt(pow(v1.x-v2.x, 2)+pow(v1.y-v2.y, 2)+pow(v1.z-v2.z, 2)); }
	inline double length(int v1_id, int v2_id)
	{ 
		Vertex v1=vertices[v1_id]; Vertex v2=vertices[v2_id];
		return sqrt(pow(v1.x-v2.x, 2)+pow(v1.y-v2.y, 2)+pow(v1.z-v2.z, 2)); 
	}
};