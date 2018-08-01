#include "Mesh.h"

void Mesh::add_cube(double x, double y, double z, double l, vector<int> &cube_vertices)
{
	if(cube_vertices[0]==-1){
		(*this).add_vertex(x, y, z);
		cube_vertices[0]=vertices.size()-1;
	}
	if(cube_vertices[1]==-1){
		(*this).add_vertex(x+l, y, z);
		cube_vertices[1]=vertices.size()-1;
	}	
	if(cube_vertices[2]==-1){
		(*this).add_vertex(x+l, y, z+l);
		cube_vertices[2]=vertices.size()-1;
	}
	if(cube_vertices[3]==-1){
		(*this).add_vertex(x, y, z+l);
		cube_vertices[3]=vertices.size()-1;
	}
	if(cube_vertices[4]==-1){
		(*this).add_vertex(x, y+l, z);
		cube_vertices[4]=vertices.size()-1;
	}
	if(cube_vertices[5]==-1){
		(*this).add_vertex(x+l, y+l, z);
		cube_vertices[5]=vertices.size()-1;
	}
	if(cube_vertices[6]==-1){
		(*this).add_vertex(x+l, y+l, z+l);
		cube_vertices[6]=vertices.size()-1;
	}
	if(cube_vertices[7]==-1){
		(*this).add_vertex(x, y+l, z+l);
		cube_vertices[7]=vertices.size()-1;
	}
	(*this).add_tet(cube_vertices[0], cube_vertices[1], cube_vertices[3], cube_vertices[4]);
	(*this).add_tet(cube_vertices[1], cube_vertices[2], cube_vertices[3], cube_vertices[6]);
	(*this).add_tet(cube_vertices[1], cube_vertices[3], cube_vertices[4], cube_vertices[6]);
	(*this).add_tet(cube_vertices[1], cube_vertices[4], cube_vertices[5], cube_vertices[6]);
	(*this).add_tet(cube_vertices[3], cube_vertices[4], cube_vertices[6], cube_vertices[7]);
}

Mesh::Mesh(int builtin_id)
{
	if(builtin_id==1) // One single tet
	{
		(*this)=Mesh(4, 1.0, true);		
		(*this).add_vertex(0, 0, 0);
		(*this).add_vertex(1, 0, 0);
		(*this).add_vertex(0, 1, 0);
		(*this).add_vertex(0, 0, 1);
		(*this).add_tet(0, 1, 2, 3);		
	}
	else if(builtin_id==2) // A cube consisting of 5 tets
	{
		(*this)=Mesh(8, 1.0, true);
		vector<int> cube_vertices;
		for(int i=0;i<8;i++)
			cube_vertices.push_back(-1);
		(*this).add_cube(0, 0, 0, 1, cube_vertices);
	}
	else if(builtin_id==3)
	{
		(*this)=Mesh(1.0, true);
		vector<int> cube_vertices;
		for(int i=0;i<8;i++)
			cube_vertices.push_back(-1);
		(*this).add_cube(0, 0, 0, 1, cube_vertices);
		for(int i=1;i<10;i++)		
		{
			cube_vertices[0]=cube_vertices[4];
			cube_vertices[1]=cube_vertices[5];
			cube_vertices[2]=cube_vertices[6];
			cube_vertices[3]=cube_vertices[7];
			cube_vertices[4]=-1;
			cube_vertices[5]=-1;
			cube_vertices[6]=-1;
			cube_vertices[7]=-1;
			(*this).add_cube(0, i, 0, 1, cube_vertices);		
		}
	}
}

Mesh::Mesh(TriMesh* input_mesh, double d)
{
	Mesh(input_mesh->vertices.size(), d, false);
	for(int i=0;i<input_mesh->vertices.size();i++)
	{
		point &p=input_mesh->vertices[i];
		add_vertex(p[0], p[1], p[2]);
	}
	for(int i=0;i<input_mesh->faces.size();i++)
	{
		TriMesh::Face &f=input_mesh->faces[i];			
		add_triangle(f[0], f[1], f[2]);
	}
}

Mesh::Mesh(int num_vertices, double d, bool isTetMesh)
{
	density=d;
	N=num_vertices;
	vtx_vtx_nbrs.resize(N);
	vtx_tri_nbrs.resize(N);
	isTet=isTetMesh;
}

Mesh::Mesh(double d, bool isTetMesh)
{
	density=d;
	isTet=isTetMesh;
}

void Mesh::compute_bounding_sphere()
{
	int size=vertices.size();
	boundingSphere[0]=0.0;boundingSphere[1]=0.0;boundingSphere[2]=0.0;
	for(int i=0;i<size;i++){
		boundingSphere[0]=boundingSphere[0]+vertices[i][0];
		boundingSphere[1]=boundingSphere[1]+vertices[i][1];
		boundingSphere[2]=boundingSphere[2]+vertices[i][2];
	}
	boundingSphere[0]=boundingSphere[0]/size;
	boundingSphere[1]=boundingSphere[1]/size;
	boundingSphere[2]=boundingSphere[2]/size;
	
	double max=-10000.0;
	for(int i=0;i<size;i++){
		double dist=pow(vertices[i][0]-boundingSphere[0], 2.0)+pow(vertices[i][1]-boundingSphere[1], 2.0)+
			pow(vertices[i][2]-boundingSphere[2], 2.0);
		if(dist>max){
			max=dist;
		}
	}
	boundingSphere[3]=sqrt(max)*2;
}

void Mesh::add_vertex(double x, double y, double z)
{
	Vertex vtx;
	vtx.x=x; vtx.y=y; vtx.z=z;
	vertices.push_back(vtx);
	Vec3d vec;
	vtx_normals.push_back(vec);
}

void Mesh::add_triangle(int v1, int v2, int v3)
{
	Triangle tri;
	tri.v1=v1; tri.v2=v2; tri.v3=v3;
	triangles.push_back(tri);
	Vec3d vec;
	face_normals.push_back(vec);

	vtx_vtx_nbrs[v1].push_back(v2);
	vtx_vtx_nbrs[v1].push_back(v3);
	vtx_vtx_nbrs[v2].push_back(v1);
	vtx_vtx_nbrs[v2].push_back(v3);
	vtx_vtx_nbrs[v3].push_back(v1);
	vtx_vtx_nbrs[v3].push_back(v2);

	vtx_tri_nbrs[v1].push_back(triangles.size()-1);
	vtx_tri_nbrs[v2].push_back(triangles.size()-1);
	vtx_tri_nbrs[v3].push_back(triangles.size()-1);
}

void Mesh::add_tet(int v1, int v2, int v3, int v4)
{
	Tet tet;
	tet.v[0]=v1;tet.v[1]=v2;tet.v[2]=v3;tet.v[3]=v4;
	tetrahedrens.push_back(tet);
	Triangle tri;
	tri.v1=tet[0];tri.v2=tet[1];tri.v3=tet[2];triangles.push_back(tri);
	tri.v1=tet[0];tri.v2=tet[1];tri.v3=tet[3];triangles.push_back(tri);
	tri.v1=tet[0];tri.v2=tet[2];tri.v3=tet[3];triangles.push_back(tri);
	tri.v1=tet[1];tri.v2=tet[2];tri.v3=tet[3];triangles.push_back(tri);
	Vec3d vec;
	face_normals.push_back(vec);
	face_normals.push_back(vec);
	face_normals.push_back(vec);
	face_normals.push_back(vec);	
}

double Mesh::triangle_area(int tri)
{
	Triangle t=triangles[tri];
	double a=length(t.v1, t.v2);
	double b=length(t.v1, t.v3);
	double c=length(t.v2, t.v3);
	double s=(a+b+c)/2;
	return sqrt(s*(s-a)*(s-b)*(s-c));
}

double Mesh::tetrahedren_volume(int tet_id)
{
	Tet& t=tetrahedrens[tet_id];
	Vertex& v1=vertices[t[0]];
	Vertex& v2=vertices[t[1]];
	Vertex& v3=vertices[t[2]];
	Vertex& v4=vertices[t[3]];
	Matrix3f mat;
	for(int i=0;i<3;i++)
	{
		mat(i, 0)=v1[i]-v2[i];
		mat(i, 1)=v2[i]-v3[i];
		mat(i, 2)=v3[i]-v4[i];
	}
	double det=mat.determinant();
	return abs(det)/6;
}

double Mesh::compute_tetrahedren_volume_and_invV(int tet_id, Matrix3f &invV)
{
	Tet& t=tetrahedrens[tet_id];
	Vertex& v1=vertices[t[0]];
	Vertex& v2=vertices[t[1]];
	Vertex& v3=vertices[t[2]];
	Vertex& v4=vertices[t[3]];
	Matrix3f mat;
	Matrix3f V;
	for(int i=0;i<3;i++)
	{
		mat(i, 0)=v1[i]-v2[i];
		mat(i, 1)=v2[i]-v3[i];
		mat(i, 2)=v3[i]-v4[i];
		V(i, 0)=v1[i]-v4[i];
		V(i, 1)=v2[i]-v4[i];
		V(i, 2)=v3[i]-v4[i];
	}
	double det=mat.determinant();
	invV=V.inverse();
	return abs(det)/6;
}

double Mesh::triangle_area(Triangle t)
{
	double a=length(t.v1, t.v2);
	double b=length(t.v1, t.v3);
	double c=length(t.v2, t.v3);
	double s=(a+b+c)/2;
	return sqrt(s*(s-a)*(s-b)*(s-c));
}

/*                 i (v_i)                                     i(v_i)
             /       |         \                         / a      |
(v_k)*it2   alpha    |   beta     *it2(v_k)       *it2(v_k)     c |
             \       |         /                         \ b      |
                   *it(v_j)                                    j(v_j)
*/  
void Mesh::compute_laplacian_matrix(SparseMatrix<double, RowMajor> *laplacian)
{			
	laplacian->reserve(N*15);
	double matrix_entry;
	for(unsigned int i=0;i<vertices.size();i++)
    {
        double weights[3];weights[0]=0;weights[1]=0;weights[2]=0;
        vector<int>* face_nbrs=&vtx_tri_nbrs[i];
        vector<int>* vtx_nbrs=&vtx_vtx_nbrs[i];
        Vertex vi=vertices[i];
        double diagonal_sum=0;
        double voronoi_area=0;
		for(int it=0;it<face_nbrs->size();it++)
			voronoi_area+=triangle_area(face_nbrs->at(it))/3;            

        // Loop for v_j
		for(int it=0;it<vtx_nbrs->size();it++)
		{        
            int target_vtx=vtx_nbrs->at(it);
            Vertex vj=vertices[target_vtx];
            double cot_angle_sum=0;                
            // Loop for v_k
			for(int it2=0;it2<vtx_nbrs->size();it2++)
            {     
                int cand_vtx=vtx_nbrs->at(it2);
                if(cand_vtx==target_vtx) continue;
                if(cand_vtx==i) continue;
                          
				for(int it3=0;it3<face_nbrs->size();it3++)
                {
                    if(triangleMatched(i, target_vtx, cand_vtx, face_nbrs->at(it3))){                
                        Vertex vk=vertices[cand_vtx];
                        double a=length(vi, vk);
                        double b=length(vj, vk);
                        double c=length(vi, vj);
                        double angle=acos((a*a+b*b-c*c)/(2*a*b));                       
                        double cot_angle=cos(angle)/sin(angle);
                        cot_angle_sum=cot_angle_sum+cot_angle;                                              
                    }                                        
                }                
            }            
            diagonal_sum=diagonal_sum+cot_angle_sum;                                                     
			matrix_entry=-cot_angle_sum/voronoi_area;
			laplacian->insert(i*3, target_vtx*3)=matrix_entry;
			laplacian->insert(i*3+1, target_vtx*3+1)=matrix_entry;
			laplacian->insert(i*3+2, target_vtx*3+2)=matrix_entry;
        } 
		matrix_entry=diagonal_sum/voronoi_area;
		laplacian->insert(i*3, i*3)=matrix_entry;
		laplacian->insert(i*3+1, i*3+1)=matrix_entry;
		laplacian->insert(i*3+2, i*3+2)=matrix_entry;
    }
	laplacian->finalize();
}

bool Mesh::triangleMatched(int v0, int v1, int v2, int tid)
{     	  
	Triangle tri=triangles[tid];
	int t0=tri.v1;
	int t1=tri.v2;
	int t2=tri.v3;
	if((t0==v0)&&(t1==v1)&&(t2==v2)) return true;
	if((t0==v0)&&(t1==v2)&&(t2==v1)) return true;
	if((t0==v1)&&(t1==v2)&&(t2==v0)) return true;
	if((t0==v1)&&(t1==v0)&&(t2==v2)) return true;
	if((t0==v2)&&(t1==v1)&&(t2==v0)) return true;
	if((t0==v2)&&(t1==v0)&&(t2==v1)) return true;
	return false;
}

void Mesh::recompute_face_normals()
{
	for(unsigned int i=0;i<triangles.size();i++)
	{
		Triangle tri=triangles[i];
		Vertex v1=vertices[tri.v1];
		Vertex v2=vertices[tri.v2];
		Vertex v3=vertices[tri.v3];
		double vec1[3];vec1[0]=v2.x-v1.x;vec1[1]=v2.y-v1.y;vec1[2]=v2.z-v1.z;
		double vec2[3];vec2[0]=v3.x-v1.x;vec2[1]=v3.y-v1.y;vec2[2]=v3.z-v1.z;
		double vec3[3];
		vec3[0]=vec1[1]*vec2[2]-vec1[2]*vec2[1];
		vec3[1]=vec1[2]*vec2[0]-vec1[0]*vec2[2];
		vec3[2]=vec1[0]*vec2[1]-vec1[1]*vec2[0];		
		double l=sqrt(vec3[0]*vec3[0]+vec3[1]*vec3[1]+vec3[2]*vec3[2]);
		face_normals[i].x=vec3[0]/l;
		face_normals[i].y=vec3[1]/l;
		face_normals[i].z=vec3[2]/l;
	}
}

void Mesh::recompute_vertex_normals()
{
	if(isTet)
	{
		for(int i=0;i<vertices.size();i++)
		{
			vtx_normals[i].x=0;
			vtx_normals[i].y=0;
			vtx_normals[i].z=0;
		}
		for(int i=0;i<tetrahedrens.size();i++)
		{
			Tet& tet=tetrahedrens[i];
			int v1=tet[0];int v2=tet[1];int v3=tet[2];int v4=tet[3];
			vtx_normals[v1].x+=face_normals[i*4].x+face_normals[i*4+1].x+face_normals[i*4+2].x;
			vtx_normals[v1].y+=face_normals[i*4].y+face_normals[i*4+1].y+face_normals[i*4+2].y;
			vtx_normals[v1].z+=face_normals[i*4].z+face_normals[i*4+1].z+face_normals[i*4+2].z;

			vtx_normals[v2].x+=face_normals[i*4].x+face_normals[i*4+1].x+face_normals[i*4+3].x;
			vtx_normals[v2].y+=face_normals[i*4].y+face_normals[i*4+1].y+face_normals[i*4+3].y;
			vtx_normals[v2].z+=face_normals[i*4].z+face_normals[i*4+1].z+face_normals[i*4+3].z;
			
			vtx_normals[v3].x+=face_normals[i*4].x+face_normals[i*4+2].x+face_normals[i*4+3].x;
			vtx_normals[v3].y+=face_normals[i*4].y+face_normals[i*4+2].y+face_normals[i*4+3].y;
			vtx_normals[v3].z+=face_normals[i*4].z+face_normals[i*4+2].z+face_normals[i*4+3].z;

			vtx_normals[v4].x+=face_normals[i*4+1].x+face_normals[i*4+2].x+face_normals[i*4+3].x;
			vtx_normals[v4].y+=face_normals[i*4+1].y+face_normals[i*4+2].y+face_normals[i*4+3].y;
			vtx_normals[v4].z+=face_normals[i*4+1].z+face_normals[i*4+2].z+face_normals[i*4+3].z;
		}
		for(int i=0;i<vertices.size();i++)
		{
			double length=sqrt(pow(vtx_normals[i].x, 2)+pow(vtx_normals[i].y, 2)+pow(vtx_normals[i].z, 2));
			vtx_normals[i].x/=length;
			vtx_normals[i].y/=length;
			vtx_normals[i].z/=length;
		}
	}
	else
	{
		for(unsigned int v_idx=0;v_idx<vertices.size();v_idx++)
		{
			double new_norm[3];new_norm[0]=0.0;new_norm[1]=0.0;new_norm[2]=0.0;
			for(int i=0;i<vtx_tri_nbrs[v_idx].size();i++){
				int face_idx=vtx_tri_nbrs[v_idx][i];
				new_norm[0]=new_norm[0]+face_normals[face_idx].x;
				new_norm[1]=new_norm[1]+face_normals[face_idx].y;
				new_norm[2]=new_norm[2]+face_normals[face_idx].z;
			}
			double l=sqrt(new_norm[0]*new_norm[0]+new_norm[1]*new_norm[1]+new_norm[2]*new_norm[2]);
			vtx_normals[v_idx].x=new_norm[0]/l;
			vtx_normals[v_idx].y=new_norm[1]/l;
			vtx_normals[v_idx].z=new_norm[2]/l;		
		}
	}
}