#include "Dynamics.h"

Dynamics::Dynamics(Mesh* mesh){ Dynamics(mesh, 1); }

Dynamics::Dynamics(Mesh* mesh, int em)
{
	N=mesh->vertices.size();
	density=mesh->density;
	energy_model=em;
	hk=0.00001;
	dampFactor=0.01;
	environment=0;
	pk.resize(N*3);
	qk.resize(N*3);
	qk1.resize(N*3);
	q.resize(N*3);
	vk.resize(N*3);
	Fext.resize(N*3);
	Fenv.resize(N*3);
	env_force_multiplier=1.0;
	lambda_k=100000;
	fix_vertices.resize(N);
	fixed_points.resize(N*3);
	for(int i=0;i<N;i++)
	{
		pk[i*3]=0;pk[i*3+1]=0;pk[i*3+2]=0;
		vk[i*3]=0;vk[i*3+1]=0;vk[i*3+2]=0;
		qk[i*3]=mesh->vertices[i].x;qk1[i*3]=qk[i*3];q[i*3]=qk[i*3];
		qk[i*3+1]=mesh->vertices[i].y;qk1[i*3+1]=qk[i*3+1];q[i*3+1]=qk[i*3+1];
		qk[i*3+2]=mesh->vertices[i].z;qk1[i*3+2]=qk[i*3+2];q[i*3+2]=qk[i*3+2];
		for(int j=0;j<3;j++)
		{
			Fext[i*3+j]=0;
			fixed_points[i*3+j]=q[i*3+j];
		}
		fix_vertices[i]=false;
	}

	compute_mass_matrix(mesh);
	CM_rest.resize(3);
	CM_rest[0]=0;CM_rest[1]=0;CM_rest[2]=0;
	for(int i=0;i<N;i++)
		for(int j=0;j<3;j++)
			CM_rest[j]=CM_rest[j]+q[i*3+j]*M[i*3+j];
	CM_rest[0]=CM_rest[0]/mass_total;CM_rest[1]=CM_rest[1]/mass_total;CM_rest[2]=CM_rest[2]/mass_total;
	shapematch_qi.resize(N*3);
	for(int i=0;i<N;i++)
		for(int j=0;j<3;j++)
			shapematch_qi[i*3+j]=q[i*3+j]-CM_rest[j];

	if(energy_model==1)
	{
		laplacian=new SparseMatrix<double, RowMajor>(N*3, N*3);		
		mesh->compute_laplacian_matrix(laplacian);
	}
	else if((energy_model>=2)&&(energy_model<=4))
	{
		tetrahedrens=&(mesh->tetrahedrens);
		tetVolumes.resize(tetrahedrens->size());
		invVs.resize(tetrahedrens->size());
		for(int i=0;i<tetrahedrens->size();i++)		
			tetVolumes[i]=mesh->compute_tetrahedren_volume_and_invV(i, invVs[i]);					
	}
}

inline void Dynamics::set_b1(double new_b1) { b1=new_b1; }

void Dynamics::compute_mass_matrix(Mesh* m)
{
	M.resize(N*3);
	invM.resize(N*3);
	mass_total=0;
	if(m->isTet){
		for(int i=0;i<N*3;i++)
			M[i]=0;

		for(int i=0;i<m->tetrahedrens.size();i++)
		{
			double tet_mass=m->tetrahedren_volume(i)*density;
			Tet& tet=m->tetrahedrens[i];
			for(int j=0;j<4;j++)
				for(int k=0;k<3;k++)
					M[tet[j]*3+k]+=tet_mass/4;
		}

		mass_total=0;
		for(int i=0;i<N*3;i++)
		{
			mass_total+=M[i];
			invM[i]=1/M[i];
		}
		mass_total/=3;
	}else{
		mass_total=0;
		for(int i=0;i<N;i++)
		{
			vector<int>* nbrs=&m->vtx_tri_nbrs[i];
			double area_total=0;
			for(int j=0;j<nbrs->size();j++)
			{
				double area=m->triangle_area(nbrs->at(j));
				area_total+=area;
			}
			double mass=area_total*density/3;
			double inv_mass=1/mass;
			M[i*3]=mass; M[i*3+1]=mass; M[i*3+2]=mass;
			invM[i*3]=inv_mass; invM[i*3+1]=inv_mass; invM[i*3+2]=inv_mass;
			mass_total=mass_total+mass;
		}	
	}
	for(int i=0;i<N*3;i++) { M[i]=1;invM[i]=1; }
}

// Computes gradW(C(qk1, qk))
void Dynamics::compute_Wdamp_gradient(VectorXd &gradWdamp)
{
	for(unsigned int i=0;i<N*3;i++)
		gradWdamp(i)=-dampFactor*vk[i];
}

double Dynamics::compute_energy_density(const VectorXd &u, int tet_id, int v1, int v2, int v3, int v4, double dx[4][3])
{
	Matrix3f V_tilda;
	for(int k=0;k<3;k++)
	{
		double v4_tilda=q[v4*3+k]+u(v4*3+k)+dx[3][k];
		V_tilda(k, 0)=q[v1*3+k]+u(v1*3+k)+dx[0][k]-v4_tilda;
		V_tilda(k, 1)=q[v2*3+k]+u(v2*3+k)+dx[1][k]-v4_tilda;
		V_tilda(k, 2)=q[v3*3+k]+u(v3*3+k)+dx[2][k]-v4_tilda;
	}
			
	Matrix3f &invV=invVs[tet_id];
	Matrix3f F;F=V_tilda*invV;
	Matrix3f C;C=F.transpose()*F;

	double w;
	if(energy_model==2){
		double I1=C.trace();
		double I3=F.determinant();
		w=a1*(I1-3)+b1*pow(I3-1, 2);
	}else if(energy_model==3){
		double I1=C.trace();
		Matrix3f Csqr;Csqr=C*C;
		double I2=Csqr.trace()-pow(I1, 2);
		w=a1*(I1-3)+b1*(I2-3);
	}else if(energy_model==4){
		Matrix3f E;E=(C-Matrix3f::Identity())/2;				
		Matrix3f E_sqr;E_sqr=E*E;
		w=a1*pow(E.trace(), 2)+b1*E_sqr.trace();
	}
	return w;
}


// Computes the gradient of the cauchy tensor with respect to the four vertices the three axis
void Dynamics::compute_cauchy_gradient(const VectorXd &u, int tet_id, int v1, int v2, int v3, int v4, double grad_output[4][3])
{	
	Matrix3f V_tilda;
	Matrix3f V_tilda_dx;
	double dx[4][3];
	for(int i=0;i<4;i++)
	{
		for(int j=0;j<3;j++)
		{
			for(int p=0;p<4;p++)
				for(int q=0;q<3;q++)
					if((p==i)&&(q==j))
						dx[p][q]=-1e-8/2;
					else
						dx[p][q]=0;

			double w1=compute_energy_density(u, tet_id, v1, v2, v3, v4, dx);
			dx[i][j]=-dx[i][j];
			double w2=compute_energy_density(u, tet_id, v1, v2, v3, v4, dx);
			double grad=(w2-w1)/(2*dx[i][j]);
			grad_output[i][j]=grad;				
		}
	}	
}

// Computes gradW(qk)
void Dynamics::compute_W_gradient(const VectorXd &u, VectorXd &gradW)
{	
	if(energy_model==1) // Laplacian energy model, only works for triangular meshes
	{
		gradW=(*laplacian)*u;	
	}
	else if((energy_model>=2)||(energy_model<=4)) 
	{
		for(int i=0;i<N*3;i++)
			gradW[i]=0;

		for(int i=0;i<tetrahedrens->size();i++)
		{
			Tet &tet=tetrahedrens->at(i);
			int v1=tet[0];int v2=tet[1];int v3=tet[2];int v4=tet[3];	
			double grad_output[4][3];
			compute_cauchy_gradient(u, i, v1, v2, v3, v4, grad_output);
			for(int j=0;j<3;j++)
			{
				gradW[v1*3+j]+=grad_output[0][j]*tetVolumes[i];
				gradW[v2*3+j]+=grad_output[1][j]*tetVolumes[i];
				gradW[v3*3+j]+=grad_output[2][j]*tetVolumes[i];
				gradW[v4*3+j]+=grad_output[3][j]*tetVolumes[i];
			}
		}

		vector<double> temp;
		temp.resize(N*3);
		for(int i=0;i<N*3;i++)
			temp[i]=gradW(i);
	}
}

void Dynamics::update_input_momentum(double qx, double qy, double qz, int idx)
{
	pk[idx*3]=qx;pk[idx*3+1]=qy;pk[idx*3+2]=qz;
}

void Dynamics::update_input_position(double qx, double qy, double qz, int idx)
{
	qk1[idx*3]=qk[idx*3];qk1[idx*3+1]=qk[idx*3+1];qk1[idx*3+2]=qk[idx*3+2];
	qk[idx*3]=qx;qk[idx*3+1]=qy;qk[idx*3+2]=qz;
}

void Dynamics::update_input_external_force(double fx, double fy, double fz, int idx)
{
	Fext[idx*3]=fx;
	Fext[idx*3+1]=fy;
	Fext[idx*3+2]=fz;
	
}

void Dynamics::update_vertex_constraints(double fix_x, double fix_y, double fix_z, int vidx)
{
	if(fix_vertices[vidx])
		fix_vertices[vidx]=false;
	else
		fix_vertices[vidx]=true;

	fixed_points[vidx*3]=fix_x;
	fixed_points[vidx*3+1]=fix_y;
	fixed_points[vidx*3+2]=fix_z;
}

void Dynamics::estimate_displacement_vector(VectorXd &u)
{
	double CM[3];CM[0]=0;CM[1]=0;CM[2]=0;
	for(int i=0;i<N;i++)
		for(int j=0;j<3;j++)
			CM[j]=CM[j]+qk[i*3+j]*M[i*3+j];
	CM[0]=CM[0]/mass_total;CM[1]=CM[1]/mass_total;CM[2]=CM[2]/mass_total;

	Matrix3d Apq;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			Apq(i, j)=0;

	for(int i=0;i<N;i++)
		for(int j=0;j<3;j++)
			for(int k=0;k<3;k++)
				Apq(j, k)=Apq(j, k)+M[i*3+j]*(qk[i*3+j]-CM[j])*shapematch_qi[i*3+k];

	SVD<Matrix3d> svd(Apq);
	Matrix3d R;Matrix3d S;
	svd.computeUnitaryPositive(&R, &S);

	// Displacement vector is u[i]=qk[i]-(R*shapematch_qi[i]+CM)
	double Rqi[3];
	for(int i=0;i<N;i++)
	{
		// Compute R*shapematch_qi[i]+CM
		Rqi[0]=CM[0];Rqi[1]=CM[1];Rqi[2]=CM[2];
		for(int j=0;j<3;j++)
			for(int k=0;k<3;k++)
				Rqi[j]=Rqi[j]+R(j, k)*shapematch_qi[i*3+k];
		
		for(int j=0;j<3;j++)
			u[i*3+j]=qk[i*3+j]-Rqi[j];
	}
}

void Dynamics::compute_environment_forces()
{
	for(int i=0;i<N;i++)
	{
		if(environment==0){
			Fenv[i*3]=0;Fenv[i*3+1]=0;Fenv[i*3+2]=0;
		}else if(environment==1){
			Fenv[i*3]=0;Fenv[i*3+2]=0;Fenv[i*3+1]=-M[i/3]*env_force_multiplier;
		}else if(environment==2){
			Fenv[i*3]=env_force_multiplier/(qk[i*3]*qk[i*3]);
			Fenv[i*3+1]=env_force_multiplier/(qk[i*3+1]*qk[i*3+1]);
			Fenv[i*3+2]=env_force_multiplier/(qk[i*3+2]*qk[i*3+2]);
			for(int j=0;j<3;j++){
				if(qk[i*3+j]>0)
					Fenv[i*3+j]=-env_force_multiplier/(qk[i*3+j]*qk[i*3+j]);
				else if(qk[i*3+j]<0)
					Fenv[i*3+j]=env_force_multiplier/(qk[i*3+j]*qk[i*3+j]);
				else
					Fenv[i*3+j]=0;				
			}
		}
	}
}

void Dynamics::update_velocity_explicit()
{
	VectorXd gradW(N*3);
	VectorXd gradWDamp(N*3);
	VectorXd u(N*3);
	estimate_displacement_vector(u);
	compute_W_gradient(u, gradW);
	compute_Wdamp_gradient(gradWDamp);
	compute_environment_forces();
	for(int i=0;i<N*3;i++)
		if(fix_vertices[i/3])
			vk[i]=invM[i]*(pk[i]-hk*gradW[i]-kD*gradWDamp[i]+hk*Fext[i]+hk*Fenv[i]+hk*lambda_k*(qk[i]-fixed_points[i]));	
		else
			vk[i]=invM[i]*(pk[i]-hk*gradW[i]-kD*gradWDamp[i]+hk*Fext[i]+hk*Fenv[i]);	

	potential_energy=compute_potential_energy(u);
	kinetic_energy=compute_kinetic_energy();
	compute_total_momentum(momentum);
}

void Dynamics::update_mometum_explicit()
{
	for(int i=0;i<N;i++)
	{		
		pk[i*3]=M[i*3]*vk[i*3];
		pk[i*3+1]=M[i*3+1]*vk[i*3+1];
		pk[i*3+2]=M[i*3+2]*vk[i*3+2];
	}
}

void Dynamics::update_position() 
{
	for(int i=0;i<N*3;i++)
	{
		double old_qk1=qk[i];
		qk[i]=qk1[i]+hk*vk[i];
		qk1[i]=old_qk1;
	}
}

void Dynamics::advance_system_explicit()
{
	update_velocity_explicit();
	update_mometum_explicit();
	update_position();
}

void Dynamics::reset()
{
	for(int i=0;i<N;i++)
	{
		pk[i*3]=0;pk[i*3+1]=0;pk[i*3+2]=0;
		vk[i*3]=0;vk[i*3+1]=0;vk[i*3+2]=0;
		qk[i*3]=q[i*3];qk1[i*3]=qk[i*3];q[i*3]=qk[i*3];
		qk[i*3+1]=q[i*3+1];qk1[i*3+1]=qk[i*3+1];q[i*3+1]=qk[i*3+1];
		qk[i*3+2]=q[i*3+2];qk1[i*3+2]=qk[i*3+2];q[i*3+2]=qk[i*3+2];
		Fext[i*3]=0;Fext[i*3+1]=0;Fext[i*3+2]=0.0;
	}
}

void Dynamics::compute_total_momentum(double total_momentum[3])
{
	total_momentum[0]=0;total_momentum[1]=0;total_momentum[2]=0;
	for(int i=0;i<N;i++)
	{
		total_momentum[0]+=vk[i*3]*M[i*3];
		total_momentum[1]+=vk[i*3+1]*M[i*3+1];
		total_momentum[2]+=vk[i*3+2]*M[i*3+2];
	}
}

double Dynamics::compute_kinetic_energy()
{
	double energy=0;
	for(int i=0;i<N*3;i++)
		energy+=vk[i]*M[i]*vk[i]/2;
	
	return energy;
}

double Dynamics::compute_potential_energy(const VectorXd &u)
{
	double dx[4][3];
	for(int i=0;i<4;i++)
		for(int j=0;j<3;j++)
			dx[i][j]=0;

	double energy=0;	

	for(int i=0;i<tetrahedrens->size();i++)
	{
		Tet &tet=tetrahedrens->at(i);
		int v1=tet[0];int v2=tet[1];int v3=tet[2];int v4=tet[3];	
		double w=compute_energy_density(u, i, v1, v2, v3, v4, dx);
	}
	return energy;
}
