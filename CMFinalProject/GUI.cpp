#include "GUI.h"

void update_color_list(int signal){
	glDeleteLists(mesh_init_list, 1);
	mesh_init_list=glGenLists(1);

	glNewList(mesh_init_list, GL_COMPILE);
		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);		
		GLfloat light_ambient[] = { 1.0, 1.0, 1.0, 1.0 };
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
		GLfloat mat_diffuse[] = {red_color, green_color, blue_color, 1.0};
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat_diffuse);
		GLfloat mat_ambient[] = {red_color/4, green_color/4, blue_color/4, 1.0};
		glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat_ambient);
		GLfloat mat_specular[] = {red_color/2, green_color/2, blue_color/2, 1.0};
		glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
		GLfloat mat_shininess[]={10.0};
		glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
	glEndList();
}

void update_force_display_list(){
	glDeleteLists(extforce_visualization_list, 1);
	extforce_visualization_list=glGenLists(1);
	glNewList(extforce_visualization_list, GL_COMPILE);
		glBegin(GL_LINES);
			for(int i=0;i<dynamics->N;i++)
			{
				double fx=dynamics->Fext[i*3];
				double fy=dynamics->Fext[i*3+1];
				double fz=dynamics->Fext[i*3+2];					
				if((fx!=0)||(fy!=0)||(fz!=0))
				{
					double alpha=0.1;
					glVertex3f(dynamics->qk[i*3], dynamics->qk[i*3+1], dynamics->qk[i*3+2]);
					glVertex3f(dynamics->qk[i*3]+fx*alpha, dynamics->qk[i*3+1]+fy*alpha, dynamics->qk[i*3+2]+fz*alpha);
				}
			}
		glEnd();
	glEndList();
}

void update_mesh_display_list(){
	glDeleteLists(mesh_display_list, 1);		
	mesh_display_list=glGenLists(1);					
	glNewList(mesh_display_list, GL_COMPILE);
		glBegin(GL_TRIANGLES);
			for(int i=0;i<mesh->triangles.size();i++){
				int v1=mesh->triangles[i].v1;
				int v2=mesh->triangles[i].v2;
				int v3=mesh->triangles[i].v3;
				glNormal3f(mesh->vtx_normals[v1][0], mesh->vtx_normals[v1][1], mesh->vtx_normals[v1][2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glNormal3f(mesh->vtx_normals[v2][0], mesh->vtx_normals[v2][1], mesh->vtx_normals[v2][2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glNormal3f(mesh->vtx_normals[v3][0], mesh->vtx_normals[v3][1], mesh->vtx_normals[v3][2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
			}
		glEnd();
	glEndList();		
}

void update_gl_matrices(){	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();	
	gluPerspective(45, 1.0, 0.0, 1000*mesh->boundingSphere[3]);
	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0);
	glTranslatef((mesh_x_translation[0])*mesh->boundingSphere[3]/300, (mesh_y_translation[0])*mesh->boundingSphere[3]/300, 
		(mesh_z_translation[0])*mesh->boundingSphere[3]/300-2*mesh->boundingSphere[3]);
	glMultMatrixf(mesh_x_rotation_matrix);
	glMultMatrixf(mesh_y_rotation_matrix);
	glMultMatrixf(mesh_z_rotation_matrix);	
}

void drawPointCloud()
{
	update_gl_matrices();
	glColor3f(0.2, 1.0, 0.6);
	glPointSize(2.0);
	glBegin(GL_POINTS);
		for(int i=0;i<mesh->vertices.size();i++)
			glVertex3f(dynamics->qk[i*3], dynamics->qk[i*3+1], dynamics->qk[i*3+2]);			
	glEnd();
}

void drawWireFrame()
{
	update_gl_matrices();
	glLineWidth(2.0);
	glColor3f(0.2, 1.0, 0.6);		
	if(mesh->isTet){
		glBegin(GL_LINES);
			for(int i=0;i<mesh->tetrahedrens.size();i++){
				int v1=mesh->tetrahedrens[i][0];
				int v2=mesh->tetrahedrens[i][1];
				int v3=mesh->tetrahedrens[i][2];
				int v4=mesh->tetrahedrens[i][3];
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);
			}
		glEnd();
	/*	glLineWidth(1.0);
		glColor3f(0.0, 0.2, 0.5);	
		glBegin(GL_LINES);
			for(int i=0;i<mesh->tetrahedrens.size();i++){
				int v1=mesh->tetrahedrens[i][0];
				int v2=mesh->tetrahedrens[i][1];
				int v3=mesh->tetrahedrens[i][2];
				int v4=mesh->tetrahedrens[i][3];
				glVertex3f(dynamics->q[v1*3], dynamics->q[v1*3+1], dynamics->q[v1*3+2]);
				glVertex3f(dynamics->q[v2*3], dynamics->q[v2*3+1], dynamics->q[v2*3+2]);
				glVertex3f(dynamics->q[v1*3], dynamics->q[v1*3+1], dynamics->q[v1*3+2]);
				glVertex3f(dynamics->q[v3*3], dynamics->q[v3*3+1], dynamics->q[v3*3+2]);
				glVertex3f(dynamics->q[v1*3], dynamics->q[v1*3+1], dynamics->q[v1*3+2]);
				glVertex3f(dynamics->q[v4*3], dynamics->q[v4*3+1], dynamics->q[v4*3+2]);
				glVertex3f(dynamics->q[v3*3], dynamics->q[v3*3+1], dynamics->q[v3*3+2]);
				glVertex3f(dynamics->q[v2*3], dynamics->q[v2*3+1], dynamics->q[v2*3+2]);
				glVertex3f(dynamics->q[v4*3], dynamics->q[v4*3+1], dynamics->q[v4*3+2]);
				glVertex3f(dynamics->q[v2*3], dynamics->q[v2*3+1], dynamics->q[v2*3+2]);
				glVertex3f(dynamics->q[v3*3], dynamics->q[v3*3+1], dynamics->q[v3*3+2]);
				glVertex3f(dynamics->q[v4*3], dynamics->q[v4*3+1], dynamics->q[v4*3+2]);
			}
		glEnd();
		*/
	}else{
		glBegin(GL_LINES);
			for(int i=0;i<mesh->triangles.size();i++){
				int v1=mesh->triangles[i].v1;
				int v2=mesh->triangles[i].v2;
				int v3=mesh->triangles[i].v3;
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
			}
		glEnd();
	}				
}

void drawFullMesh()
{
	glCallList(mesh_init_list);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixf(light_x_rotation_matrix);
	glMultMatrixf(light_y_rotation_matrix);
	glMultMatrixf(light_z_rotation_matrix);
	GLfloat light_direction[]={ 0.0, 0.0, mesh->boundingSphere[3], 0.0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_direction);
	update_gl_matrices();
	glBegin(GL_TRIANGLES);
		/*if(mesh->isTet){			
			for(int i=0;i<mesh->tetrahedrens.size();i++)
			{
				Tet& tet=mesh->tetrahedrens[i];
				int v1=tet[0];int v2=tet[1];int v3=tet[2];int v4=tet[3];
				glNormal3f(mesh->vtx_normals[v1][0], mesh->vtx_normals[v1][1], mesh->vtx_normals[v1][2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glNormal3f(mesh->vtx_normals[v2][0], mesh->vtx_normals[v2][1], mesh->vtx_normals[v2][2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glNormal3f(mesh->vtx_normals[v3][0], mesh->vtx_normals[v3][1], mesh->vtx_normals[v3][2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);

				glNormal3f(mesh->vtx_normals[v1][0], mesh->vtx_normals[v1][1], mesh->vtx_normals[v1][2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glNormal3f(mesh->vtx_normals[v2][0], mesh->vtx_normals[v2][1], mesh->vtx_normals[v2][2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glNormal3f(mesh->vtx_normals[v4][0], mesh->vtx_normals[v4][1], mesh->vtx_normals[v4][2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);

				glNormal3f(mesh->vtx_normals[v1][0], mesh->vtx_normals[v1][1], mesh->vtx_normals[v1][2]);
				glVertex3f(dynamics->qk[v1*3], dynamics->qk[v1*3+1], dynamics->qk[v1*3+2]);
				glNormal3f(mesh->vtx_normals[v3][0], mesh->vtx_normals[v3][1], mesh->vtx_normals[v3][2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glNormal3f(mesh->vtx_normals[v4][0], mesh->vtx_normals[v4][1], mesh->vtx_normals[v4][2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);

				glNormal3f(mesh->vtx_normals[v2][0], mesh->vtx_normals[v2][1], mesh->vtx_normals[v2][2]);
				glVertex3f(dynamics->qk[v2*3], dynamics->qk[v2*3+1], dynamics->qk[v2*3+2]);
				glNormal3f(mesh->vtx_normals[v3][0], mesh->vtx_normals[v3][1], mesh->vtx_normals[v3][2]);
				glVertex3f(dynamics->qk[v3*3], dynamics->qk[v3*3+1], dynamics->qk[v3*3+2]);
				glNormal3f(mesh->vtx_normals[v4][0], mesh->vtx_normals[v4][1], mesh->vtx_normals[v4][2]);
				glVertex3f(dynamics->qk[v4*3], dynamics->qk[v4*3+1], dynamics->qk[v4*3+2]);
			}
		}else{
		*/
			vector<Vec3d> &vtx_normals=mesh->vtx_normals;
			vector<double> &qk=dynamics->qk;
			for(int i=0;i<mesh->triangles.size();i++){
				int v1=mesh->triangles[i].v1;
				int v2=mesh->triangles[i].v2;
				int v3=mesh->triangles[i].v3;
				glNormal3f(vtx_normals[v1][0], vtx_normals[v1][1], vtx_normals[v1][2]);
				glVertex3f(qk[v1*3], qk[v1*3+1], qk[v1*3+2]);
				glNormal3f(vtx_normals[v2][0], vtx_normals[v2][1], vtx_normals[v2][2]);
				glVertex3f(qk[v2*3], qk[v2*3+1], qk[v2*3+2]);
				glNormal3f(vtx_normals[v3][0], vtx_normals[v3][1], vtx_normals[v3][2]);
				glVertex3f(qk[v3*3], qk[v3*3+1], qk[v3*3+2]);
			}
		
	glEnd();
	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
}	
void drawExtForces()
{
	glLineWidth(3.0);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
		for(int i=0;i<dynamics->N;i++)
		{
			double fx=dynamics->Fext[i*3]+dynamics->Fenv[i*3];
			double fy=dynamics->Fext[i*3+1]+dynamics->Fenv[i*3+1];
			double fz=dynamics->Fext[i*3+2]+dynamics->Fenv[i*3+2];
			if((fx!=0)||(fy!=0)||(fz!=0))
			{
				double alpha=0.3;
				glVertex3f(dynamics->qk[i*3], dynamics->qk[i*3+1], dynamics->qk[i*3+2]);
				glVertex3f(dynamics->qk[i*3]+fx*alpha, dynamics->qk[i*3+1]+fy*alpha, dynamics->qk[i*3+2]+fz*alpha);
			}
		}
	glEnd();
}

void drawMomentums()
{
	glLineWidth(3.0);
	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_LINES);
		for(int i=0;i<dynamics->N;i++)
		{
			double px=dynamics->pk[i*3];
			double py=dynamics->pk[i*3+1];
			double pz=dynamics->pk[i*3+2];				
			if((px!=0)||(py!=0)||(pz!=0))
			{
				double alpha=0.3;
				glVertex3f(dynamics->qk[i*3], dynamics->qk[i*3+1], dynamics->qk[i*3+2]);
				glVertex3f(dynamics->qk[i*3]+px*alpha, dynamics->qk[i*3+1]+py*alpha, dynamics->qk[i*3+2]+pz*alpha);
			}
		}
	glEnd();
}

void drawFixedPoints()
{
	glPointSize(10.0);
	glColor3f(1.0, 1.0, 0.0);
	glBegin(GL_POINTS);
		for(int i=0;i<dynamics->N;i++)
		{
			if(dynamics->fix_vertices[i])
			{
				double fix=dynamics->fixed_points[i*3];
				double fiy=dynamics->fixed_points[i*3+1];
				double fiz=dynamics->fixed_points[i*3+2];
				glVertex3f(fix, fiy, fiz);
			}
		}
	glEnd();
}

void drawGraphics(){
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);		
	if(rendering_options==0)
		drawPointCloud();
	else if(rendering_options==1)
		drawWireFrame();
	else if(rendering_options==2)
		drawFullMesh();

	if(draw_external_forces)
		drawExtForces();
	if(draw_momentums)
		drawMomentums();
	if(draw_fixed_points)
		drawFixedPoints();

	if(selected_vtx!=-1)
	{
		glPointSize(10.0);
		glColor3f(0.5, 1.0, 1.0);
		glBegin(GL_POINTS);
			glVertex3f(dynamics->qk[selected_vtx*3], dynamics->qk[selected_vtx*3+1], dynamics->qk[selected_vtx*3+2]);
		glEnd();
	}
	glFlush();
}

void get_pixel_coordinates(double x, double y, double z, double &pixel_x, double &pixel_y, double &pixel_z){
	GLdouble model_view[16];
	glGetDoublev(GL_MODELVIEW_MATRIX, model_view);
	GLdouble projection[16];
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT, viewport);
	GLdouble gl_pixel_x;GLdouble gl_pixel_y;GLdouble gl_pixel_z;
	gluProject(x, y, z, model_view, projection, viewport, &gl_pixel_x, &gl_pixel_y, &gl_pixel_z);
	pixel_x=(double)gl_pixel_x;pixel_y=glutGet(GLUT_WINDOW_HEIGHT)-(double)gl_pixel_y;pixel_z=gl_pixel_z;
}

void select_vertex(int mouse_x, int mouse_y)
{	
	double min_dist=999999;
	for(int i=0;i<dynamics->N;i++)
	{
		double pixel_x, pixel_y, pixel_z;
		get_pixel_coordinates(dynamics->qk[i*3], dynamics->qk[i*3+1], dynamics->qk[i*3+2], pixel_x, pixel_y, pixel_z);
		double dist=pow(pixel_x-mouse_x, 2)+pow(pixel_y-mouse_y, 2);
		if(dist<min_dist)
		{
			min_dist=dist;
			selected_vtx=i;
		}
	}				
}

double compute_direction_and_magnitude(int mouse_x, int mouse_y, double v[3])
{
	double pixel_x, pixel_y, pixel_z;
	get_pixel_coordinates(dynamics->qk[selected_vtx*3], dynamics->qk[selected_vtx*3+1], dynamics->qk[selected_vtx*3+2], pixel_x, pixel_y, pixel_z);
	double magnitude=sqrt(pow(pixel_x-mouse_x, 2)+pow(pixel_y-mouse_y, 2));
	v[0]=(mouse_x-pixel_x)/magnitude;v[1]=(mouse_y-pixel_y)/magnitude;v[2]=0;
	/*
	Matrix4f Rx;Matrix4f Ry;Matrix4f Rz;
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			Rx(i, j)=mesh_x_rotation_matrix[i*4+j];
			Ry(i, j)=mesh_y_rotation_matrix[i*4+j];
			Rz(i, j)=mesh_z_rotation_matrix[i*4+j];
		}
	}
	Matrix4f R;R=Ry*Rx;R=Rz*R;
	Matrix4f invR;invR=R.inverse();
	Vector4f vsrc;vsrc(0)=mouse_x-pixel_x;vsrc(1)=mouse_y-pixel_y;vsrc(2)=0;vsrc(3)=0;
	Vector4f vtar;vtar=R*vsrc;vtar(3)=0;
	v[0]=vtar(0)/vtar.norm();v[1]=vtar(1)/vtar.norm();v[2]=vtar(2)/vtar.norm();
	*/
	return magnitude;
}

void update_force(int signal)
{
	dynamics->update_input_external_force(fx, fy, fz, force_idx);
	glutPostRedisplay();
}

void query_force(int signal)
{
	fx=dynamics->Fext[force_idx*3];
	fy=dynamics->Fext[force_idx*3+1];
	fz=dynamics->Fext[force_idx*3+2];
	glui->sync_live();
}

void query_momentum(int signal)
{
	qx=dynamics->pk[q_idx*3];
	qy=dynamics->pk[q_idx*3+1];
	qz=dynamics->pk[q_idx*3+2];
	glui->sync_live();
}

void update_momentum(int signal)
{
	dynamics->update_input_momentum(qx, qy, qz, q_idx);
	glutPostRedisplay();
}

void update_Fext_via_mouse(int mouse_x, int mouse_y)
{
	double dir[3];
	double length=compute_direction_and_magnitude(mouse_x, mouse_y, dir);
	length=length/70;
	fx=dir[0]*length;fy=dir[1]*length;fz=dir[2]*length;force_idx=selected_vtx;
	glui->sync_live();
	update_force(0);
}

void update_momentum_via_mouse(int mouse_x, int mouse_y)
{
	double dir[3];
	double length=compute_direction_and_magnitude(mouse_x, mouse_y, dir);
	length=length/70;
	qx=dir[0]*length;qy=dir[1]*length;qz=dir[2]*length;q_idx=selected_vtx;
	glui->sync_live();
	update_momentum(0);
}

void query_fixed_points(int signal)
{
	fix_x=dynamics->fixed_points[fix_idx*3];
	fix_y=dynamics->fixed_points[fix_idx*3+1];
	fix_z=dynamics->fixed_points[fix_idx*3+2];
	if(dynamics->fix_vertices[fix_idx])
		fix_point_text->set_text("Fixed");
	else
		fix_point_text->set_text("Unfixed");

	glui->sync_live();
}

void update_fixed_points(int signal)
{
	dynamics->update_vertex_constraints(fix_x, fix_y, fix_z, fix_idx);
	glutPostRedisplay();
}

void glutMouseClickFunction(int button, int button_status, int x, int y){
	if((button==GLUT_LEFT_BUTTON)&&(button_status==GLUT_DOWN))
	{
		if(mouse_click_options==0)
			selected_vtx=-1;
		else if(mouse_click_options==1)
			select_vertex(x, y);
		else if((mouse_click_options==2)&&(selected_vtx!=-1))
			update_Fext_via_mouse(x, y);
		else if((mouse_click_options==3)&&(selected_vtx!=-1))
			update_momentum_via_mouse(x, y);
	}
	else if(button_status==GLUT_UP)
	{
		if((button==GLUT_WHEEL_UP)&&(zooming_enabled))
			zoom_factor=zoom_factor-0.03;								
		else if((button==GLUT_WHEEL_DOWN)&&(zooming_enabled))
			zoom_factor=zoom_factor+0.03;											
	}
	glutPostRedisplay();
}

void glutMouseMotionFunction(int x, int y){
}

void glutIdleFunction()
{
	if(animate)
	{			
		dynamics->advance_system_explicit();
		if(current_animation_step>=advance_step_count){
			animate=false;
		}else{
			current_animation_step++;
			Sleep(1);
		}
	}
	glutSetWindow(window_id);
	glutPostRedisplay();
}

void redraw(int signal) { 
	if(rendering_options==2){
		mesh->recompute_face_normals();
		mesh->recompute_vertex_normals();
	}
	glutPostRedisplay(); 
}

void update_modelview_matrices(int signal){
	if(light_rotation_enabled){
		for(int i=0;i<16;i++){
			light_x_rotation_matrix[i]=x_rotation_matrix[i];
			light_y_rotation_matrix[i]=y_rotation_matrix[i];
			light_z_rotation_matrix[i]=z_rotation_matrix[i];
		}	
	}else{
		for(int i=0;i<16;i++){
			mesh_x_rotation_matrix[i]=x_rotation_matrix[i];
			mesh_y_rotation_matrix[i]=y_rotation_matrix[i];
			mesh_z_rotation_matrix[i]=z_rotation_matrix[i];
		}
		mesh_x_translation[0]=x_translation[0];
		mesh_y_translation[0]=y_translation[0];
		mesh_z_translation[0]=z_translation[0];
	}
}


// Time-stepping related functions
void dynamics_advance_one_step(int signal)
{
	dynamics->advance_system_explicit();
	update_color_list(0);
	glutPostRedisplay();
}

void dynamics_advance(int signal)
{
	if(animate){
		animate=false;
	}else{
		current_animation_step=0;
		animate=true;
	}
}


inline void update_step_size(int signal){ dynamics->hk=step_size; }

inline void update_a1_and_b1(int signal) { dynamics->a1=new_a1; dynamics->b1=new_b1; }

inline void update_energy_model(int signal) { dynamics->energy_model=energy_model+2; }
void dynamics_reset(int signal) { 
	dynamics->reset(); 
	glutPostRedisplay();
}

void init(int signal){
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);	
	red_color=0.0;
	green_color=1.0;
	blue_color=0.6;
	for(int i=0;i<16;i++){
		x_rotation_matrix[i]=0.0;y_rotation_matrix[i]=0.0;z_rotation_matrix[i]=0.0;
		mesh_x_rotation_matrix[i]=0.0;mesh_y_rotation_matrix[i]=0.0;mesh_z_rotation_matrix[i]=0.0;
		light_x_rotation_matrix[i]=0.0;light_y_rotation_matrix[i]=0.0;light_z_rotation_matrix[i]=0.0;
	}
	x_rotation_matrix[0]=1.0;x_rotation_matrix[5]=1.0;x_rotation_matrix[10]=1.0;x_rotation_matrix[15]=1.0;
	y_rotation_matrix[0]=1.0;y_rotation_matrix[5]=1.0;y_rotation_matrix[10]=1.0;y_rotation_matrix[15]=1.0;
	z_rotation_matrix[0]=1.0;z_rotation_matrix[5]=1.0;z_rotation_matrix[10]=1.0;z_rotation_matrix[15]=1.0;
	mesh_x_rotation_matrix[0]=1.0;mesh_x_rotation_matrix[5]=1.0;mesh_x_rotation_matrix[10]=1.0;mesh_x_rotation_matrix[15]=1.0;
	mesh_y_rotation_matrix[0]=1.0;mesh_y_rotation_matrix[5]=1.0;mesh_y_rotation_matrix[10]=1.0;mesh_y_rotation_matrix[15]=1.0;
	mesh_z_rotation_matrix[0]=1.0;mesh_z_rotation_matrix[5]=1.0;mesh_z_rotation_matrix[10]=1.0;mesh_z_rotation_matrix[15]=1.0;
	light_x_rotation_matrix[0]=1.0;light_x_rotation_matrix[5]=1.0;light_x_rotation_matrix[10]=1.0;light_x_rotation_matrix[15]=1.0;
	light_y_rotation_matrix[0]=1.0;light_y_rotation_matrix[5]=1.0;light_y_rotation_matrix[10]=1.0;light_y_rotation_matrix[15]=1.0;
	light_z_rotation_matrix[0]=1.0;light_z_rotation_matrix[5]=1.0;light_z_rotation_matrix[10]=1.0;light_z_rotation_matrix[15]=1.0;
	mesh_x_translation[0]=0.0;mesh_y_translation[0]=0.0;mesh_z_translation[0]=0.0;	
	x_translation[0]=0.0;y_translation[0]=0.0;z_translation[0]=0.0;
	zoom_factor=0.6;
	light_rotation_enabled=0;
	zooming_enabled=1;
	draw_external_forces=1;
	draw_momentums=1;
	draw_fixed_points=1;
	step_size=0.001;
	mouse_click_options=0;
	selected_vtx=-1;
	energy_model=2;
	advance_step_count=1000;
	animate=false;
	glui->sync_live();
	delete mesh;
	mesh=new Mesh(mesh_ID+1);
	
	if(dynamics!=NULL)
		delete dynamics;
	dynamics=new Dynamics(mesh, 4);
	dynamics->a1=1000.0;dynamics->b1=1.0;		
	new_a1=dynamics->a1;new_b1=dynamics->b1;

	mesh->compute_bounding_sphere();		
	update_color_list(0);
}

void initGUI(int argc, char *argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize (600, 600); 
	glutInitWindowPosition (0, 0);		
	window_id=glutCreateWindow ("Simulator");
	glutSetWindow(window_id);
	glui=GLUI_Master.create_glui("Control Panel");
	GLUI_Master.set_glutIdleFunc(glutIdleFunction);
	glui->set_main_gfx_window(window_id);

	GLUI_Listbox *mesh_loader=glui->add_listbox("Select Mesh: ", &mesh_ID, 0, init);
	mesh_loader->add_item(0, "single tet");
	mesh_loader->add_item(1, "cube");
	mesh_loader->add_item(2, "rod");		

	GLUI_Panel *display_panel=glui->add_panel("Display & Control Options");
	GLUI_Panel *hidden_disp_panel=glui->add_panel_to_panel(display_panel, "x", GLUI_PANEL_NONE);
	glui->add_column_to_panel(hidden_disp_panel, false);		
	GLUI_Panel *color_panel=glui->add_panel_to_panel(hidden_disp_panel, "Display Options");
	glui->add_checkbox_to_panel(color_panel, "Draw Forces?", &draw_external_forces);
	glui->add_checkbox_to_panel(color_panel, "Draw Momentums?", &draw_momentums);
	glui->add_checkbox_to_panel(color_panel, "Draw Fixed Points?", &draw_fixed_points);
	glui->add_checkbox_to_panel(color_panel, "Enable Zooming? ", &zooming_enabled);
	glui->add_checkbox_to_panel(color_panel, "Rotate Light? ", &light_rotation_enabled);
	glui->add_column_to_panel(color_panel, false);
	GLUI_Panel *hidden_color_panel=glui->add_panel_to_panel(color_panel, "Rendering Options");
	GLUI_RadioGroup* radio_group=glui->add_radiogroup_to_panel(hidden_color_panel, &rendering_options, 0, redraw);
	glui->add_radiobutton_to_group(radio_group, "Point Cloud");
	glui->add_radiobutton_to_group(radio_group, "Wire Frame");
	glui->add_radiobutton_to_group(radio_group, "Full Mesh");						
	
	GLUI_Panel *rotation_panel=glui->add_panel_to_panel(display_panel, "Modeling Options");		
	GLUI_Panel *hidden_rotation_panel=glui->add_panel_to_panel(rotation_panel, "NULL", GLUI_PANEL_NONE);
	glui->add_rotation_to_panel(hidden_rotation_panel, "X Rotation", x_rotation_matrix, 0, update_modelview_matrices);
	glui->add_column_to_panel(hidden_rotation_panel, false);
	glui->add_column_to_panel(hidden_rotation_panel, false);
	glui->add_rotation_to_panel(hidden_rotation_panel, "Y Rotation", y_rotation_matrix, 0, update_modelview_matrices);
	glui->add_column_to_panel(hidden_rotation_panel, false);
	glui->add_column_to_panel(hidden_rotation_panel, false);
	glui->add_rotation_to_panel(hidden_rotation_panel, "Z Rotation", z_rotation_matrix, 0, update_modelview_matrices);
	GLUI_Panel *hidden_translation_panel=glui->add_panel_to_panel(rotation_panel, "NULL", GLUI_PANEL_NONE);
	glui->add_translation_to_panel(hidden_translation_panel, "X Translation", GLUI_TRANSLATION_X, x_translation, 0, update_modelview_matrices);
	glui->add_column_to_panel(hidden_translation_panel, false);
	glui->add_translation_to_panel(hidden_translation_panel, "Y Translation", GLUI_TRANSLATION_Y, y_translation, 0, update_modelview_matrices);
	glui->add_column_to_panel(hidden_translation_panel, false);
	glui->add_translation_to_panel(hidden_translation_panel, "Z Translation", GLUI_TRANSLATION_Z, z_translation, 0, update_modelview_matrices);

	GLUI_Panel *timestepping_panel=glui->add_panel("Time Stepping Options");
	GLUI_Panel *timestepping_sub_panel1=glui->add_panel_to_panel(timestepping_panel, "Time Stepping Controls");
	GLUI_Panel *selection_panel=glui->add_panel_to_panel(timestepping_sub_panel1, "Environment");
	GLUI_RadioGroup* radio_group2=glui->add_radiogroup_to_panel(selection_panel, &environment, 0, update_environment);
	glui->add_radiobutton_to_group(radio_group2, "None");
	glui->add_radiobutton_to_group(radio_group2, "Uniform Gravity");
	glui->add_radiobutton_to_group(radio_group2, "Central Monopole");
	glui->add_edittext_to_panel(timestepping_sub_panel1, "Environment Force Multiplier: ", GLUI_EDITTEXT_FLOAT, &environment_force_multiplier, 0, update_env_forc_multiplier);
	glui->add_edittext_to_panel(timestepping_sub_panel1, "Damp Factor", GLUI_EDITTEXT_FLOAT, &damp_factor, 0, update_damp_factor);
	GLUI_Panel *energy_panel=glui->add_panel_to_panel(timestepping_sub_panel1, "Energy Models");
	GLUI_RadioGroup* radio_group3=glui->add_radiogroup_to_panel(energy_panel, &energy_model, 0, update_energy_model);
	glui->add_radiobutton_to_group(radio_group3, "Neo-Hookean");
	glui->add_radiobutton_to_group(radio_group3, "Mooney-Rivlin");
	glui->add_radiobutton_to_group(radio_group3, "Saint Venant-Kirchhoff");
	glui->add_column_to_panel(timestepping_sub_panel1, true);
	glui->add_edittext_to_panel(timestepping_sub_panel1, "Step size: ", GLUI_EDITTEXT_FLOAT, &step_size, 0, update_step_size);
	glui->add_button_to_panel(timestepping_sub_panel1, "Advance One Step", 0, dynamics_advance_one_step);
	glui->add_edittext_to_panel(timestepping_sub_panel1, "Advance Step Count: ", GLUI_EDITTEXT_INT, &advance_step_count);
	glui->add_button_to_panel(timestepping_sub_panel1, "Advance", 0, dynamics_advance);	
	glui->add_button_to_panel(timestepping_sub_panel1, "Reset System", 0, dynamics_reset);
	glui->add_edittext_to_panel(timestepping_sub_panel1, "a1: ", GLUI_EDITTEXT_FLOAT, &new_a1, 0, update_a1_and_b1);
	glui->add_edittext_to_panel(timestepping_sub_panel1, "b1: ", GLUI_EDITTEXT_FLOAT, &new_b1, 0, update_a1_and_b1);

	GLUI_Panel *timestepping_sub_panel2=glui->add_panel_to_panel(timestepping_panel, "Force & Momentum Updates");
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fext_x: ", GLUI_EDITTEXT_FLOAT, &fx, 0, update_force);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fext_y: ", GLUI_EDITTEXT_FLOAT, &fy, 0, update_force);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fext_z: ", GLUI_EDITTEXT_FLOAT, &fz, 0, update_force);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fext_idx: ", GLUI_EDITTEXT_INT, &force_idx, 0, query_force);
	glui->add_column_to_panel(timestepping_sub_panel2, true);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Momentum_x: ", GLUI_EDITTEXT_FLOAT, &qx, 0, update_momentum);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Momentum_y: ", GLUI_EDITTEXT_FLOAT, &qy, 0, update_momentum);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Momemtum_z: ", GLUI_EDITTEXT_FLOAT, &qz, 0, update_momentum);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Momentum_idx: ", GLUI_EDITTEXT_INT, &q_idx, 0, query_momentum);	
	glui->add_column_to_panel(timestepping_sub_panel2, true);
	fix_point_text=glui->add_statictext_to_panel(timestepping_sub_panel2, "Unfixed");
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fix_x: ", GLUI_EDITTEXT_FLOAT, &fix_x);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fix_y: ", GLUI_EDITTEXT_FLOAT, &fix_y);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fix_z: ", GLUI_EDITTEXT_FLOAT, &fix_z);
	glui->add_edittext_to_panel(timestepping_sub_panel2, "Fix_idx: ", GLUI_EDITTEXT_INT, &fix_idx, 0, query_fixed_points);	
	glui->add_button_to_panel(timestepping_sub_panel2, "Fix/Unfix", 0, update_fixed_points);

	GLUI_Master.set_glutMouseFunc(glutMouseClickFunction);
	glutDisplayFunc(drawGraphics); 
	glutMotionFunc(glutMouseMotionFunction);	

	init(0);
}

int main(int argc, char *argv[])
{	
	initGUI(argc, argv);
	glutMainLoop();
	return 0;
}