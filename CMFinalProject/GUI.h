#include <stdio.h>
#include <stdlib.h>
#include <gl/glui.h>
#include "Dynamics.h"
#include "windows.h"

using namespace std;
using namespace stdext;

GLUI* glui;
int window_id;

Mesh* mesh;

int mesh_ID;

// Rendering related variables
float red_color;
float green_color;
float blue_color;
GLuint mesh_init_list;
GLuint mesh_display_list;
GLuint extforce_visualization_list;

GLfloat x_rotation_matrix[16];
GLfloat y_rotation_matrix[16];
GLfloat z_rotation_matrix[16];
GLfloat mesh_x_rotation_matrix[16];
GLfloat mesh_y_rotation_matrix[16];
GLfloat mesh_z_rotation_matrix[16];
GLfloat light_x_rotation_matrix[16];
GLfloat light_y_rotation_matrix[16];
GLfloat light_z_rotation_matrix[16];

GLfloat x_translation[1];
GLfloat y_translation[1];
GLfloat z_translation[1];
GLfloat mesh_x_translation[1];
GLfloat mesh_y_translation[1];
GLfloat mesh_z_translation[1];

float zoom_factor;

int light_rotation_enabled;
int zooming_enabled;
int rendering_options=2;
int draw_external_forces;
int draw_momentums;
int draw_fixed_points;
int mouse_click_options;
	
// Time-stepping related variables
Dynamics* dynamics;

int advance_step_count=1000;
bool animate=false;
int current_animation_step;

float step_size=0.001;
float fx, qx, fix_x;
float fy, qy, fix_y;
float fz, qz, fix_z;
int force_idx, q_idx, fix_idx;	
float new_a1, new_b1;
int energy_model;
int environment;
float environment_force_multiplier;
float damp_factor;
GLUI_StaticText* fix_point_text;

//
int selected_vtx;

// Rendering related functions
void update_color_list(int signal);
void update_force_display_list();
void update_mesh_display_list();
void update_gl_matrices();
void drawPointCloud();
void drawWireFrame();
void drawFullMesh();
void drawExtForces();
void drawMomentums();
void drawGraphics();
void redraw(int signal);
// GUI control functions
void get_pixel_coordinates(double x, double y, double z, double &pixel_x, double &pixel_y, double &pixel_z);
void select_vertex(int mouse_x, int mouse_y);
double compute_direction_and_magnitude(int mouse_x, int mouse_y, double v[3]);
void glutMouseClickFunction(int button, int button_status, int x, int y);
void glutIdleFunction();
void glutMouseMotionFunction(int x, int y);
void init(int signal);
void initGUI(int argc, char *argv[]);
// Dynamics variable update
void update_force(int signal);
void query_force(int signal);
void query_momentum(int signal);
void update_momentum(int signal);
void update_momentum_via_mouse(int mouse_x, int mouse_y);
void update_Fext_via_mouse(int mouse_x, int mouse_y);	
inline void update_environment(int signal){ dynamics->environment=environment; };
inline void update_damp_factor(int signal){ dynamics->dampFactor=damp_factor; };
inline void update_env_forc_multiplier(int signal){ dynamics->env_force_multiplier=environment_force_multiplier; }
void query_fixed_points(int signal);
void update_fixed_points(int signal);
void update_modelview_matrices(int signal);
void dynamics_advance_one_step(int signal);
void dynamics_advance(int signal);
void update_energy_model(int signal);
void update_a1_and_b1(int signal);
void dynamics_reset(int signal);
