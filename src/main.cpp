/*
CPE/CSC 474 Lab base code Eckhardt/Dahl
based on CPE/CSC 471 Lab base code Wood/Dunn/Eckhardt
*/

#include <iostream>
#include <fstream>
#include <glad/glad.h>



#include "GLSL.h"
#include "Program.h"
#include "WindowManager.h"
#include "Shape.h"
#include "skmesh.h"
#include "line.h"
// value_ptr for glm
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

// assimp
#include "assimp/Importer.hpp"
#include "assimp/postprocess.h"
#include "assimp/scene.h"
#include "assimp/vector3.h"
#include "assimp/scene.h"
#include <assimp/mesh.h>

// BOIDS
#include "Swarm.h"
#define WORLD_SIZE 30

ofstream file;

using namespace std;
using namespace glm;
using namespace Assimp;
mat4 linint_between_two_orientations(mat4 m1, mat4 m2, float t);

double get_last_elapsed_time()
{
	static double lasttime = glfwGetTime();
	double actualtime =glfwGetTime();
	double difference = actualtime- lasttime;
	lasttime = actualtime;
	return difference;
}

float cosint(float t){
    return 1 - (cos(t*3.1415)+1.0f)/2.0f;
}

mat4 linint_between_two_orientations(mat4 m1, mat4 m2, float t)
    {
    quat q1, q2;
    
    q1 = quat(m1);
    q2 = quat(m2);
    quat qt = slerp(q1, q2, t); //<---
    qt = normalize(qt);
    mat4 mt = mat4(qt);
    //mt = transpose(mt);         //<---
    return mt;
    }

mat4 create_orientations (vec3 ez_aka_lookto_1, vec3 ey_aka_up_1) {
    mat4 m1;
    quat q1;
    vec3 ex, ey, ez;
    ey = ey_aka_up_1;
    ez = ez_aka_lookto_1;
    ex = cross(ey, ez);
    m1[0][0] = ex.x;        m1[0][1] = ex.y;        m1[0][2] = ex.z;        m1[0][3] = 0;
    m1[1][0] = ey.x;        m1[1][1] = ey.y;        m1[1][2] = ey.z;        m1[1][3] = 0;
    m1[2][0] = ez.x;        m1[2][1] = ez.y;        m1[2][2] = ez.z;        m1[2][3] = 0;
    m1[3][0] = 0;            m1[3][1] = 0;            m1[3][2] = 0;            m1[3][3] = 1.0f;
    
    return m1;
}

class camera
{
public:
	glm::vec3 pos, rot;
	int w, a, s, d, q, e, z, c;
    glm::mat4 R;
	camera()
	{
        w = a = s = d = q = e = z = c = 0;;
		pos = glm::vec3(0, 0, 0);
		rot = glm::vec3(0, 0, 0);
	}

    glm::mat4 process(double ftime)
    {
        float speed = 0;

        float fwdspeed = 20;
        /*if (realspeed)
            fwdspeed = 8;*/

        if (w == 1)
        {
            speed = fwdspeed*ftime;
        }
        else if (s == 1)
        {
            speed = -fwdspeed*ftime;
        }
        float yangle=0;
        if (a == 1)
            yangle = -1*ftime;
        else if(d==1)
            yangle = 1*ftime;
        rot.y += yangle;
        float zangle = 0;
        if (q == 1)
            zangle = -1 * ftime;
        else if (e == 1)
            zangle = 1 * ftime;
        rot.z += zangle;
        float xangle = 0;
        if (z == 1)
            xangle = -0.1 * ftime;
        else if (c == 1)
            xangle = 0.1 * ftime;
        rot.x += xangle;

        glm::mat4 Ry = glm::rotate(glm::mat4(1), rot.y, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1), rot.z, glm::vec3(0, 0, 1));
        glm::mat4 Rx = glm::rotate(glm::mat4(1), rot.x, glm::vec3(1, 0, 0));
        glm::vec4 dir = glm::vec4(0, 0, speed,1);
        R = Rz * Rx  * Ry;
        dir = dir*R;
        pos += glm::vec3(dir.x, dir.y, dir.z);
        glm::mat4 T = glm::translate(glm::mat4(1), pos);
        return R*T;
    }
    
    void get_dirpos(vec3 &up,vec3 &dir,vec3 &position)
        {
        position = pos;
        glm::mat4 Ry = glm::rotate(glm::mat4(1), rot.y, glm::vec3(0, 1, 0));
        glm::mat4 Rz = glm::rotate(glm::mat4(1), rot.z, glm::vec3(0, 0, 1));
        glm::mat4 Rx = glm::rotate(glm::mat4(1), rot.x, glm::vec3(1, 0, 0));
        glm::vec4 dir4 = glm::vec4(0, 0, 1, 0);
        //R = Rz * Rx  * Ry;
        dir4 = dir4*R;
        dir = vec3(dir4);
        glm::vec4 up4 = glm::vec4(0, 1, 0, 0);
        up4 = R*vec4(0, 1, 0, 0);
        up4 = vec4(0, 1, 0, 0)*R;
        up = vec3(up4);
        }
};

camera mycam;

class Application : public EventCallbacks
{
public:
	WindowManager * windowManager = nullptr;

	// Our shader program
    std::shared_ptr<Program> psky, skinProg, prog, cubeProg, plantsProg;

	// Contains vertex information for OpenGL
	GLuint VertexArrayID;

	// Data necessary to give our box to OpenGL
	GLuint VertexBufferID, VertexNormDBox, VertexTexBox, IndexBufferIDBox;

	//texture data
	GLuint Texture;
	GLuint Texture2;

	// skinnedMesh
	SkinnedMesh skmesh;

	// textures
	shared_ptr<SmartTexture> skyTex, chihiroTex, chihiroTex1, flowerTex, grassTex, treeTex;

	// shapes
    shared_ptr<Shape> skyShape, cube, flower, tree, chihiro;
    
    // SWARM
    Swarm* swarm;
    
    //line
    Line linerender;
    Line smoothrender;
    vector<vec3> line;
    vector<vec3> splinepoints;
    
    //orientations 1
    vector<vec3> ey;
    vector<vec3> ez;
    vector<mat4> marr;
    
    //line camera
    Line linerender_cam;
    Line smoothrender_cam;
    vector<vec3> line_cam;
    vector<vec3> splinepoints_cam;
    
    //camera orientations
    vector<vec3> ey_cam;
    vector<vec3> ez_cam;
    vector<mat4> marr_cam;
    
    
    //sky box
    vector<string> faces {
        "bloodynight_lf.png", // left face
        "bloodynight_rt.png", // right face
        "bloodynight_up.png", // top face
        "bloodynight_dn.png", // bottom face
        "bloodynight_ft.png", // front face
        "bloodynight_bk.png", // back face
    };
    
    unsigned int cubeMapTexture;
    
    vector<vec3> flowerPos, treePos;
	
	void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
	{
		if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		{
			glfwSetWindowShouldClose(window, GL_TRUE);
		}
		
		if (key == GLFW_KEY_W && action == GLFW_PRESS)
		{
			mycam.w = 1;
		}
		else if (key == GLFW_KEY_W && action == GLFW_RELEASE)
		{
			mycam.w = 0;
		}
		if (key == GLFW_KEY_S && action == GLFW_PRESS)
		{
			mycam.s = 1;
		}
		else if (key == GLFW_KEY_S && action == GLFW_RELEASE)
		{
			mycam.s = 0;
		}
		if (key == GLFW_KEY_A && action == GLFW_PRESS)
		{
			mycam.a = 1;
		}
		else if (key == GLFW_KEY_A && action == GLFW_RELEASE)
		{
			mycam.a = 0;
		}
		if (key == GLFW_KEY_D && action == GLFW_PRESS)
		{
			mycam.d = 1;
		}
		else if (key == GLFW_KEY_D && action == GLFW_RELEASE)
		{
			mycam.d = 0;
		}
        if (key == GLFW_KEY_Q && action == GLFW_PRESS)
        {
            mycam.q = 1;
        }
        if (key == GLFW_KEY_Q && action == GLFW_RELEASE)
        {
            mycam.q = 0;
        }
        if (key == GLFW_KEY_E && action == GLFW_PRESS)
        {
            mycam.e = 1;
        }
        if (key == GLFW_KEY_E && action == GLFW_RELEASE)
        {
            mycam.e = 0;
        }
        if (key == GLFW_KEY_Z && action == GLFW_PRESS)
        {
            mycam.z = 1;
        }
        if (key == GLFW_KEY_Z && action == GLFW_RELEASE)
        {
            mycam.z = 0;
        }
        if (key == GLFW_KEY_C && action == GLFW_PRESS)
        {
            mycam.c = 1;
        }
        if (key == GLFW_KEY_C && action == GLFW_RELEASE)
        {
            mycam.c = 0;
        }

		if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
		{
			mycam.pos = vec3(mycam.pos.x, mycam.pos.y-0.1, mycam.pos.z);
		}
        
        if (key == GLFW_KEY_BACKSPACE && action == GLFW_PRESS)
            {
            vec3 dir, pos, up;
            mycam.get_dirpos(up, dir, pos);
            cout << endl;
            cout << "point position:" << pos.x << "," << pos.y << "," << pos.z << endl;
            cout << "Zbase:" << dir.x << "," << dir.y << "," << dir.z << endl;
            cout << "Ybase:" << up.x << "," << up.y << "," << up.z << endl;
            cout << "point saved into file!" << endl << endl;
            file << "line.push_back(vec3(" << pos.x * -1 << "," << pos.y * -1 << "," << pos.z * -1 << "));" << endl;
            file << "ez.push_back(vec3(" << dir.x << "," << dir.y << "," << dir.z << "));" << endl;
            file << "ey.push_back(vec3(" << up.x << "," << up.y << "," << up.z << ")); \n" << endl;
            }
	}

	// callback for the mouse when clicked move the triangle when helper functions
	// written
	void mouseCallback(GLFWwindow *window, int button, int action, int mods)
	{
		double posX, posY;
		float newPt[2];
		if (action == GLFW_PRESS)
		{
			glfwGetCursorPos(window, &posX, &posY);
			std::cout << "Pos X " << posX <<  " Pos Y " << posY << std::endl;

			//change this to be the points converted to WORLD
			//THIS IS BROKEN< YOU GET TO FIX IT - yay!
			newPt[0] = 0;
			newPt[1] = 0;

			std::cout << "converted:" << newPt[0] << " " << newPt[1] << std::endl;
			glBindBuffer(GL_ARRAY_BUFFER, VertexBufferID);
			//update the vertex array with the updated points
			glBufferSubData(GL_ARRAY_BUFFER, sizeof(float)*6, sizeof(float)*2, newPt);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
	}

	//if the window is resized, capture the new size and reset the viewport
	void resizeCallback(GLFWwindow *window, int in_width, int in_height)
	{
		//get the window size - may be different then pixels for retina
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		glViewport(0, 0, width, height);
	}
	/*Note that any gl calls must always happen after a GL state is initialized */
	
	
	/*Note that any gl calls must always happen after a GL state is initialized */
	void initGeom(const std::string& resourceDirectory)
	{
        // CHIHIRO
        if (!skmesh.LoadMesh(resourceDirectory + "/Hip Hop Dancing.fbx")) {
			printf("Mesh load failed\n");
			return;
			}
        auto strChar = resourceDirectory + "/Chihiro_happy.png"; ///PolySphere3_TXTR.png
        chihiroTex = SmartTexture::loadTexture(strChar, true);
        if (!chihiroTex)
            cerr << "error: texture " << strChar << " not found" << endl;
        
        chihiro = make_shared<Shape>();
        chihiro->loadMesh(resourceDirectory + "/Sen.obj");
        chihiro->resize();
        chihiro->init();
        auto strChar1 = resourceDirectory + "/Chihiro_surprised.png";
        chihiroTex1 = SmartTexture::loadTexture(strChar1, true);
        if (!chihiroTex1)
            cerr << "error: texture " << strChar << " not found" << endl;
        
        cube = make_shared<Shape>();
        cube->loadMesh(resourceDirectory + "/cube.obj");
        cube->resize();
        cube->init();
        
        tree = make_shared<Shape>();
        //string mtl_path = resourceDirectory + "/acacia/";
        tree->loadMesh(resourceDirectory + "/acacia/acacia.obj");
        tree->resize();
        tree->init();
        
        auto strTree = resourceDirectory + "/acacia/textures/acacia leaf.png";
        treeTex = SmartTexture::loadTexture(strTree, true);
        if (!treeTex)
            cerr << "error: texture " << strTree << " not found" << endl;
        
        /*tree = make_shared<Shape>();
        tree->loadMesh(resourceDirectory + "/TREE(low poly).obj");
        tree->resize();
        tree->init();
        auto strTree = resourceDirectory + "/TREE_Mat_1_BaseColor.png";
        treeTex = SmartTexture::loadTexture(strTree, true);
        if (!treeTex)
            cerr << "error: texture " << strTree << " not found" << endl;*/
        
        auto strGrass = resourceDirectory + "/skybox/bloodynight_dn.png";
        grassTex = SmartTexture::loadTexture(strGrass, true);
        if (!grassTex)
            cerr << "error: texture " << strGrass << " not found" << endl;
        
        flower = make_shared<Shape>();
        flower->loadMesh(resourceDirectory + "/mayflower/model.obj");
        flower->resize();
        flower->init();
        auto strFlower = resourceDirectory + "/mayflower/texture.png";
        flowerTex = SmartTexture::loadTexture(strFlower, true);
        if (!flowerTex)
            cerr << "error: texture " << strFlower << " not found" << endl;
        
        // init random plant positions
        for (int i = 0; i < 50; i++) {
            int randX = (rand() % 30) + -15;
            int randZ = (rand() % 30) + -15;
            flowerPos.push_back(vec3(randX, 0, randZ));
        }
        /*for (int i = 0; i < 25; i++) {
            int randX = 10 + (rand() % 15);
            int randZ = 10 + (rand() % 15);
            treePos.push_back(vec3(randX, 0, randZ));
        }
        for (int i = 0; i < 25; i++) {
            int randX = -10 - (rand() % 15);
            int randZ = -10 - (rand() % 15);
            treePos.push_back(vec3(randX, 0, randZ));
        }*/
        for (int i = 0; i < 50; i++) {
            int randX = (rand() % 100) - 50;
            int randZ = (rand() % 100) - 50;
            treePos.push_back(vec3(randX, 0, randZ));
        }

		// Initialize mesh.
		skyShape = make_shared<Shape>();
		skyShape->loadMesh(resourceDirectory + "/sphere.obj");
		skyShape->resize();
		skyShape->init();

		// sky texture
		/*auto strSky = resourceDirectory + "/sky.jpg";
		skyTex = SmartTexture::loadTexture(strSky, true);
		if (!skyTex)
			cerr << "error: texture " << strSky << " not found" << endl;*/
        
        //generate the VAO
        glGenVertexArrays(1, &VertexArrayID);
        glBindVertexArray(VertexArrayID);

        //generate vertex buffer to hand off to OGL
        glGenBuffers(1, &VertexBufferID);
        //set the current state to focus on our vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, VertexBufferID);

        GLfloat cube_vertices[] = {
            // front
            -1.0, -1.0,  1.0,//LD
            1.0, -1.0,  1.0,//RD
            1.0,  1.0,  1.0,//RU
            -1.0,  1.0,  1.0,//LU
        };
        //make it a bit smaller
        for (int i = 0; i < 12; i++)
            cube_vertices[i] *= 0.5;
        //actually memcopy the data - only do this once
        glBufferData(GL_ARRAY_BUFFER, sizeof(cube_vertices), cube_vertices, GL_DYNAMIC_DRAW);

        //we need to set up the vertex array
        glEnableVertexAttribArray(0);
        //key function to get up how many elements to pull out at a time (3)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        //color
        GLfloat cube_norm[] = {
            // front colors
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,

        };
        glGenBuffers(1, &VertexNormDBox);
        //set the current state to focus on our vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, VertexNormDBox);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cube_norm), cube_norm, GL_STATIC_DRAW);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);

        //color
        glm::vec2 cube_tex[] = {
            // front colors
            glm::vec2(0.0, 1.0),
            glm::vec2(1.0, 1.0),
            glm::vec2(1.0, 0.0),
            glm::vec2(0.0, 0.0),

        };
        glGenBuffers(1, &VertexTexBox);
        //set the current state to focus on our vertex buffer
        glBindBuffer(GL_ARRAY_BUFFER, VertexTexBox);
        glBufferData(GL_ARRAY_BUFFER, sizeof(cube_tex), cube_tex, GL_STATIC_DRAW);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, (void*)0);

        glGenBuffers(1, &IndexBufferIDBox);
        //set the current state to focus on our vertex buffer
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferIDBox);
        GLushort cube_elements[] = {

            // front
            0, 1, 2,
            2, 3, 0,
        };
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cube_elements), cube_elements, GL_STATIC_DRAW);

        
        glBindVertexArray(0);

    

        int width, height, channels;
        char filepath[1000];

        //texture 1
        string str = resourceDirectory + "/fly-angle1.png";
        strcpy(filepath, str.c_str());
        unsigned char* data = stbiload(filepath, &width, &height, &channels, 4);
        glGenTextures(1, &Texture);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
        //texture 2
        str = resourceDirectory + "/rest-angle.png";
        strcpy(filepath, str.c_str());
        data = stbiload(filepath, &width, &height, &channels, 4);
        glGenTextures(1, &Texture2);
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, Texture2);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);

        //[TWOTEXTURES]
        //set the 2 textures to the correct samplers in the fragment shader:
        GLuint Tex1Location = glGetUniformLocation(prog->pid, "tex");//tex, tex2... sampler in the fragment shader
        GLuint Tex2Location = glGetUniformLocation(prog->pid, "tex2");
        // Then bind the uniform samplers to texture units:
        glUseProgram(prog->pid);
        glUniform1i(Tex1Location, 0);
        glUniform1i(Tex2Location, 1);
        
        // chihiro PATH - SCENE 1
        /*smoothrender.init();
        linerender.init();
        line.push_back(vec3(-1.68961,0,-27.5789));
        ez.push_back(vec3(0.396589,0,0.917996));
        ey.push_back(vec3(0,1,0));
        
        line.push_back(vec3(3.53032,0,-15.4962));
        ez.push_back(vec3(0.396589,0,0.917996));
        ey.push_back(vec3(0,1,0));
        
        line.push_back(vec3(5.37346,0,-11.2298));
        ez.push_back(vec3(-0.192946,0,0.981209));
        ey.push_back(vec3(0,1,0));
        
        line.push_back(vec3(4.19675,0,-5.24575));
        ez.push_back(vec3(-0.192946,0,0.981209));
        ey.push_back(vec3(0,1,0));
        
        line.push_back(vec3(-0.266923,0,4.12222));
        ez.push_back(vec3(-0.257008,0,0.966409));
        ey.push_back(vec3(0,1,0));
        
        line.push_back(vec3(-0.266923,0,4.12222));
        linerender.re_init_line(line);
        
        spline(splinepoints, line, 30, 1.0);
        smoothrender.re_init_line(splinepoints);
        
        for (int i = 0; i < ey.size(); i++) {
            marr.push_back(create_orientations(ey[i], ez[i]));
        }
        
        // CAMERA PATH - SCENE 1
        smoothrender_cam.init();
        linerender_cam.init();
        
        line_cam.push_back(vec3(-2.68961,0,-41.5789));
        ez_cam.push_back(vec3(-0.0191004,0,-0.999818));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(2.53032,0,-30.4962));
        ez_cam.push_back(vec3(-0.0191004,0,-0.999818));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(4.37346,0,-25.2298));
        ez_cam.push_back(vec3(-0.0191004,0,-0.999818));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(3.19675,0,-20.24575));
        ez_cam.push_back(vec3(-0.0191004,0,-0.999818));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(-1.266923,0,-10.12222));
        ez_cam.push_back(vec3(-0.0191004,0,-0.999818));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(-1.266923,0,-10.12222));
        
        spline(splinepoints_cam, line_cam, 10, 1.0);
        smoothrender_cam.re_init_line(splinepoints_cam);
        
        for (int i = 0; i < ey_cam.size(); i++) {
            marr_cam.push_back(create_orientations(ey_cam[i], ez_cam[i]));
        }*/
        
        // butterfly PATH - SCENE 3
        /*smoothrender.init();
        linerender.init();
        line.push_back(vec3(16.5254,-3.03855,26.033));
        ez.push_back(vec3(-0.471933,-0.076869,-0.878277));
        ey.push_back(vec3(-0.0363847,0.997041,-0.0677126));
        
        line.push_back(vec3(10.7236,-2.91519,15.2358));
        ez.push_back(vec3(-0.466524,0.169005,-0.868212));
        ey.push_back(vec3(0.0799958,0.985615,0.148874));
        
        line.push_back(vec3(9.93722,-2.07061,10.3735));
        ez.push_back(vec3(-0.569405,-0.105121,-0.815308));
        ey.push_back(vec3(-0.0601898,0.994459,-0.0861833));
        
        line.push_back(vec3(6.63031,-2.56108,7.1188));
        ez.push_back(vec3(-0.0777537,0.121101,-0.98959));
        ey.push_back(vec3(0.00948586,0.99264,0.120729));
        
        line.push_back(vec3(6.37005,-2.15573,3.80638));
        ez.push_back(vec3(-0.552113,0.170654,-0.816118));
        ey.push_back(vec3(0.0956227,0.985331,0.141347));
        
        line.push_back(vec3(1.83982,-1.04885,-2.25771));
        ez.push_back(vec3(-0.957854,0.2666,-0.106962));
        ey.push_back(vec3(0.264953,0.963807,0.0295869));
        
        line.push_back(vec3(1.83982,-1.04885,-2.25771));
        linerender.re_init_line(line);
        
        spline(splinepoints, line, 30, 1.0);
        smoothrender.re_init_line(splinepoints);
        
        for (int i = 0; i < ey.size(); i++) {
            marr.push_back(create_orientations(ey[i], ez[i]));
        }*/
        
        // CAMERA PATH - SCENE 3
        /*smoothrender_cam.init();
        linerender_cam.init();
        
        line_cam.push_back(vec3(16.5254,0.53855,24.033));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(10.7236,-1.41519,13.2358));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(9.93722,0.57061,8.3735));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(6.63031,-1.06108,5.1188));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(6.37005,0.45573,1.80638));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        line_cam.push_back(vec3(1.83982,0.54885,-4.25771));
        ez_cam.push_back(vec3(0,0,1));
        ey_cam.push_back(vec3(0,1,0));
        
        spline(splinepoints_cam, line_cam, 10, 1.0);
        smoothrender_cam.re_init_line(splinepoints_cam);
        
        for (int i = 0; i < ey_cam.size(); i++) {
            marr_cam.push_back(create_orientations(ey_cam[i], ez_cam[i]));
        }*/
        
        // CAMERA PATH - SCENE 2
        /*smoothrender_cam.init();
        linerender_cam.init();
        
        line_cam.push_back(vec3(-1.11983,0,11.1238));
        ez_cam.push_back(vec3(0.175187,-0.273957,-0.945652));
        ey_cam.push_back(vec3(0.0499029,0.961742,-0.269374));
        
        line_cam.push_back(vec3(6.39166,-0.254877,8.14981));
        ez_cam.push_back(vec3(-0.813136,-0.236967,-0.531654));
        ey_cam.push_back(vec3(-0.198336,0.971518,-0.129678));
        
        line_cam.push_back(vec3(0.499288,-0.658734,-3.07302));
        ez_cam.push_back(vec3(-0.0386231,-0.21654,0.975509));
        ey_cam.push_back(vec3(-0.00856672,0.976274,0.216371));
        
        line_cam.push_back(vec3(-1.499288,-0.658734,-1.07302));
        ez_cam.push_back(vec3(-0.0386231,-0.21654,0.975509));
        ey_cam.push_back(vec3(-0.00856672,0.976274,0.216371));
        
        line_cam.push_back(vec3(-3.499288,-0.658734,2.07302));
        ez_cam.push_back(vec3(-0.0386231,-0.21654,0.975509));
        ey_cam.push_back(vec3(-0.00856672,0.976274,0.216371));
        
        line_cam.push_back(vec3(-5.85429,-0.902325,4.12563));
        ez_cam.push_back(vec3(0.992169,-0.124475,0.0103308));
        ey_cam.push_back(vec3(0.124469,0.992223,0.00129601));
        
        line_cam.push_back(vec3(-0.594785,-1.2922,12.7249));
        ez_cam.push_back(vec3(0.0320464,-0.169782,-0.98496));
        ey_cam.push_back(vec3(0.00552107,0.985482,-0.169693));
        
        line_cam.push_back(vec3(-0.486688,-1.42718,9.40252));
        ez_cam.push_back(vec3(0.164461,-0.0405704,-0.985549));
        ey_cam.push_back(vec3(0.00667776,0.999177,-0.040017));
        
        spline(splinepoints_cam, line_cam, 10, 1.0);
        smoothrender_cam.re_init_line(splinepoints_cam);
        
        for (int i = 0; i < ey_cam.size(); i++) {
            marr_cam.push_back(create_orientations(ey_cam[i], ez_cam[i]));
        }*/
        
        
	}

	//General OGL initialization - set OGL state here
	void init(const std::string& resourceDirectory)
	{
		GLSL::checkVersion();
        
        // Init SWARM
        swarm = new Swarm(WORLD_SIZE, 100, 0);

		// Set background color.
		glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
		// Enable z-buffer test.
		glEnable(GL_DEPTH_TEST);
		//glDisable(GL_DEPTH_TEST);
		// Initialize the GLSL program.
		psky = std::make_shared<Program>();
		psky->setVerbose(true);
		psky->setShaderNames(resourceDirectory + "/skyvertex.glsl", resourceDirectory + "/skyfrag.glsl");
		if (!psky->init())
		{
			std::cerr << "One or more shaders failed to compile... exiting!" << std::endl;
			exit(1);
		}
		
		psky->addUniform("P");
		psky->addUniform("V");
		psky->addUniform("M");
		psky->addUniform("tex");
		psky->addUniform("camPos");
		psky->addAttribute("vertPos");
		psky->addAttribute("vertNor");
		psky->addAttribute("vertTex");

		skinProg = std::make_shared<Program>();
		skinProg->setVerbose(true);
		skinProg->setShaderNames(resourceDirectory + "/skinning_vert.glsl", resourceDirectory + "/skinning_frag.glsl");
		if (!skinProg->init())
		{
			std::cerr << "One or more shaders failed to compile... exiting!" << std::endl;
			exit(1);
		}
		
		skinProg->addUniform("P");
		skinProg->addUniform("V");
		skinProg->addUniform("M");
		skinProg->addUniform("tex");
		skinProg->addUniform("camPos");
		skinProg->addAttribute("vertPos");
		skinProg->addAttribute("vertNor");
		skinProg->addAttribute("vertTex");
		skinProg->addAttribute("BoneIDs");
		skinProg->addAttribute("Weights");
        
        // BUTTERFLY SHADER
        prog = std::make_shared<Program>();
        prog->setVerbose(true);
        prog->setShaderNames(resourceDirectory + "/butterfly_vert.glsl", resourceDirectory + "/butterfly_frag.glsl");
        if (!prog->init())
        {
            std::cerr << "One or more shaders failed to compile... exiting!" << std::endl;
            exit(1);
        }
        prog->addUniform("P");
        prog->addUniform("V");
        prog->addUniform("M");
        prog->addUniform("campos");
        prog->addUniform("texoff");
        prog->addUniform("texoff_next");
        prog->addUniform("t");
        prog->addAttribute("vertPos");
        prog->addAttribute("vertNor");
        prog->addAttribute("vertTex");
        
        // SKY BOX SHADER
        cubeProg = make_shared<Program>();
        cubeProg->setVerbose(true);
        cubeProg->setShaderNames(resourceDirectory + "/cube_vert.glsl", resourceDirectory + "/cube_frag.glsl");
        cubeProg->init();
        cubeProg->addUniform("P");
        cubeProg->addUniform("V");
        cubeProg->addUniform("M");
        cubeProg->addUniform("skybox");
        cubeProg->addAttribute("vertPos");
        cubeProg->addAttribute("vertNor");
        
        // PLANTS PROGRAM
        plantsProg = std::make_shared<Program>();
        plantsProg->setVerbose(true);
        plantsProg->setShaderNames(resourceDirectory + "/shader_vertex.glsl", resourceDirectory + "/shader_fragment.glsl");
        if (!plantsProg->init())
        {
            std::cerr << "One or more shaders failed to compile... exiting!" << std::endl;
            exit(1);
        }
        
        plantsProg->addUniform("P");
        plantsProg->addUniform("V");
        plantsProg->addUniform("M");
        plantsProg->addUniform("tex");
        plantsProg->addUniform("camPos");
        plantsProg->addAttribute("vertPos");
        plantsProg->addAttribute("vertNor");
        plantsProg->addAttribute("vertTex");
	}


	/****DRAW
	This is the most important function in your program - this is where you
	will actually issue the commands to draw any geometry you have set up to
	draw
	********/
	void render()
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        double frametime = get_last_elapsed_time();
		static double totaltime = 0;
		totaltime += frametime * 0.5;

		// Get current frame buffer size.
		int width, height;
		glfwGetFramebufferSize(windowManager->getHandle(), &width, &height);
		float aspect = width/(float)height;
		glViewport(0, 0, width, height);

		// Clear framebuffer.
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Create the matrix stacks - please leave these alone for now
		
		glm::mat4 V, M, P; //View, Model and Perspective matrix
		V = mycam.process(frametime);
		M = glm::mat4(1);
		// Apply orthographic projection....
		P = glm::ortho(-1 * aspect, 1 * aspect, -1.0f, 1.0f, -2.0f, 100.0f);		
		if (width < height)
			{
			P = glm::ortho(-1.0f, 1.0f, -1.0f / aspect,  1.0f / aspect, -2.0f, 100.0f);
			}
		// ...but we overwrite it (optional) with a perspective projection.
		P = glm::perspective((float)(3.14159 / 4.), (float)((float)width/ (float)height), 0.1f, 1000.0f); //so much type casting... GLM metods are quite funny ones
		/*auto sangle = -3.1415926f / 2.0f;
		glm::mat4 RotateXSky = glm::rotate(glm::mat4(1.0f), sangle, glm::vec3(1.0f, 0.0f, 0.0f));
		glm::vec3 camp = -mycam.pos;
		glm::mat4 TransSky = glm::translate(glm::mat4(1.0f), camp);
		glm::mat4 SSky = glm::scale(glm::mat4(1.0f), glm::vec3(0.8f, 0.8f, 0.8f));

		M = TransSky  * RotateXSky * SSky;

		// Draw the sky using GLSL.
		psky->bind();
		GLuint texLoc = glGetUniformLocation(psky->pid, "tex");
		skyTex->bind(texLoc);	
		glUniformMatrix4fv(psky->getUniform("P"), 1, GL_FALSE, &P[0][0]);
		glUniformMatrix4fv(psky->getUniform("V"), 1, GL_FALSE, &V[0][0]);
		glUniformMatrix4fv(psky->getUniform("M"), 1, GL_FALSE, &M[0][0]);
		glUniform3fv(psky->getUniform("camPos"), 1, &mycam.pos[0]);

		glDisable(GL_DEPTH_TEST);
		skyShape->draw(psky, false);
		glEnable(GL_DEPTH_TEST);	
		skyTex->unbind();
		psky->unbind();*/
        
        // global shift down
        glm::mat4 TransDown = glm::translate(glm::mat4(1.0f), vec3(0, -1, 0));

		// draw the skinned mesh		
		skinProg->bind();
		GLuint texLoc = glGetUniformLocation(skinProg->pid, "tex");
        GLuint texLoc1 = glGetUniformLocation(skinProg->pid, "tex2");
        chihiroTex->bind(texLoc);
        
        // CHIHIRO ON PATH
        /*static double totaltimeC = 0;
        totaltimeC += frametime;
        double tenthtimeC = totaltimeC * 10;
        int itenthtimeC = (int)tenthtimeC;
        float tC = tenthtimeC - itenthtimeC;
        glm::vec3 chihiro_pos;
        float chihiro_orient;
        glm::mat4 MoveChihiro;
        glm::mat4 ChihiroOrientation;
        
        double realtime_o = tenthtimeC/28.0;
        int itime_o = realtime_o;
        float t_o = realtime_o - itime_o;
        
        // plane 1
        
        if (itenthtimeC < splinepoints.size()) {
            glm::vec3 A = splinepoints[itenthtimeC];
            glm::vec3 B = splinepoints[itenthtimeC+1];
            
            chihiro_pos = (A * (1 - tC)) + (B * tC);
            
            glm::vec3 Z = normalize(B - A);
            glm::vec3 Y = glm::vec3(0, 1, 0);
        
        
            MoveChihiro = glm::translate(glm::mat4(1.0f), chihiro_pos);
            
            //from one control point to another is a distance of 28
            
            mat4 m1 = marr[itime_o];
            mat4 m2 = marr[itime_o+1];
            
            ChihiroOrientation = linint_between_two_orientations(m1, m2, cosint(t_o));
            
        }*/
        
		auto sangle = 3.1415926f;
		glm::mat4 Trans = glm::translate(glm::mat4(1.0f), vec3(0, 0, -4));
		glm::mat4 RotX = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 1, 0));
        glm::mat4 RotZ = glm::rotate(glm::mat4(1.0f), -sangle/2, vec3(1, 0, 0));
		glm::mat4 Scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.5f, 0.5f));
		M = TransDown * Trans * RotX * Scale; // T R S
		
		glUniform3fv(skinProg->getUniform("camPos"), 1, &mycam.pos[0]);
		glUniformMatrix4fv(skinProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
		glUniformMatrix4fv(skinProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
		glUniformMatrix4fv(skinProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
		skmesh.setBoneTransformations(skinProg->pid, totaltime * 1.36);
		skmesh.Render(texLoc);
        chihiroTex->unbind();
		skinProg->unbind();
        
        sangle = 3.1415926f / 3.0f;
        glm::vec3 camp = -mycam.pos * 0.5f;
        glm::mat4 TransSky = glm::translate(glm::mat4(1.0f), camp);
        glm::mat4 TransUp = glm::translate(glm::mat4(1.0f), vec3(0,3,0));
        glm::mat4 RotSky = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 1, 0));
        glm::mat4 SSky = glm::scale(glm::mat4(1.0f), glm::vec3(500, 500, 500));

        M = TransSky * TransUp * RotSky * SSky;
        
        //draw sky box
        cubeProg->bind();
        glUniformMatrix4fv(cubeProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
        glDepthFunc(GL_LEQUAL);
        glUniformMatrix4fv(cubeProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
        glUniformMatrix4fv(cubeProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
        
        glBindTexture(GL_TEXTURE_CUBE_MAP, cubeMapTexture);
        cube->draw(cubeProg, false);
        
        glDepthFunc(GL_LESS);
        cubeProg->unbind();
        
        // CAMERA PATH
        static double totaltime_cam = 0;
        totaltime_cam += frametime;
        double tenthtime_cam = totaltime_cam * 2.8;
        int i_cam = (int)tenthtime_cam;
        float t_cam = tenthtime_cam - i_cam;
        glm::vec3 camera_pos;
        glm::mat4 MoveCamera;
        glm::mat4 CameraLookAt;
        
        double cam_o = tenthtime_cam/16.0;
        int icam_o = cam_o;
        float tcam_o = cam_o - icam_o;
        
        if (i_cam < splinepoints_cam.size()) {
            glm::vec3 A = splinepoints_cam[i_cam];
            glm::vec3 B = splinepoints_cam[i_cam+1];
            
            
            camera_pos = (A * (1 - t_cam)) + (B * t_cam);
            
            glm::vec3 Z = normalize(B - A);
            glm::vec3 Y = glm::vec3(0, 1, 0);
        
        
            MoveCamera = glm::translate(glm::mat4(1.0f), camera_pos);
            
            mat4 m1 = marr_cam[icam_o];
            mat4 m2 = marr_cam[icam_o+1];
            
            CameraLookAt = linint_between_two_orientations(m1, m2, cosint(tcam_o));
            
        }
        
        sangle = 3.1415926 / 2.;
        glm::mat4 RotateXPlane = glm::rotate(glm::mat4(1.0f), sangle, vec3(1,0,0));
        float szangle = 3.1415926;
        glm::mat4 RotateZ = glm::rotate(glm::mat4(1.0f), szangle, vec3(0,0,1));
        glm::mat4 RotateYCam = glm::rotate(glm::mat4(1.0f), szangle / 2, vec3(0,1,0));
        
        vec4 new_cam_pos = (MoveCamera * glm::vec4(1,1,1,1));
        //vec4 new_cam_rot = ((CameraLookAt * RotateXPlane * RotateZ) * glm::vec4(1,1,1,0));
        //--mycam.pos = vec3(-new_cam_pos.x + 2, -new_cam_pos.y - 0.5, -new_cam_pos.z +5); // - 4.25
        //mycam.rot = vec3(new_cam_rot.x, new_cam_rot.y, new_cam_rot.z);
        //--mycam.R = CameraLookAt * RotateXPlane * RotateYCam;
        
        // surprised Chihiro
        plantsProg->bind();
        texLoc = glGetUniformLocation(skinProg->pid, "tex");
        chihiroTex1->bind(texLoc);
        sangle = -3.1415926f / 2.0f;
        glm::mat4 TransC= glm::translate(glm::mat4(1.0f), vec3(0, 1, -3));
        RotX = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 1, 0));
        glm::mat4 ScaleC = glm::scale(glm::mat4(1.0f), glm::vec3(2.f, 2.f, 2.f));
        M = TransDown * TransC * RotX * ScaleC; // T R S
        
        glUniform3fv(plantsProg->getUniform("camPos"), 1, &mycam.pos[0]);
        glUniformMatrix4fv(plantsProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
        glUniformMatrix4fv(plantsProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
        glUniformMatrix4fv(plantsProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
        //chihiro->draw(plantsProg, false);
        chihiroTex1->unbind();
        plantsProg->unbind();

        
        prog->bind();

        
        //send the matrices to the shaders
        glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, &P[0][0]);
        glUniformMatrix4fv(prog->getUniform("V"), 1, GL_FALSE, &V[0][0]);
        //glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, &M[0][0]);
        glUniform3fv(prog->getUniform("campos"), 1, &mycam.pos[0]);

        // TEXTURE ANIMATION BUTTERFLY
        // vec2
        vec2 to;
        vec2 to_next;

        // Frame change every 1/10 second
        double tenthtime = (totaltime * 0.8) * 10.0;
        int itenthtime = (int)tenthtime;
        int x = itenthtime % 4;
        int y = itenthtime % 4;
        to.x = x;
        to.y = y;
        
        int itenthtime2 = (int)tenthtime + 1/10;
        int x2 = itenthtime2 % 4;
        int y2 = itenthtime2 % 4;
        to_next.x = x2;
        to_next.y = y2;
        
        float t = tenthtime - itenthtime;
        
        glUniform2fv(prog->getUniform("texoff"), 1, &to.x);
        glUniform2fv(prog->getUniform("texoff_next"), 1, &to_next.x);
        glUniform1fv(prog->getUniform("t"), 1, &t);
    
        glBindVertexArray(VertexArrayID);
        //actually draw from vertex 0, 3 vertices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferIDBox);
        //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, (void*)0);
        mat4 Vi = glm::transpose(V);
        Vi[0][3] = 0;
        Vi[1][3] = 0;
        Vi[2][3] = 0;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        
        // BUTTERFLY ON PATH
        /*static double totaltimeBut = 0;
        totaltimeBut += frametime;
        double tenthtimeB = totaltimeBut * 10;
        int itenthtimeB = (int)tenthtimeB;
        float tB = tenthtimeB - itenthtimeB;
        glm::vec3 butterfly_pos;
        float butterfly_orient;
        glm::mat4 MoveButterfly;
        glm::mat4 ButterflyOrientation;
        
        double realtime_o = tenthtime/28.0;
        int itime_o = realtime_o;
        float t_o = realtime_o - itime_o;
        
        // plane 1
        
        if (itenthtimeB < splinepoints.size()) {
            glm::vec3 A = splinepoints[itenthtimeB];
            glm::vec3 B = splinepoints[itenthtimeB+1];
            
            butterfly_pos = (A * (1 - tB)) + (B * tB);
            
            glm::vec3 Z = normalize(B - A);
            glm::vec3 Y = glm::vec3(0, 1, 0);
        
        
            MoveButterfly = glm::translate(glm::mat4(1.0f), butterfly_pos);
            
            //from one control point to another is a distance of 28
            
            mat4 m1 = marr[itime_o];
            mat4 m2 = marr[itime_o+1];
            
            ButterflyOrientation = linint_between_two_orientations(m1, m2, cosint(t_o));
            
        }*/

        static float angleB = 0.0f;
        angleB += 1.0 * frametime; //rotation angle
        glm::mat4 RotateYB = glm::rotate(glm::mat4(1.0f), angleB, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 Flip = glm::rotate(glm::mat4(1.0f), 3.14159265f, glm::vec3(0.0f, 0.0f, 1.0f));
        glm::mat4 Rot = glm::rotate(glm::mat4(1.0f), 3.14159265f/4.f, glm::vec3(0.0f, 1.0f, 0.0f));
        glm::mat4 TransY = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, (sin(glfwGetTime() * 3.0) * 0.5) + 1.0, 0.0f));
        glm::mat4 TransZ = glm::translate(glm::mat4(1.0f), glm::vec3(sin(glfwGetTime() * 2.0) * 2., 0.f, cos(glfwGetTime() * 2.0)  * 2. ));
        //glm::mat4 BoidPos = glm::translate(glm::mat4(1.0f), bit->position);
        
        glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(1.f, 1.f, 1.f));
        M = TransZ * TransY * Trans * Rot * Flip * angleB * S * Vi;
        glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, &M[0][0]);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, (void*)0);
        
        glBindVertexArray(0);

        
        prog->unbind();
        
        // BUTTERFLY BOIDS
        swarm->move_boids();
        
        for (vector<Boid>::iterator bit = swarm->boids.begin(); bit != swarm->boids.end(); ++bit) {
        
            //animation with the model matrix

            // Draw the box using GLSL.
            prog->bind();

            
            //send the matrices to the shaders
            glUniformMatrix4fv(prog->getUniform("P"), 1, GL_FALSE, &P[0][0]);
            glUniformMatrix4fv(prog->getUniform("V"), 1, GL_FALSE, &V[0][0]);
            //glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, &M[0][0]);
            glUniform3fv(prog->getUniform("campos"), 1, &mycam.pos[0]);

            // vec2
            vec2 to;
            vec2 to_next;

            static double totaltime = 0;
            totaltime += frametime * 0.01;

            // Frame change every 1/10 second
            double tenthtime = totaltime * 10.0;
            int itenthtime = (int)tenthtime;
            int x = itenthtime % 4;
            int y = itenthtime % 4;
            to.x = x;
            to.y = y;
            
            int itenthtime2 = (int)tenthtime + 1/10;
            int x2 = itenthtime2 % 4;
            int y2 = itenthtime2 % 4;
            to_next.x = x2;
            to_next.y = y2;
            
            float t = tenthtime - itenthtime;
            
            glUniform2fv(prog->getUniform("texoff"), 1, &to.x);
            glUniform2fv(prog->getUniform("texoff_next"), 1, &to_next.x);
            glUniform1fv(prog->getUniform("t"), 1, &t);
        
            glBindVertexArray(VertexArrayID);
            //actually draw from vertex 0, 3 vertices
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IndexBufferIDBox);
            //glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_SHORT, (void*)0);
            mat4 Vi = glm::transpose(V);
            Vi[0][3] = 0;
            Vi[1][3] = 0;
            Vi[2][3] = 0;
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, Texture);

            static float angleB = 0.0f;
            angleB += 1.0 * frametime * 0.01; //rotation angle
            glm::mat4 RotateYB = glm::rotate(glm::mat4(1.0f), angleB, glm::vec3(0.0f, 1.0f, 0.0f));
            glm::mat4 Flip = glm::rotate(glm::mat4(1.0f), 3.14159265f, glm::vec3(0.0f, 0.0f, 1.0f));
            //glm::mat4 TransY = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, sin(glfwGetTime() * 2) * 0.5, 0.0f));
            glm::mat4 TransZ = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -3));
            glm::mat4 BoidPos = glm::translate(glm::mat4(1.0f), bit->position);
            
            glm::mat4 S = glm::scale(glm::mat4(1.0f), glm::vec3(1.5f, 1.5f, 1.5f));
            M = BoidPos * TransDown * TransZ * RotateYB * Flip * S * Vi;
            glUniformMatrix4fv(prog->getUniform("M"), 1, GL_FALSE, &M[0][0]);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, (void*)0);
            
            glBindVertexArray(0);

            
            prog->unbind();
        }
        
        // draw the flowers
        for (int i = 0; i < 50; i++) {
            plantsProg->bind();
            texLoc = glGetUniformLocation(plantsProg->pid, "tex");
            flowerTex->bind(texLoc);
            //auto sangle = -3.1415926f / 2.0f;
            Trans = glm::translate(glm::mat4(1.0f), flowerPos[i]);
            //glm::mat4 RotX = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 0, 0));
            Scale = glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.5f, 0.5f));
            M = TransDown * Trans * Scale; // T R S
            
            glUniform3fv(plantsProg->getUniform("camPos"), 1, &mycam.pos[0]);
            glUniformMatrix4fv(plantsProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
            glUniformMatrix4fv(plantsProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
            glUniformMatrix4fv(plantsProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
            flower->draw(plantsProg, false);
            flowerTex->unbind();
            plantsProg->unbind();
        }
        
        // draw the trees
        for (int i = 0; i < 50; i++) {
            plantsProg->bind();
            texLoc = glGetUniformLocation(plantsProg->pid, "tex");
            treeTex->bind(texLoc);
            //auto sangle = -3.1415926f / 2.0f;
            Trans = glm::translate(glm::mat4(1.0f), treePos[i]);
            //glm::mat4 RotX = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 0, 0));
            Scale = glm::scale(glm::mat4(1.0f), glm::vec3(3.f, 3.f, 3.f));
            M = TransDown * Trans * Scale; // T R S
            
            glUniform3fv(plantsProg->getUniform("camPos"), 1, &mycam.pos[0]);
            glUniformMatrix4fv(plantsProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
            glUniformMatrix4fv(plantsProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
            glUniformMatrix4fv(plantsProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
            tree->draw(plantsProg, false);
            treeTex->unbind();
            plantsProg->unbind();
        }
        
        // draw ground
        plantsProg->bind();
        texLoc = glGetUniformLocation(plantsProg->pid, "tex");
        grassTex->bind(texLoc);
        //auto sangle = -3.1415926f / 2.0f;
        Trans = glm::translate(glm::mat4(1.0f), vec3(0, -1, 0));
        //glm::mat4 RotX = glm::rotate(glm::mat4(1.0f), sangle, vec3(0, 0, 0));
        Scale = glm::scale(glm::mat4(1.0f), glm::vec3(30.f, 0.5f, 30.f));
        M = TransDown * Trans * Scale; // T R S
        
        glUniform3fv(plantsProg->getUniform("camPos"), 1, &mycam.pos[0]);
        glUniformMatrix4fv(plantsProg->getUniform("P"), 1, GL_FALSE, &P[0][0]);
        glUniformMatrix4fv(plantsProg->getUniform("V"), 1, GL_FALSE, &V[0][0]);
        glUniformMatrix4fv(plantsProg->getUniform("M"), 1, GL_FALSE, &M[0][0]);
        cube->draw(plantsProg, false);
        grassTex->unbind();
        plantsProg->unbind();
	}
};

//******************************************************************************************
int main(int argc, char **argv)
{
	std::string resourceDir = "../resources"; // Where the resources are loaded from
	std::string missingTexture = "missing.png";
	
	if (argc >= 2)
	{
		resourceDir = argv[1];
	}
    
    SkinnedMesh::setResourceDir(resourceDir);
    SkinnedMesh::setDefaultTexture(missingTexture);

	Application *application = new Application();

	/* your main will always include a similar set up to establish your window
		and GL context, etc. */
	WindowManager * windowManager = new WindowManager();
	windowManager->init(1920, 1080);
	windowManager->setEventCallbacks(application);
	application->windowManager = windowManager;

	/* This is the code that will likely change program to program as you
		may need to initialize or set up different data and state */
	// Initialize scene.
	application->init(resourceDir);
	application->initGeom(resourceDir);
    application->cubeMapTexture = SmartTexture::createSky(resourceDir + "/skybox/", application->faces);

	// Loop until the user closes the window.
	while(! glfwWindowShouldClose(windowManager->getHandle()))
	{
		// Render scene.
		application->render();

		// Swap front and back buffers.
		glfwSwapBuffers(windowManager->getHandle());
		// Poll for and process events.
		glfwPollEvents();
	}

	// Quit program.
	windowManager->shutdown();
	return 0;
}

