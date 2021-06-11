//
//  Swarm.h
//  lab1
//
//  Created by Jaelyn Domingo on 5/19/21.
//

#ifndef Swarm_h
#define Swarm_h
#endif /* Swarm_h */

#include <vector>
#include "Boid.h"

// value_ptr for glm
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace std;
using namespace glm;

class Swarm {
    
public:
    Swarm(int worldSize, int swarmSize, int flyHeight);
    void addBoid(Boid b);
    vec3 random_coord(float min, float max, float startHeight);
    void move_boids();
    vec3 cohesion(Boid b);
    vec3 separation(Boid b);
    vec3 alignment(Boid b);
    vec3 wallAvoidance(Boid b);
    vector<Boid> boids;
    int worldSize;
    int swarmSize;
    int minflyHeight;
};
