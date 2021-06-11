//
//  Boid.h
//  lab1
//
//  Created by Jaelyn Domingo on 5/19/21.
//

#ifndef Boid_h
#define Boid_h
#endif /* Boid_h */

// value_ptr for glm
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace std;
using namespace glm;

class Boid {
    
public:
    Boid(vec3 pos, vec3 vel);
    vec3 position;
    vec3 velocity;
};
