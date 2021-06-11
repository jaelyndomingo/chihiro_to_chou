//
//  Swarm.cpp
//  lab1
//
//  Created by Jaelyn Domingo on 5/19/21.
//

#include <iostream>
#include <stdio.h>
using namespace std;
#include "Swarm.h"

Swarm::Swarm(int worldSize, int swarmSize, int flyHeight) {
    Swarm::worldSize = worldSize;
    Swarm::swarmSize = swarmSize;
    minflyHeight = flyHeight;
    
    
    for (int i = 0; i < swarmSize; i++) {
        Swarm::addBoid(
                      Boid(Swarm::random_coord(-worldSize, worldSize, flyHeight),
                           Swarm::random_coord(0.1, 0.4, 0.1)));
    }
}

void Swarm::addBoid(Boid b) {
    boids.push_back(b);
}

vec3 Swarm::random_coord(float min, float max, float startHeight) {
    float x = fmod(rand(), (max - min) + min);
    float y = fmod(rand(), (max - startHeight) + startHeight);
    float z = fmod(rand(), (max - min) + min);

    return vec3(x,y,z);
}

void Swarm::move_boids() {
    vec3 v1, v2, v3, wv;
    
    for (vector<Boid>::iterator bit = boids.begin(); bit != boids.end(); ++bit) {
        v1 = cohesion(*bit);
        v2 = alignment(*bit);
        v3 = separation(*bit);
        wv = wallAvoidance(*bit);
        
        bit->velocity += v1 + v2 + v3 + wv;
        bit->position += bit->velocity;
    }
}

vec3 Swarm::cohesion(Boid b) {
    vec3 pc = vec3(0,0,0); // perceived center
    
    // calculate perceived center
    for (vector<Boid>::iterator bit = boids.begin(); bit != boids.end(); ++bit) {
        if (bit->position != b.position) {
            pc += b.position;
        }
    }
    
    pc = pc / float(swarmSize - 1);
    //cout << pc.x << ", " << pc.y << ", " << pc.z << endl;
    
    return (pc - b.position) / 100.f;
}

vec3 Swarm::alignment(Boid b) {
    vec3 pv = vec3(0,0,0); // perceived alignment
    
    // calculate perceived alignment
    for (vector<Boid>::iterator bit = boids.begin(); bit != boids.end(); ++bit) {
        if (bit->velocity != b.velocity) {
            pv += b.velocity;
        }
    }
    
    pv = pv / float(swarmSize - 1);
    //cout << pc.x << ", " << pc.y << ", " << pc.z << endl;
    
    return (pv - b.velocity) / 8.f;
}

vec3 Swarm::separation(Boid b) {
    vec3 c = vec3(0,0,0); // separation constant
    
    // calculate separation constant
    for (vector<Boid>::iterator bit = boids.begin(); bit != boids.end(); ++bit) {
        if (bit->position != b.position) {
            if (length(bit->position - b.position) < 0.5f){
                c -= bit->position - b.position;
            }
        }
    }
    
    return c;
}

vec3 Swarm::wallAvoidance(Boid b)
{
    float treshold = 2;
    vec3 minDists = b.position - float(-worldSize);
    minDists.y = 5;
    vec3 maxDists = float(worldSize) - b.position;
    vec3 steer = vec3(0,0,0);
    for(int i = 0 ; i < 3 ; i++)
    {
        if(minDists[i] < treshold && minDists[i] < maxDists[i])
            steer[i] = treshold - minDists[i];
        
        else if(maxDists[i] < treshold)
            steer[i] = maxDists[i] - treshold;
    }

    float d = length(steer);
    if ( d > 0)
    {
        steer /= d;
        steer *= 0.04; //max speed
        steer -= b.velocity;
        /*d = length(steer);
        if(d > maxForce)
            steer *= maxForce/d;*/
    }
    return steer;
}
