#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/math/al_Functions.hpp"

const float MAX_PREY_LIFESPAN           = 300.0f;
const float MAX_PREDATOR_LIFESPAN       = 100.0f;

const float MIN_PREY_EDGE_PROXIMITY     = 0.01f;
const float MIN_PREDATOR_EDGE_PROXIMITY = 0.05f;

const float MAX_PREY_TURN_RATE          = 0.1f;
const float MAX_PREDATOR_TURN_RATE      = 0.2f;

using namespace al;

class Boid {
public:    
    Nav bNav;                     // Navigation object
    Vec3d target;                 // Where the boid is going
    bool lifeStatus{true};
    float frequency{5.0f};
    float attentionSpan{1.0f};
    float hunger{1.0f};
    float fear{0.0f};
    float mutation{0.0f};
    float mutationRate{0.0f};
    float age{0.0f};
    float ageRate{0.001f};
    float lifespan;
    int meshIdx;
    int Nv;

    float minEdgeProximity{1.5f}; // Minimum distance from edge to start turning
    float turnRateFactor{0.2f};   // Factor to adjust turning rate

    std::vector<int> i_boids;
    // Boid() : {}
    
    ~Boid() {}

    void handleBoundary(float size) {
        // Vec3d bUf = bNav.uf();
        float dist = bNav.pos().mag();
        // float proximity = (size - dist) / size;
        // float turnRate = turnRateFactor;

        if (dist > size) {
            bNav.faceToward(Vec3d(0,0,0), 1.0);
        }

        // if (dist < 0.1) {
        //     bNav.faceToward(Vec3d(0,0,0), 0.9);
        // } else if (dist < minEdgeProximity) {
        //     bNav.faceToward(Vec3d(0,0,0), turnRateFactor*proximity);
        // }
    }

    void originAvoidance(float avoidanceRadius) {
        Vec3d pos = bNav.pos(); 
        Vec3d forward = bNav.uf(); 
        float dist = pos.mag(); 

        if (dist < avoidanceRadius) {
            Vec3d awayFromOrigin = pos.normalize(); 
            float theta = forward.dot(awayFromOrigin); 

            if (theta < 0) { // If facing towards the origin, theta is negative
                theta = -theta; // "degree" of facing towards the origin
                float proximityFactor = 1.0 - (dist / avoidanceRadius);               
                float turnRate = std::min(theta * proximityFactor, 1.0f);
                Vec3d biasedForward = forward + awayFromOrigin * theta;
                biasedForward.normalize(); 
                bNav.faceToward(pos + biasedForward, bNav.uu(), turnRate);
            }
        }
    }

    void originAvoidance(float avoidanceRadius, float preparationRadius) {
        Vec3d pos = bNav.pos(); 
        float dist = pos.mag(); 
        if (dist < preparationRadius) {
            Vec3d forward = bNav.uf(); 
            Vec3d biasedForward = Vec3d(0, 0, 0);
            Vec3d awayFromOrigin = pos.normalize();
            float theta = forward.dot(awayFromOrigin);
            float turnRate = turnRateFactor * (1.0 - dist / preparationRadius);
            if (theta < 0) { 
                theta = -theta; 
                float proximityFactor = 1.0 - (dist / avoidanceRadius);
                biasedForward = forward + awayFromOrigin * theta * proximityFactor;
                biasedForward.normalize();
                bNav.faceToward(pos + biasedForward, bNav.uu(), 0.005);
                if (dist < avoidanceRadius) {
                    turnRate = turnRateFactor * (1.0 - dist / avoidanceRadius);
                    biasedForward = forward + awayFromOrigin * theta * proximityFactor;
                    bNav.faceToward(bNav.pos() + biasedForward, bNav.uu(), turnRate);
                }                
            }
        }
    }

    void boidForces(const std::vector<Boid>& boids, float headingForce = 0.5, float cohesionForce = 0.5, float separationForce = 0.5) {
        Vec3f averageHeading(0, 0, 0);
        Vec3f averageUp(0, 0, 0);
        Vec3f centerOfMass(0, 0, 0);
        Vec3f separation(0, 0, 0);

        for (int i : i_boids) {
            Vec3f toNeighbor = boids[i].bNav.pos() - bNav.pos();
            float dist = toNeighbor.mag();
            // alignment
            averageHeading += boids[i].bNav.uf();
            averageUp += boids[i].bNav.uu();
            // cohesion
            centerOfMass += boids[i].bNav.pos();
            // separation
            Vec3f away = Vec3f(toNeighbor).normalize() / (dist*dist);
            separation -= away;
        }
        averageHeading /= i_boids.size();
        averageUp /= i_boids.size();
        centerOfMass /= i_boids.size();
        // separation /= i_boids.size();

        // Vec3f direction(0, 0, 0);

        // direction += averageHeading;
        // direction += centerOfMass;
        // direction += separationForce;
        
        bNav.faceToward(bNav.pos() + averageHeading.normalized(), averageUp.normalized(), turnRateFactor*headingForce);
        bNav.faceToward(centerOfMass, turnRateFactor*cohesionForce);
        bNav.faceToward(separation, turnRateFactor*separationForce);
    }

    void seek(const Vec3d& a, double amt, float smooth = 0.1) { 
        target.set(a);
        bNav.smooth(smooth);
        bNav.faceToward(target, bNav.uu(), amt);
    }

    void updatePosition(double dt) {
        bNav.moveF(0.67);
        bNav.step(dt);        
    }

    virtual void updateParams(float dts) {
        if (age > lifespan) {
            lifeStatus = false;
        }
        age += ageRate*(dts/5.0) + (ageRate * fear);  // Fear increases rate of aging
        if (hunger < 0.0) {
              hunger = 0.0;
        }

        if (attentionSpan < 0.0) {
            attentionSpan = 0.0;
        } else if (attentionSpan > 1.0) {
            attentionSpan = 1.0;
        }
        // float deathProximity = age / lifespan;        
        // if (fear > 1.0) {
        //     lifeStatus = rnd::uniform() > deathProximity;
        // }
        // hunger -= 0.01 - (0.1 * fear);      // Fear decreases hunger        
        // if (hunger < 0.0) {
        //     fear += 0.01;
        //     lifespan -= 0.001 + 0.001 * deathProximity;
        // }
        // mutation += mutationRate + (mutationRate * fear) + (mutationRate * deathProximity); // Fear and age increase mutation rate
    }
};
