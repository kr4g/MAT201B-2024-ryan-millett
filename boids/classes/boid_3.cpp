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

    float minEdgeProximity{1.5f}; // Minimum distance from edge to start turning
    float turnRateFactor{0.2f};  // Factor to adjust turning rate

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

    void heading(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f averageOncomingHeading(0, 0, 0);
        Vec3f averageParallelHeading(0, 0, 0);
        int oncomingCount = 0, parallelCount = 0;

        // Calculate average heading of oncoming and parallel boids
        for (int i : i_navs) {
            Vec3f otherPos = navs[i]->pos();
            Vec3f toOther = (otherPos - bNav.pos()).normalize();
            float oncoming = bNav.uf().dot(toOther);
            
            if (oncoming < 0) {
                averageOncomingHeading += navs[i]->uf();
                oncomingCount++;
            } else {
                averageParallelHeading += navs[i]->uf();
                parallelCount++;
            }
        }

        if (oncomingCount > 0) {
            averageOncomingHeading /= oncomingCount;
            Vec3f perpendicularDirection;
            float seriousnessFactor = std::min(std::max(oncomingCount / (float)i_navs.size(), 0.0f), 1.0f);
            
            if (parallelCount > 0) {
                averageParallelHeading /= parallelCount;
                perpendicularDirection = averageParallelHeading.cross(averageOncomingHeading).normalize();
            } else {
                perpendicularDirection = bNav.uf().cross(averageOncomingHeading).normalize();
            }

            if (perpendicularDirection.mag() == 0) { // Check for zero vector
                // Use the boid's up vector or a random up vector as a fallback
                perpendicularDirection = bNav.uu(); // Or generate a random up vector if preferred
            }
            float turnRate = std::min(turnRateFactor * seriousnessFactor, 1.0f);
            bNav.faceToward(bNav.pos() + perpendicularDirection, bNav.uu(), turnRate);
        }
    }

    void detectCollisions(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f avoidanceDirection(0, 0, 0);
        float highestRisk = 0.0f;

        for (int i : i_navs) {
            Vec3f otherPos = navs[i]->pos();
            Vec3f toOther = otherPos - bNav.pos();
            Vec3f otherHeading = navs[i]->uf();
            float angleToOther = acos(bNav.uf().dot(toOther.normalize()));
            float angleFromOther = acos(otherHeading.dot(-toOther.normalize()));

            float riskLevel = 0.0f;
            if (angleToOther < M_PI / 4 && angleFromOther < M_PI / 4) { // Head-on collision
                riskLevel = 1.0f;
            } else if (std::abs(angleToOther - angleFromOther) < M_PI / 6) { // High risk of perpendicular collision
                riskLevel = 0.5f;
            }

            if (riskLevel > highestRisk) {
                highestRisk = riskLevel;
                avoidanceDirection = toOther.cross(bNav.uf()).normalize();
            }
        }

        if (highestRisk > 0) {
            float turnRate = highestRisk; 
            bNav.faceToward(bNav.pos() + avoidanceDirection, bNav.uu(), turnRate);
        }
    }

    void boidForces(const std::vector<Boid>& boids, const std::vector<int>& i_boids, float headingForce = 0.5, float cohesionForce = 0.5, float separationForce = 0.5) {
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

    void alignment(const std::vector<Boid>& boids, const std::vector<int>& i_boids, float threshold = 4.0, double force = 0.5) {
        Vec3f averageHeading(0, 0, 0);
        Vec3f averageUp(0, 0, 0);
        int alignCount = 0;
        for (int i : i_boids) {
            float dist = (bNav.pos() - boids[i].bNav.pos()).mag();
            if (dist < threshold) { 
                averageHeading += boids[i].bNav.uf();
                averageUp += boids[i].bNav.uu();
                alignCount++;
            }
        }
        if (alignCount > 0) {
            averageHeading /= alignCount;
            averageUp /= alignCount;
            bNav.faceToward(bNav.pos() + averageHeading.normalized(), averageUp.normalized(), turnRateFactor*force);
        }
    }

    void cohesion(const std::vector<Boid>& boids, const std::vector<int>& i_boids, float threshold = 4.3, double force = 0.5) {
        Vec3f centerOfMass(0, 0, 0);
        int cohesionCount = 0;
        for (int i : i_boids) {
            float dist = (bNav.pos() - boids[i].bNav.pos()).mag();
            if (dist < threshold) {
                centerOfMass += boids[i].bNav.pos();
                cohesionCount++;
            }
        }
        if (cohesionCount > 0) {
            float turnRate = cohesionCount / i_boids.size();
            centerOfMass /= cohesionCount;
            bNav.faceToward(centerOfMass, turnRate*turnRateFactor*force);
        }
    }

    void separation(const std::vector<Boid>& boids, const std::vector<int>& i_boids, float threshold = 0.75, float force = 0.5) {
        Vec3f separationForce(0, 0, 0);
        int closeBoids = 0;
        for (int i : i_boids) {
            float dist = (bNav.pos() - boids[i].bNav.pos()).mag();
            Vec3f away = (bNav.pos() - boids[i].bNav.pos()).normalize() / dist;
            if (dist < threshold) {
                separationForce += away;
                closeBoids++;
            }            
        }
        if (closeBoids > 0) {
            float turnRate = closeBoids / i_boids.size();
            separationForce /= closeBoids;
            bNav.faceToward(separationForce, turnRate*sqrt(sqrt(turnRateFactor)*force));
        }
    }

    // void findFood(const Octree& tree, float size, const std::vector<Vec3f>& food, const std::vector<float>& mass) {
    //     // hunger = (hunger > 1.0) ? hunger = 1.0 : hunger; 
    //     if ((target - bNav.pos()).magSqr() < 0.5) {
    //         // hunger -= 0.0001; // make proportional to mass of food
    //         target.set(Vec3d(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()));
    //     }
    //     vector<int> i_food;
    //     tree.queryRegion(bNav.pos(), Vec3f(size, size, size), i_food);
    //     float si = i_food.size();
    //     if (si > 0) {
    //         int biggestFood = i_food[0];
    //         for (int i : i_food) {
    //             // float foodMass = mass[i];
    //             if (mass[i] > mass[biggestFood]) {
    //                 biggestFood = i;
    //             }
    //             hunger += i / si; // make proportional to mass of food
    //         }
    //         seek(food[biggestFood], 0.31);
    //     }        
    // }

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
