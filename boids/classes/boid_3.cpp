#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/math/al_Functions.hpp"

#include "../../utils/octtree.cpp"

const float MAX_PREY_LIFESPAN           = 300.0f;
const float MAX_PREDATOR_LIFESPAN       = 100.0f;

const float MIN_PREY_EDGE_PROXIMITY     = 0.01f;
const float MIN_PREDATOR_EDGE_PROXIMITY = 0.05f;

const float MAX_PREY_TURN_RATE          = 0.1f;
const float MAX_PREDATOR_TURN_RATE      = 0.2f;

using namespace al;

class Boid {
public:    
    Nav bNav;                   // Navigation object
    Vec3d target;                 // Where the boid is going
    bool lifeStatus{true};
    float attentionSpan{1.0f};
    float hunger{1.0f};
    float fear{0.0f};
    float mutation{0.0f};
    float mutationRate{0.0f};
    float age{0.0f};
    float ageRate{0.001f};
    float lifespan;

    float minEdgeProximity{5.5f};     // Minimum distance from edge to start turning
    float turnRateFactor{0.43f};       // Factor to adjust turning rate

    // Boid() : {}
    
    ~Boid() {}

    void handleBoundary(float size) {
        // take into account not just position, but unit forward vector
        Vec3d bUf = bNav.uf();

        // distance to the edge of the world
        float dist = size - (bNav.pos() + bUf).mag();
        // if (dist < 0.0) 
        float proximity = (size - dist) / size;

        // float xDist = std::min(std::abs(bNav.pos().x - size), std::abs(bNav.pos().x + size));
        // float yDist = std::min(std::abs(bNav.pos().y - size), std::abs(bNav.pos().y + size));
        // float zDist = std::min(std::abs(bNav.pos().z - size), std::abs(bNav.pos().z + size));

        // float closestDist = std::min({xDist, yDist, zDist});
        float turnRate = turnRateFactor * 0.5;// / closestDist;

        if (dist < 0.0) {
            bNav.quat().set(rnd::uniformS(), rnd::uniformS(), rnd::uniformS(), rnd::uniformS()).normalize();
            bNav.faceToward(Vec3d(0,0,0), bNav.uu(), turnRate + turnRate*sqrt(proximity));
        } else if (dist < minEdgeProximity) {            
            bNav.faceToward(Vec3d(0,0,0), bNav.uu(), turnRate*proximity);
        }
    }

    // void originAvoidance(float avoidanceRadius) {
    //     float dist = bNav.pos().mag();
    //     // if the boid is too close to the origin and not moving away, turn away
    //     // this means that if the boid is within the distance but leaving the origin, dont turn away
    //     if (dist < avoidanceRadius) {
    //         // how much the boid is heading away from the origin
    //         float theta = bNav.uf().dot(bNav.pos().normalize());
    //         if (theta > 0.0) { // if the boid is moving away from the origin
    //             // steer them slightly more away from the origin proportional to how close they are and where they are heading
    //             float turnRate = turnRateFactor * (1.0 - dist / avoidanceRadius);
    //             Vec3d away = (bNav.pos().normalize() * 0.1) / dist;
    //             // theta determines how much the boid is heading away from the origin
    //             // the more it is heading away, the more it should turn away

    //         } else {
    //             // turnRate is a function of both proximity to the origin and how much the boid is heading away (theta)
    //             // the more the boid is facing the origin, the more it should turn away
    //             float turnRate = turnRateFactor * (1.0 - dist / avoidanceRadius);
    //             // Vec3d away = (bNav.pos().normalize() * (th              


    //             bNav.faceToward(away, bNav.uu(), turnRate);
    //         // bNav.faceToward(away, bNav.uu(), turnRate);
    //         }
    //     }
    // }

    void originAvoidance(float avoidanceRadius) {
        Vec3d pos = bNav.pos(); // Position of the boid relative to the origin
        Vec3d forward = bNav.uf(); // Forward direction of the boid
        float dist = pos.mag(); // Distance from the origin

        if (dist < avoidanceRadius) {
            Vec3d awayFromOrigin = pos.normalize(); // Direction away from the origin
            float theta = forward.dot(awayFromOrigin); // Measure of orientation towards the origin

            if (theta < 0) { // If facing towards the origin, theta is negative
                // Make theta positive and scale by proximity to the origin for the turn rate calculation
                theta = -theta; // Convert to positive to indicate "degree" of facing towards the origin
                float proximityFactor = 1.0 - (dist / avoidanceRadius); // Scales up as the boid gets closer to the origin

                // Calculate turn rate as a function of both theta and proximity to the origin
                // This ensures the turn rate is higher when the boid is closer and more directly facing the origin
                float turnRate = std::min(theta * proximityFactor, 1.0f); // Clamp turn rate to a maximum of 1

                // Determine the new direction by biasing the forward vector away from the origin
                Vec3d biasedForward = forward + awayFromOrigin * theta;
                biasedForward.normalize(); // Ensure the direction is normalized

                // Steer the boid towards this new direction
                bNav.faceToward(pos + biasedForward, bNav.uu(), turnRate);
            }
        }
    }

    void originAvoidance(float avoidanceRadius, float preparationRadius) {
        Vec3d pos = bNav.pos(); // Position of the boid relative to the origin
        float dist = pos.mag(); // Distance from the origin
        if (dist < preparationRadius) {
            Vec3d forward = bNav.uf(); // Forward direction of the boid
            Vec3d biasedForward = Vec3d(0, 0, 0);
            Vec3d awayFromOrigin = pos.normalize(); // Direction away from the origin
            float theta = forward.dot(awayFromOrigin); // Measure of orientation towards the origin
            float turnRate = turnRateFactor * (1.0 - dist / preparationRadius);
            if (theta < 0) { 
                theta = -theta; 
                float proximityFactor = 1.0 - (dist / avoidanceRadius);
                biasedForward = forward + awayFromOrigin * theta * proximityFactor;
                biasedForward.normalize(); // Ensure the direction is normalized
                bNav.faceToward(pos + biasedForward, bNav.uu(), 0.005);
                if (dist < avoidanceRadius) {
                    turnRate = turnRateFactor * (1.0 - dist / avoidanceRadius);
                    biasedForward = forward + awayFromOrigin * theta * proximityFactor;
                    bNav.faceToward(bNav.pos() + biasedForward, bNav.uu(), turnRate);
                }                
            }

            
            // Vec3d biasedForward = forward + awayFromOrigin * std::abs(theta);
            // biasedForward.normalize(); // Ensure the direction is normalized

            // // Steer the boid towards this new direction
            // bNav.faceToward(pos + biasedForward, bNav.uu(), turnRate);
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

            // Adjust the turn rate based on the seriousness of the oncoming threat
            float turnRate = std::min(turnRateFactor * seriousnessFactor, 1.0f);

            // Steer the boid in the perpendicular direction to avoid collision
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

            // Calculate angles to assess collision risk
            float angleToOther = acos(bNav.uf().dot(toOther.normalize()));
            float angleFromOther = acos(otherHeading.dot(-toOther.normalize()));

            // Determine risk level based on angles
            float riskLevel = 0.0f;
            if (angleToOther < M_PI / 4 && angleFromOther < M_PI / 4) { // Head-on collision
                riskLevel = 1.0f;
            } else if (std::abs(angleToOther - angleFromOther) < M_PI / 6) { // High risk of perpendicular collision
                riskLevel = 0.5f; // Adjust risk level based on scenario
            }

            // Adjust avoidance direction based on risk level
            if (riskLevel > highestRisk) {
                highestRisk = riskLevel;
                avoidanceDirection = toOther.cross(bNav.uf()).normalize(); // Example avoidance direction
            }
        }

        if (highestRisk > 0) {
            // Scale turn rate by risk level
            float turnRate = highestRisk; // Simplified for example, adjust as needed

            // Steer away based on calculated avoidance direction
            bNav.faceToward(bNav.pos() + avoidanceDirection, bNav.uu(), turnRate);
        }
    }




    // XXX - TODO:  boid flocking dynamics
    void alignment(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f averageHeading(0, 0, 0);
        int alignCount = 0;
        for (int i : i_navs) {
            float dist = (bNav.pos() - navs[i]->pos()).mag();
            if (dist < 11.0) { 
                averageHeading += navs[i]->uf();
                alignCount++;
            }
        }
        if (alignCount > 0) {
            averageHeading /= alignCount;
            // Normalize to get direction and apply alignment
            float turnRate = turnRateFactor * (bNav.pos() - bNav.pos() + averageHeading).mag() / 30.0;
            bNav.faceToward(bNav.pos() + averageHeading.normalized(), bNav.uu(), turnRate);
        }
    }

    void separation(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f separationForce(0, 0, 0);
        int closeBoids = 0;
        for (int i : i_navs) {
            float dist = (bNav.pos() - navs[i]->pos()).mag();
            if (dist < 6.0) {
                Vec3f away = (bNav.pos() - navs[i]->pos()).normalize() / dist;
                separationForce += away;
                closeBoids++;
            }
        }
        if (closeBoids > 0) {            
            separationForce /= closeBoids;
            // Apply the separation force to adjust boid's direction
            bNav.faceToward(bNav.pos() + separationForce, bNav.uu(), 0.75);
        }
    }

    void cohesion(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f centerOfMass(0, 0, 0);
        int cohesionCount = 0;
        for (int i : i_navs) {
            float dist = (bNav.pos() - navs[i]->pos()).mag();
            if (dist > 10.0) {
                centerOfMass += navs[i]->pos();
                cohesionCount++;
            }
        }
        if (cohesionCount > 0) {
            centerOfMass /= cohesionCount;
            // Move towards the center of mass of nearby boids
            float turnRate = std::min((bNav.pos() - centerOfMass).mag() / 30.0, 0.75);
            bNav.faceToward(centerOfMass, Vec3f(0, 1, 0), turnRate);
        }
    }

    void detectSurroundings(const Octree& tree, float size, const std::vector<Nav*>& navs) {
        vector<int> i_navs;
        tree.queryRegion(bNav.pos(), Vec3f(13, 13, 13), i_navs);

        heading(navs, i_navs);
        detectCollisions(navs, i_navs);
        alignment(navs, i_navs);
        cohesion(navs, i_navs);
        separation(navs, i_navs);

        handleBoundary(size*1.333);
        originAvoidance(9.0, 15.0);
    }

    void findFood(const Octree& tree, float size, const std::vector<Vec3f>& food, const std::vector<float>& mass) {
        hunger = (hunger > 1.0) ? hunger = 1.0 : hunger; 
        if ((target - bNav.pos()).magSqr() < 0.1) {
            hunger -= 0.0001; // make proportional to mass of food
            target = Vec3d(rnd::uniformS(), rnd::uniformS(), rnd::uniformS());
        }
        vector<int> i_food;
        tree.queryRegion(bNav.pos(), Vec3f(size, size, size), i_food);
        float si = i_food.size();
        if (si > 0) {
            int biggestFood = i_food[0];
            for (int i : i_food) {
                // float foodMass = mass[i];
                if (mass[i] > mass[biggestFood]) {
                    biggestFood = i;
                }
                hunger += i / si; // make proportional to mass of food
            }
            seek(food[biggestFood], 0.31);
        }        
    }

    void seek(Vec3d a, double amt, float smooth = 0.1) { 
        target.set(a);
        bNav.smooth(smooth);
        bNav.faceToward(target, Vec3d(0, 1, 0), amt);
    }

    void updatePosition(double v, double dt) {
        bNav.moveF(v);
        bNav.step(dt);        
    }

    virtual void updateParams(float dts) {
        if (age > lifespan) {
            lifeStatus = false;
        }
        age += ageRate*(dts/5.0) + (ageRate * fear);  // Fear increases rate of aging
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
