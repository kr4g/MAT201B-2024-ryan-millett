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
    float hunger{1.0f};
    float fear{0.0f};
    float mutation{0.0f};
    float mutationRate{0.0f};
    float age{0.0f};
    float ageRate{0.001f};
    float lifespan;

    float minEdgeProximity{9.5f};     // Minimum distance from edge to start turning
    float turnRateFactor{0.23f};       // Factor to adjust turning rate

    // Boid() : {}
    
    ~Boid() {}

    void handleBoundary(float size) {
        float xDist = std::min(std::abs(bNav.pos().x - size), std::abs(bNav.pos().x + size));
        float yDist = std::min(std::abs(bNav.pos().y - size), std::abs(bNav.pos().y + size));
        float zDist = std::min(std::abs(bNav.pos().z - size), std::abs(bNav.pos().z + size));

        float closestDist = std::min({xDist, yDist, zDist});
        float turnRate = turnRateFactor * 0.5;// / closestDist;

        float proximity = (size - xDist) / size;
        if (xDist < minEdgeProximity) {
            bNav.faceToward(Vec3d(-bNav.pos().x, bNav.pos().y, bNav.pos().z), bNav.uu(), turnRate*sqrt(proximity));
            if (xDist < 1.75) { 
                bNav.quat().set(rnd::uniformS(), bNav.quat().y, bNav.quat().z, rnd::uniformS()).normalize();
            }
        } 

        proximity = (size - yDist) / size;
        if (yDist < minEdgeProximity) {
            bNav.faceToward(Vec3d(bNav.pos().x, -bNav.pos().y, bNav.pos().z), bNav.uu(), turnRate*sqrt(proximity));
            if (yDist < 1.75) { 
                bNav.quat().set(bNav.quat().x, rnd::uniform(), bNav.quat().z, rnd::uniformS()).normalize();
            }
        } 
        
        proximity = (size - zDist) / size;
        if (zDist < minEdgeProximity) {
            bNav.faceToward(Vec3d(bNav.pos().x, bNav.pos().y, -bNav.pos().z), bNav.uu(), turnRate*sqrt(proximity));
            if (zDist < 1.75) { 
                bNav.quat().set(bNav.quat().x, bNav.quat().y, rnd::uniformS(), rnd::uniformS()).normalize();
            }
        }
    }

    // void originAvoidance(float size) {
    //     float dist = bNav.pos().mag();
    //     float turnRate = turnRateFactor * (1.0 - dist / size);
    //     if (dist < 1.75) {
    //         Vec3d away = bNav.pos().normalize() / dist;
    //         bNav.faceToward(away, bNav.uu(), turnRate);
    //     }
    // }
    void originAvoidance(float avoidanceRadius) {
        float dist = bNav.pos().mag(); // Distance from the origin
        if (dist < avoidanceRadius) {     
            Vec3d away = bNav.pos().normalize();
            float turnRate = sqrt(turnRateFactor) * (1.0 - dist / avoidanceRadius);
            bNav.faceToward(away, bNav.uu(), turnRate);
        }
    }

    void heading(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        // if heading into other oncoming boids (ie, boids not heading in the same direction), turn away
        for (int i : i_navs) {
            // determine how oncoming the boid is (ie, how much it is heading towards the boid in question)
            Vec3f otherPos = navs[i]->pos();
            Vec3f otherHeading = navs[i]->uf();
            // the more oncoming, the more it should turn away
            float oncoming = bNav.uf().dot(otherHeading);
            float turnRate = turnRateFactor * (otherPos - bNav.pos()).mag() / 20.0;
            if (oncoming < 0.0) {
                bNav.faceToward(bNav.pos() + otherHeading, bNav.uu(), turnRate);
            }
        }    
    }

    void alignment(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f averageHeading(0, 0, 0);
        int alignCount = 0;
        for (int i : i_navs) {
            float dist = (bNav.pos() - navs[i]->pos()).mag();
            if (dist < 5.0) { 
                averageHeading += navs[i]->uf();
                alignCount++;
            }
        }
        if (alignCount > 0) {
            averageHeading /= alignCount;
            // Normalize to get direction and apply alignment
            float turnRate = turnRateFactor * (bNav.pos() - bNav.pos() + averageHeading).mag() / 20.0;
            bNav.faceToward(bNav.pos() + averageHeading.normalized(), bNav.uu(), turnRate);
        }
    }

    void separation(const std::vector<Nav*>& navs, const std::vector<int>& i_navs) {
        Vec3f separationForce(0, 0, 0);
        int closeBoids = 0;
        for (int i : i_navs) {
            float dist = (bNav.pos() - navs[i]->pos()).mag();
            if (dist < 2.667) {
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
            if (dist > 5.0) {
                centerOfMass += navs[i]->pos();
                cohesionCount++;
            }
        }
        if (cohesionCount > 0) {
            centerOfMass /= cohesionCount;
            // Move towards the center of mass of nearby boids
            float turnRate = std::min((bNav.pos() - centerOfMass).mag() / 25.0, 0.75);
            bNav.faceToward(centerOfMass, Vec3f(0, 1, 0), turnRate);
        }
    }

    void detectSurroundings(const Octree& tree, float size, const std::vector<Nav*>& navs) {
        vector<int> i_navs;
        tree.queryRegion(bNav.pos(), Vec3f(12, 12, 12), i_navs);

        heading(navs, i_navs);
        alignment(navs, i_navs);
        cohesion(navs, i_navs);
        separation(navs, i_navs);

        handleBoundary(size*1.1667);
        // originAvoidance(5.0);
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
        bNav.faceToward(target, bNav.uu(), amt);
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
