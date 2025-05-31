#pragma once

#include <vector>

#include "al/io/al_ControlNav.hpp"
#include "al/math/al_Vec.hpp"

constexpr float MIN_PREY_EDGE_PROXIMITY = 0.01f;
constexpr float MIN_PREDATOR_EDGE_PROXIMITY = 0.05f;

constexpr float MAX_PREY_TURN_RATE = 0.1f;
constexpr float MAX_PREDATOR_TURN_RATE = 0.2f;

using namespace al;

class Boid {
 public:
  Nav bNav;
  Vec3d target;
  bool lifeStatus{true};
  float frequency{5.0f};
  float attentionSpan{1.0f};
  float hunger{1.0f};
  float fear{0.0f};
  int meshIdx;
  int Nv;

  float minEdgeProximity{1.5f};
  float turnRateFactor{0.15f};
  float maxSpeed{1.5f};

  std::vector<int> i_boids;

  void handleBoundary(float size)
  {
    Vec3d pos = bNav.pos();
    float dist = pos.mag();
    float boundary = size * 0.85f;
    
    if (dist > boundary) {
      Vec3d awayFromWall = -pos.normalized();
      Vec3d currentDir = bNav.uf();
      
      float edgeProximity = (dist - boundary) / (size - boundary);
      edgeProximity = std::min(edgeProximity, 1.0f);
      
      Vec3d desiredDir = currentDir * (1.0f - edgeProximity) + awayFromWall * edgeProximity;
      desiredDir.normalize();
      
      float turnRate = 0.1f + edgeProximity * 0.9f;
      bNav.faceToward(pos + desiredDir, bNav.uu(), turnRate);
    }
  }

  void originAvoidance(float avoidanceRadius)
  {
    Vec3d pos = bNav.pos();
    Vec3d forward = bNav.uf();
    float dist = pos.mag();

    if (dist < avoidanceRadius) {
      Vec3d awayFromOrigin = pos.normalize();
      float theta = forward.dot(awayFromOrigin);

      if (theta < 0) {
        theta = -theta;
        float proximityFactor = 1.0 - (dist / avoidanceRadius);
        float turnRate = std::min(theta * proximityFactor, 1.0f);
        Vec3d biasedForward = forward + awayFromOrigin * theta;
        biasedForward.normalize();
        bNav.faceToward(pos + biasedForward, bNav.uu(), turnRate);
      }
    }
  }

  void originAvoidance(float avoidanceRadius, float preparationRadius)
  {
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

  void boidForces(const std::vector<Boid>& boids, float alignmentForce = 0.5,
                  float cohesionForce = 0.5, float separationForce = 0.5,
                  float turnRate = 0.5)
  {
    if (this->i_boids.empty()) return;

    Vec3f separationSum(0, 0, 0);
    Vec3f alignmentSum(0, 0, 0);
    Vec3f cohesionSum(0, 0, 0);
    
    Vec3f myPos = this->bNav.pos();
    Vec3f myVel = this->bNav.uf();
    
    int separationCount = 0;
    int alignmentCount = 0;
    int cohesionCount = 0;

    for (int i : this->i_boids) {
      const Boid& neighbor = boids[i];
      Vec3f neighborPos = neighbor.bNav.pos();
      Vec3f diff = myPos - neighborPos;
      float distance = diff.mag();
      
      if (distance > 0.001f) {
        if (distance < 2.0f) {
          diff.normalize();
          diff /= distance;
          separationSum += diff;
          separationCount++;
        }
        
        if (distance < 4.0f) {
          alignmentSum += neighbor.bNav.uf();
          alignmentCount++;
        }
        
        if (distance < 5.0f) {
          cohesionSum += neighborPos;
          cohesionCount++;
        }
      }
    }

    Vec3f steering(0, 0, 0);
    
    if (separationCount > 0) {
      separationSum /= separationCount;
      steering += separationSum * separationForce;
    }
    
    if (alignmentCount > 0) {
      alignmentSum /= alignmentCount;
      alignmentSum.normalize();
      steering += (alignmentSum - myVel) * alignmentForce;
    }
    
    if (cohesionCount > 0) {
      cohesionSum /= cohesionCount;
      Vec3f cohesionDir = (cohesionSum - myPos).normalize();
      steering += cohesionDir * cohesionForce;
    }

    if (steering.mag() > 0.001f) {
      steering.normalize();
      Vec3f newDirection = myVel + steering * turnRate;
      newDirection.normalize();
      this->bNav.faceToward(myPos + newDirection, this->bNav.uu(), turnRate * turnRateFactor);
    }
  }

  void seek(const Vec3d& a, double amt, float smooth = 0.1)
  {
    target.set(a);
    bNav.smooth(smooth);
    bNav.faceToward(target, bNav.uu(), amt);
  }

  void updatePosition(double dt, double amt)
  {
    amt = std::min(amt * maxSpeed, 2.0);
    bNav.moveF(amt);
    bNav.step(dt);
  }
};
