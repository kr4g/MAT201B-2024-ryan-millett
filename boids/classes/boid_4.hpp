#pragma once

#include <vector>

#include "al/io/al_ControlNav.hpp"
#include "al/math/al_Vec.hpp"

using namespace al;

enum class BoidType {
  SMALL_PREY = 0,  // Green, fast, twitchy
  LARGE_PREY = 1,  // Blue, slower, stable
  PREDATOR = 2     // Red, large, cluster-seeking
};

class Boid {
 public:
  Nav bNav;
  Vec3d target;
  BoidType type{BoidType::SMALL_PREY};
  float hunger{0.0f};
  float hungerRate{1.0f};

  float turnRateFactor{0.15f};
  float maxSpeed{1.5f};
  float baseSpeed{1.5f};
  bool huntingMode{false};
  bool panicMode{false};
  bool beingHunted{false};
  float huntingThreshold{0.7f};
  Vec3f targetCluster{0, 0, 0};
  Vec3f wanderTarget{0, 0, 0};

  std::vector<int> i_boids;

  Boid(BoidType boidType = BoidType::SMALL_PREY) : type(boidType) {
    switch(type) {
      case BoidType::SMALL_PREY:
        turnRateFactor = 0.15f;
        maxSpeed = baseSpeed = 2.4f;
        hungerRate = 0.8f + (rnd::uniformS() * 0.4f);
        break;
      case BoidType::LARGE_PREY:
        turnRateFactor = 0.08f;
        maxSpeed = baseSpeed = 1.0f;
        hungerRate = 0.6f + (rnd::uniformS() * 0.4f);
        break;
      case BoidType::PREDATOR:
        turnRateFactor = 0.1f;
        maxSpeed = baseSpeed = 1.8f;
        hungerRate = 1.0f + (rnd::uniformS() * 0.8f);
        break;
    }
  }

  void handleBoundary(float size)
  {
    Vec3d pos = bNav.pos();
    float dist = pos.mag();
    float boundary = size * 0.75f;
    
    if (dist > boundary) {
      Vec3d toOrigin = -pos.normalized();
      Vec3d currentDir = bNav.uf();
      
      float edgeProximity = (dist - boundary) / (size * 0.2f);
      edgeProximity = std::min(edgeProximity, 1.0f);
      
      Vec3d desiredDir = currentDir * (1.0f - edgeProximity * 0.8f) + toOrigin * edgeProximity;
      desiredDir.normalize();
      
      float turnRate = 0.1f + edgeProximity * 0.4f;
      bNav.faceToward(pos + desiredDir * 5.0, bNav.uu(), turnRate);
    }
  }

  void originAvoidance(float avoidanceRadius)
  {
    Vec3d pos = bNav.pos();
    float dist = pos.mag();

    if (dist < avoidanceRadius) {
      Vec3d tangent = pos.cross(Vec3d(0, 1, 0));
      if (tangent.mag() < 0.001) {
        tangent = pos.cross(Vec3d(1, 0, 0));
      }
      tangent.normalize();
      
      float proximityFactor = 1.0f - (dist / avoidanceRadius);
      float turnRate = turnRateFactor * (0.5f + proximityFactor * 1.5f);
      
      Vec3d swimDirection = tangent + pos.normalized() * 0.3;
      swimDirection.normalize();
      
      bNav.faceToward(pos + swimDirection * 5.0, bNav.uu(), turnRate);
    }
  }

  Vec3f findNearbyPreyCenter(const std::vector<Boid>& boids, const std::vector<int>& nearbyBoids, int minPreyCount = 6) {
    Vec3f centerOfMass(0, 0, 0);
    int preyCount = 0;
    
    for (int i : nearbyBoids) {
      if (boids[i].type != BoidType::PREDATOR) {
        centerOfMass += Vec3f(boids[i].bNav.pos());
        preyCount++;
      }
    }
    
    if (preyCount >= minPreyCount) {
      return centerOfMass / preyCount;
    }
    
    return Vec3f(0, 0, 0);
  }

  void checkIfBeingHunted(const std::vector<Boid>& boids) {
    beingHunted = false;
    for (int i : i_boids) {
      if (boids[i].type == BoidType::PREDATOR && boids[i].huntingMode) {
        float distToTarget = al::dist(boids[i].targetCluster, Vec3f(bNav.pos()));
        if (distToTarget < 8.0f) {
          beingHunted = true;
          break;
        }
      }
    }
  }

  void avoidPredators(const std::vector<Boid>& boids, float avoidRadius = 12.0f) {
    Vec3f avoidance(0, 0, 0);
    int predatorCount = 0;
    float minDistance = avoidRadius;
    
    for (int i : i_boids) {
      if (boids[i].type == BoidType::PREDATOR) {
        Vec3f diff = Vec3f(bNav.pos()) - Vec3f(boids[i].bNav.pos());
        float distance = diff.mag();
        
        if (distance < avoidRadius && distance > 0.001f) {
          minDistance = std::min(minDistance, distance);
          diff.normalize();
          float urgency = 1.0f - (distance / avoidRadius);
          
          if (beingHunted) {
            urgency *= 1.8f;
          }
          
          diff *= urgency * urgency;
          avoidance += diff;
          predatorCount++;
        }
      }
    }
    
    if (predatorCount > 0) {
      panicMode = true;
      hunger = 0.0f;
      avoidance /= predatorCount;
      avoidance.normalize();
      
      float panicLevel = 1.0f - (minDistance / avoidRadius);
      panicLevel = panicLevel * panicLevel;
      
      if (beingHunted) {
        panicLevel *= 1.5f;
        maxSpeed = baseSpeed * (2.0f + panicLevel * 1.0f);
      } else {
        maxSpeed = baseSpeed * (1.6f + panicLevel * 0.8f);
      }
      
      Vec3f currentPos = Vec3f(bNav.pos());
      float panicTurnRate = turnRateFactor * (beingHunted ? 1.8f : 1.2f) * (1.0f + panicLevel * 0.8f);
      bNav.faceToward(currentPos + avoidance * (beingHunted ? 8.0f : 6.0f), bNav.uu(), panicTurnRate);
    } else {
      panicMode = false;
      maxSpeed = baseSpeed;
    }
  }

  Vec3f generateWanderTarget(float cubeSize) {
    float range = cubeSize * 0.7f;
    return Vec3f(
      (rnd::uniformS() * range),
      (rnd::uniformS() * range), 
      (rnd::uniformS() * range)
    );
  }

  void updateHunger(float globalHungerRate) {
    hunger += 0.001f * globalHungerRate * hungerRate;
    hunger = std::min(hunger, 1.0f);
  }

  float getFoodAttractionStrength(float baseFoodAttraction) {
    if (type == BoidType::PREDATOR) return 0.0f;
    if (hunger < 0.1f) return 0.0f;
    return baseFoodAttraction * hunger;
  }

  bool isNearFood(const std::vector<Vec3f>& food, float criticalDistance = 5.0f) {
    if (type == BoidType::PREDATOR) return false;
    Vec3f myPos = Vec3f(bNav.pos());
    for (const auto& foodPos : food) {
      if (al::dist(myPos, foodPos) < criticalDistance) {
        return true;
      }
    }
    return false;
  }

  void boidForces(const std::vector<Boid>& boids, const std::vector<Vec3f>& food,
                  float alignmentForce = 0.5, float cohesionForce = 0.5, 
                  float separationForce = 0.5)
  {
    if (this->i_boids.empty()) return;

    updateHunger(1.0f);

    if (type == BoidType::PREDATOR) {
      if (hunger > huntingThreshold) {
        Vec3f clusterCenter = findNearbyPreyCenter(boids, i_boids, 6);
        if (clusterCenter.mag() > 0.001f) {
          huntingMode = true;
          targetCluster = clusterCenter;
          maxSpeed = baseSpeed * 2.5f;
          seek(targetCluster, 0.3);
          
          if (al::dist(bNav.pos(), targetCluster) < 3.0f) {
            hunger -= 0.3f;
            hunger = std::max(hunger, 0.0f);
            if (hunger < 0.2f) {
              huntingMode = false;
              targetCluster = Vec3f(0, 0, 0);
            }
          }
        } else {
          huntingMode = false;
          maxSpeed = baseSpeed * 0.8f;
          
          if (wanderTarget.mag() < 0.001f || al::dist(bNav.pos(), wanderTarget) < 5.0f) {
            wanderTarget = generateWanderTarget(30.0f);
          }
          seek(wanderTarget, 0.05);
        }
      } else {
        huntingMode = false;
        targetCluster = Vec3f(0, 0, 0);
        maxSpeed = baseSpeed * 0.6f;
        
        if (wanderTarget.mag() < 0.001f || al::dist(bNav.pos(), wanderTarget) < 5.0f) {
          wanderTarget = generateWanderTarget(30.0f);
        }
        seek(wanderTarget, 0.03);
      }
      
      Vec3f predatorAvoidance(0, 0, 0);
      int predatorCount = 0;
      for (int i : i_boids) {
        if (boids[i].type == BoidType::PREDATOR && &boids[i] != this) {
          Vec3f diff = Vec3f(bNav.pos()) - Vec3f(boids[i].bNav.pos());
          float distance = diff.mag();
          if (distance < 25.0f && distance > 0.001f) {
            diff.normalize();
            diff /= (distance * 0.3f);
            predatorAvoidance += diff;
            predatorCount++;
          }
        }
      }
      if (predatorCount > 0) {
        predatorAvoidance /= predatorCount;
        predatorAvoidance.normalize();
        Vec3f currentPos = Vec3f(bNav.pos());
        bNav.faceToward(currentPos + predatorAvoidance * 3.0f, bNav.uu(), 0.6f * turnRateFactor);
      }
      return;
    }

    checkIfBeingHunted(boids);
    avoidPredators(boids);
    
    if (panicMode) {
      return;
    }

    Vec3f separationSum(0, 0, 0);
    Vec3f alignmentSum(0, 0, 0);
    Vec3f cohesionSum(0, 0, 0);
    
    Vec3f myPos = this->bNav.pos();
    Vec3f myVel = this->bNav.uf();
    
    int separationCount = 0;
    int alignmentCount = 0;
    int cohesionCount = 0;

    for (int i : this->i_boids) {
      if (boids[i].type == BoidType::PREDATOR) continue;
      
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
      Vec3f newDirection = myVel + steering * turnRateFactor;
      newDirection.normalize();
      float effectiveTurnRate = turnRateFactor;
      if (isNearFood(food, 5.0f)) {
        effectiveTurnRate *= 2.5f;
      }
      this->bNav.faceToward(myPos + newDirection, this->bNav.uu(), effectiveTurnRate);
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
    amt = std::min(amt * maxSpeed, 3.0);
    bNav.moveF(amt);
    bNav.step(dt);
  }
};
