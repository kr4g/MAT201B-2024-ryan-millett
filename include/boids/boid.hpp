#pragma once

#include <algorithm>
#include <vector>

#include "al/io/al_ControlNav.hpp"
#include "al/math/al_Vec.hpp"

using namespace al;

enum class BoidType : uint8_t { SMALL_PREY = 0, LARGE_PREY = 1, PREDATOR = 2 };

union BoidMode {
  struct {
    bool huntingMode  : 1;
    bool panicMode    : 1;
    bool beingHunted  : 1;
    bool foragingMode : 1;
    BoidType type     : 4;
  };
  uint8_t value{0};
};

class Boid {
 public:
  Nav bNav;
  BoidMode mode{0};
  Vec3f target{0};

  float hunger{0.0f};
  float hungerRate{1.0f};
  float turnRateFactor{0.15f};
  float maxSpeed{1.5f};
  float baseSpeed{1.5f};
  float huntingThreshold{0.7f};

  std::vector<int> i_boids;

  Boid(const BoidType boidType = BoidType::SMALL_PREY)
      : mode{false, false, false, false, boidType}
  {
    switch (mode.type) {
      case BoidType::SMALL_PREY:
        turnRateFactor = 0.15f;
        maxSpeed = baseSpeed = 2.4f;
        hungerRate = 0.8f + (rnd::uniformS() * 0.4f);
        break;
      case BoidType::LARGE_PREY:
        turnRateFactor = 0.08f;
        maxSpeed = baseSpeed = 2.0f;
        hungerRate = 0.6f + (rnd::uniformS() * 0.4f);
        break;
      case BoidType::PREDATOR:
        turnRateFactor = 0.1f;
        maxSpeed = baseSpeed = 2.2f;
        hungerRate = 1.0f + (rnd::uniformS() * 0.8f);
        break;
    }
  }

  void handleBoundary(const float size)
  {
    Vec3d pos = bNav.pos();
    float dist = pos.mag();
    float boundary = size * 0.75f;

    if (dist > boundary) {
      Vec3d toOrigin = -pos.normalized();
      Vec3d currentDir = bNav.uf();

      float edgeProximity = (dist - boundary) / (size * 0.2f);
      edgeProximity = std::min(edgeProximity, 1.0f);

      Vec3d desiredDir =
          currentDir * (1.0f - edgeProximity * 0.8f) + toOrigin * edgeProximity;
      desiredDir.normalize();

      float turnRate = 0.1f + edgeProximity * 0.4f;
      bNav.faceToward(pos + desiredDir * 5.0, bNav.uu(), turnRate);
    }
  }

  void originAvoidance(const float avoidanceRadius)
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

  const Vec3f newWanderTarget(const float cubeSize) const
  {
    float range = cubeSize * 0.7f;
    return Vec3f((rnd::uniformS() * range), (rnd::uniformS() * range),
                 (rnd::uniformS() * range));
  }

  void updateHunger(const float globalHungerRate)
  {
    hunger += 0.001f * globalHungerRate * hungerRate;
    hunger = std::min(hunger, 1.0f);
  }

  void updateSpeed()
  {
    if (mode.type == BoidType::PREDATOR) return;

    float minSpeed, maxSpeed;
    if (mode.type == BoidType::LARGE_PREY) {
      minSpeed = 1.5f;
      maxSpeed = 2.0f;
    }
    else {
      minSpeed = 1.8f;
      maxSpeed = 2.4f;
    }

    baseSpeed = minSpeed + (maxSpeed - minSpeed) * hunger;
    if (!mode.panicMode && !mode.huntingMode) {
      this->maxSpeed = baseSpeed;
    }
  }

  // const bool nearFood(const std::vector<Vec3f>& food,
  //                     const float criticalDistance = 5.0f) const
  // {
  //   if (mode.type == BoidType::PREDATOR) return false;
  //   Vec3f myPos = Vec3f(bNav.pos());

  //   for (const auto& foodPos : food) {
  //     if (al::dist(myPos, foodPos) < criticalDistance) {
  //       return true;
  //     }
  //   }
  //   return false;
  // }

  // void checkFoodConsumption(const std::vector<Vec3f>& food,
  //                           const float consumeDistance)
  // {
  //   if (mode.type == BoidType::PREDATOR) {
  //     targetFood = Vec3f(0, 0, 0);
  //     return;
  //   }

  //   Vec3f myPos = Vec3f(bNav.pos());
  //   targetFood = Vec3f(0, 0, 0);

  //   for (const auto& foodPos : food) {
  //     if (al::dist(myPos, foodPos) < consumeDistance) {
  //       targetFood = foodPos;
  //       break;
  //     }
  //   }
  // }

  // void setFoodTarget(const Vec3f& foodPos) { targetFood = foodPos; }

  // void clearFoodTarget() { targetFood = Vec3f(0, 0, 0); }

  void boidForces(const std::vector<Boid>& boids,
                  const std::vector<Vec3f>& food,
                  const float alignmentForce = 0.5,
                  const float cohesionForce = 0.5,
                  const float separationForce = 0.5,
                  const float cubeSize = 20.0f, const float visionRadius = 5.0f)
  {
    if (this->i_boids.empty()) return;

    updateHunger(1.0f);
    updateSpeed();

    if (mode.type != BoidType::PREDATOR && hunger > 0.3f && !food.empty()) {
      Vec3f nearestFood(0, 0, 0);
      float minFoodDist = visionRadius * 0.5f;
      bool foundFood = false;

      Vec3f myPos(bNav.pos());
      for (const auto& foodPos : food) {
        float dist = al::dist(myPos, foodPos);
        if (dist < minFoodDist) {
          minFoodDist = dist;
          nearestFood = foodPos;
          foundFood = true;
        }
      }

      if (foundFood) {
        mode.foragingMode = true;
        target = nearestFood;
        maxSpeed = baseSpeed * 1.3f;
        seek(target, 0.4);

        if (minFoodDist < visionRadius * 0.1f) {
          hunger -= 0.4f;
          hunger = std::max(hunger, 0.0f);
          if (hunger < 0.2f) {
            mode.foragingMode = false;
            target.set(0);
          }
        }
      }
      else if (!mode.panicMode) {
        mode.foragingMode = false;
        target.set(0);
      }
    }
    else if (mode.type != BoidType::PREDATOR) {
      mode.foragingMode = false;
      target.set(0);
    }

    Vec3f myPos(bNav.pos());
    Vec3f myVel(bNav.uf());

    Vec3f predatorAvoidance(0, 0, 0);
    Vec3f preyCenter(0, 0, 0);
    Vec3f separationSum(0, 0, 0);
    Vec3f alignmentSum(0, 0, 0);
    Vec3f cohesionSum(0, 0, 0);
    Vec3f predatorSteering(0, 0, 0);

    int predatorCount = 0;
    int preyCount = 0;
    int separationCount = 0;
    int alignmentCount = 0;
    int cohesionCount = 0;
    int nearbyPredators = 0;

    bool hunted = false;
    float predatorAvoidRadius = visionRadius * 2.4f;
    float minPredatorDist = predatorAvoidRadius;

    for (int i : this->i_boids) {
      const Boid& neighbor = boids[i];
      Vec3f neighborPos(neighbor.bNav.pos());
      Vec3f diff = myPos - neighborPos;
      float distance = diff.mag();

      if (neighbor.mode.type == BoidType::PREDATOR) {
        if (mode.type != BoidType::PREDATOR) {
          if (distance < predatorAvoidRadius && distance > 0.001f) {
            minPredatorDist = std::min(minPredatorDist, distance);
            diff.normalize();
            float urgency = 1.0f - (distance / predatorAvoidRadius);

            if (mode.beingHunted) {
              urgency *= 1.8f;
            }

            diff *= urgency * urgency;
            predatorAvoidance += diff;
            predatorCount++;
          }

          if (!neighbor.mode.huntingMode && distance < visionRadius * 1.5f &&
              distance > 0.001f) {
            Vec3f gentleDiff = diff;
            gentleDiff.normalize();
            float gentleUrgency = 1.0f - (distance / (visionRadius * 1.5f));
            gentleDiff *= gentleUrgency * 0.3f;
            predatorSteering += gentleDiff;
            nearbyPredators++;
          }

          if (neighbor.mode.huntingMode) {
            float distToTarget = al::dist(neighbor.target, myPos);
            if (distToTarget < visionRadius * 1.6f) {
              hunted = true;
            }
          }
        }
        else if (&neighbor != this) {
          float predatorSpacing = visionRadius * 5.0f;
          if (distance < predatorSpacing && distance > 0.001f) {
            diff.normalize();
            diff /= (distance * 0.3f);
            predatorAvoidance += diff;
            predatorCount++;
          }
        }
      }
      else {
        if (mode.type == BoidType::PREDATOR) {
          preyCenter += neighborPos;
          preyCount++;
        }
        else if (distance > 0.001f) {
          if (distance < visionRadius * 0.4f) {
            diff.normalize();
            diff /= distance;
            separationSum += diff;
            separationCount++;
          }

          if (distance < visionRadius * 0.8f) {
            alignmentSum += neighbor.bNav.uf();
            alignmentCount++;
          }

          if (distance < visionRadius) {
            cohesionSum += neighborPos;
            cohesionCount++;
          }
        }
      }
    }

    if (mode.type == BoidType::PREDATOR) {
      if (hunger > huntingThreshold) {
        Vec3f clusterCenter(0, 0, 0);
        if (preyCount >= 6) {
          clusterCenter = preyCenter / preyCount;
        }

        if (clusterCenter.mag() > 0.001f) {
          mode.huntingMode = true;
          target = clusterCenter;
          maxSpeed = baseSpeed * 3.0f;
          seek(target, 0.3);

          if (al::dist(bNav.pos(), target) < 3.0f) {
            hunger -= 0.3f;
            hunger = std::max(hunger, 0.0f);
            if (hunger < 0.2f) {
              mode.huntingMode = false;
              target.set(0);
            }
          }
        }
        else {
          mode.huntingMode = false;
          maxSpeed = baseSpeed * 0.8f;

          float wanderDistance = cubeSize * 0.95f;
          if (target.mag() < 0.001f ||
              al::dist(bNav.pos(), target) < wanderDistance) {
            target = newWanderTarget(cubeSize);
          }
          seek(target, 0.05);
        }
      }
      else {
        mode.huntingMode = false;
        target.set(0);
        maxSpeed = baseSpeed * 0.6f;

        float wanderDistance = cubeSize * 0.95f;
        if (target.mag() < 0.001f ||
            al::dist(bNav.pos(), target) < wanderDistance) {
          target = newWanderTarget(cubeSize);
        }
        seek(target, 0.03);
      }

      if (predatorCount > 0) {
        predatorAvoidance /= predatorCount;
        predatorAvoidance.normalize();
        bNav.faceToward(myPos + predatorAvoidance * visionRadius * 0.6f,
                        bNav.uu(), 0.6f * turnRateFactor);
      }
      return;
    }

    mode.beingHunted = hunted;

    if (predatorCount > 0) {
      mode.panicMode = true;
      hunger = 0.0f;
      predatorAvoidance /= predatorCount;
      predatorAvoidance.normalize();

      float panicLevel = 1.0f - (minPredatorDist / predatorAvoidRadius);
      panicLevel = panicLevel * panicLevel;

      if (mode.beingHunted) {
        panicLevel *= 1.5f;
        maxSpeed = baseSpeed * (2.0f + panicLevel * 1.0f);
      }
      else {
        maxSpeed = baseSpeed * (1.6f + panicLevel * 0.8f);
      }

      float escapeDistance = visionRadius * (mode.beingHunted ? 1.6f : 1.2f);
      float panicTurnRate = turnRateFactor * (mode.beingHunted ? 1.8f : 1.2f) *
                            (1.0f + panicLevel * 0.8f);
      bNav.faceToward(myPos + predatorAvoidance * escapeDistance, bNav.uu(),
                      panicTurnRate);
      return;
    }
    else {
      mode.panicMode = false;
      maxSpeed = baseSpeed;
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

    if (nearbyPredators > 0) {
      predatorSteering /= nearbyPredators;
      steering += predatorSteering * 0.6f;
    }

    if (steering.mag() > 0.001f) {
      steering.normalize();
      Vec3f newDirection = myVel + steering * turnRateFactor;
      newDirection.normalize();
      float effectiveTurnRate = turnRateFactor;
      if (mode.foragingMode) {
        effectiveTurnRate *= 2.5f;
      }
      this->bNav.faceToward(myPos + newDirection, this->bNav.uu(),
                            effectiveTurnRate);
    }
  }

  void seek(const Vec3d& a, const double amt, const float smooth = 0.1)
  {
    bNav.smooth(smooth);
    bNav.faceToward(a, bNav.uu(), amt);
  }

  void updatePosition(const double dt, double amt)
  {
    amt = std::min(amt * maxSpeed, 3.0);
    bNav.moveF(amt);
    bNav.step(dt);
  }

  const Vec3f& getTarget() const { return target; }
};
