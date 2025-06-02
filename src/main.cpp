// Ryan Millett
// MAT201B-2024

#include <fstream>

#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/graphics/al_VAOMesh.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "utils/octree.hpp"

constexpr int CUBE_SIZE = 40;
constexpr int MAX_BOIDS = 2000;
constexpr int NEIGHBOR_LIMIT = 100;
constexpr int N_FOOD_PARTICLES = 100;

using namespace al;

const double r() { return rnd::uniformS(); }
const Vec3f randomVec3f(const float scale = 1.0)
{
  return Vec3f(r(), r(), r()) * scale;
}

struct CommonState {
  float pointSize;
  Pose boid[MAX_BOIDS];
  BoidMode mode[MAX_BOIDS];
  Vec3f food[N_FOOD_PARTICLES];
  Pose pose;
};

struct MyApp : DistributedAppWithState<CommonState> {
  Parameter timeStep{"timeStep", "", 1.0, "", 0.0333, 3.0};
  Parameter pointSize{"pointSize", "", 0.1, 0.05, 6.0};
  Parameter bRadius{"boidVisionRadius", "", 5.0, 1.0, 8.0};
  Parameter predatorVision{"predatorVisionRadius", "", 12.0, 4.0, 15.0};
  Parameter cohesionForce{"cohesionForce", "", 1.3, 0.0, 3.0};
  Parameter separationForce{"separationForce", "", 1.7, 0.0, 3.0};
  Parameter alignmentForce{"alignmentForce", "", 1.3, 0.0, 3.0};
  Parameter turnRate{"turnRate", "", 0.54, 0.01, 0.5};

  std::vector<Boid> boids;
  std::vector<Vec3f> food;
  Octree* boidTree{nullptr};
  Octree* foodTree{nullptr};

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  // Axes axes;
  VAOMesh largeBoidMesh;
  VAOMesh smallBoidMesh;
  VAOMesh largeBoidPanicMesh;
  VAOMesh smallBoidPanicMesh;
  VAOMesh predatorMesh;
  VAOMesh predatorHuntMesh;
  VAOMesh foodMesh;
                   
  ShaderProgram pointShader;

  const std::string slurp(const std::string& fileName)
  {
    std::fstream file(fileName);
    std::string returnValue = "";
    while (file.good()) {
      std::string line;
      getline(file, line);
      returnValue += line + "\n";
    }
    return returnValue;
  }

  void onCreate() override
  {
    pointShader.compile(slurp("shaders/point-vertex.glsl"),
                        slurp("shaders/point-fragment.glsl"),
                        slurp("shaders/point-geometry.glsl"));

    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    lens().near(0.02).far(CUBE_SIZE * 4);
    nav().pos(Vec3f(5.0, 0.0, 0.0));
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    smallBoidMesh.primitive(Mesh::TRIANGLE_FAN);
    smallBoidMesh.vertex(0, 0, -5);
    smallBoidMesh.color(0.6, 1.0, 0.2);
    smallBoidMesh.vertex(0, 0.5, 0);
    smallBoidMesh.color(0.2, 0.7, 0.1);
    smallBoidMesh.vertex(-1, 0, 0);
    smallBoidMesh.color(0.3, 0.8, 0.2);
    smallBoidMesh.vertex(1, 0, 0);
    smallBoidMesh.color(0.3, 0.8, 0.2);
    smallBoidMesh.vertex(0, 0.5, 0);
    smallBoidMesh.color(0.2, 0.7, 0.1);
    smallBoidMesh.update();

    smallBoidPanicMesh.primitive(Mesh::TRIANGLE_FAN);
    smallBoidPanicMesh.vertex(0, 0, -5);
    smallBoidPanicMesh.color(0.8, 1.0, 0.4);
    smallBoidPanicMesh.vertex(0, 0.5, 0);
    smallBoidPanicMesh.color(0.4, 0.9, 0.2);
    smallBoidPanicMesh.vertex(-1, 0, 0);
    smallBoidPanicMesh.color(0.5, 1.0, 0.3);
    smallBoidPanicMesh.vertex(1, 0, 0);
    smallBoidPanicMesh.color(0.5, 1.0, 0.3);
    smallBoidPanicMesh.vertex(0, 0.5, 0);
    smallBoidPanicMesh.color(0.4, 0.9, 0.2);
    smallBoidPanicMesh.update();

    largeBoidMesh.primitive(Mesh::TRIANGLE_FAN);
    largeBoidMesh.vertex(0, 0, -3);
    largeBoidMesh.color(0, 0.5, 1.0);
    largeBoidMesh.vertex(0, 1, 0);
    largeBoidMesh.color(0.45, 0.17, 0.28);
    largeBoidMesh.vertex(-3, -1, 0);
    largeBoidMesh.color(0, 0.15, 0.7);
    largeBoidMesh.vertex(3, -1, 0);
    largeBoidMesh.color(0.08, 0.08, 0.60);
    largeBoidMesh.vertex(0, 1, 0);
    largeBoidMesh.color(0.45, 0.17, 0.28);
    largeBoidMesh.update();

    largeBoidPanicMesh.primitive(Mesh::TRIANGLE_FAN);
    largeBoidPanicMesh.vertex(0, 0, -3);
    largeBoidPanicMesh.color(0.2, 0.7, 1.0);
    largeBoidPanicMesh.vertex(0, 1, 0);
    largeBoidPanicMesh.color(0.6, 0.3, 0.4);
    largeBoidPanicMesh.vertex(-3, -1, 0);
    largeBoidPanicMesh.color(0.1, 0.3, 0.9);
    largeBoidPanicMesh.vertex(3, -1, 0);
    largeBoidPanicMesh.color(0.2, 0.2, 0.8);
    largeBoidPanicMesh.vertex(0, 1, 0);
    largeBoidPanicMesh.color(0.6, 0.3, 0.4);
    largeBoidPanicMesh.update();

    predatorMesh.primitive(Mesh::TRIANGLE_FAN);
    predatorMesh.vertex(0, 0, -4);
    predatorMesh.color(0.6, 0.1, 0.1);
    predatorMesh.vertex(-2, 1.5, 0);
    predatorMesh.color(0.5, 0.05, 0.05);
    predatorMesh.vertex(-4, -0.5, 0);
    predatorMesh.color(0.4, 0.03, 0.03);
    predatorMesh.vertex(0, -2, 0);
    predatorMesh.color(0.7, 0.15, 0.08);
    predatorMesh.vertex(4, -0.5, 0);
    predatorMesh.color(0.4, 0.03, 0.03);
    predatorMesh.vertex(2, 1.5, 0);
    predatorMesh.color(0.5, 0.05, 0.05);
    predatorMesh.vertex(0, 0, -4);
    predatorMesh.color(0.6, 0.1, 0.1);
    predatorMesh.update();

    predatorHuntMesh.primitive(Mesh::TRIANGLE_FAN);
    predatorHuntMesh.vertex(0, 0, -4);
    predatorHuntMesh.color(1.0, 0.1, 0.1);
    predatorHuntMesh.vertex(-2, 1.5, 0);
    predatorHuntMesh.color(0.8, 0.0, 0.0);
    predatorHuntMesh.vertex(-4, -0.5, 0);
    predatorHuntMesh.color(0.6, 0.0, 0.0);
    predatorHuntMesh.vertex(0, -2, 0);
    predatorHuntMesh.color(0.9, 0.2, 0.1);
    predatorHuntMesh.vertex(4, -0.5, 0);
    predatorHuntMesh.color(0.6, 0.0, 0.0);
    predatorHuntMesh.vertex(2, 1.5, 0);
    predatorHuntMesh.color(0.8, 0.0, 0.0);
    predatorHuntMesh.vertex(0, 0, -4);
    predatorHuntMesh.color(1.0, 0.1, 0.1);
    predatorHuntMesh.update();

    foodMesh.primitive(Mesh::POINTS);
    foodMesh.vertex(0, 0, 0);
    foodMesh.color(0.6, 0.4, 0.2);
    foodMesh.texCoord(0.5, 0);
    foodMesh.update();

    boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);
    foodTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);

    boids.clear();
    for (int i = 0; i < MAX_BOIDS; ++i) {
      BoidType type;
      float rand = rnd::uniform();
      if (rand < 0.002f) {
        type = BoidType::PREDATOR;
      }
      else if (rand < 0.55f) {
        type = BoidType::LARGE_PREY;
      }
      else {
        type = BoidType::SMALL_PREY;
      }

      Boid b(type);
      randomize(b.bNav);
      state().boid[i] = b.bNav.pos();
      state().mode[i] = b.mode;
      boids.push_back(b);
    }

    state().pointSize = 0.3f;

    for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
      Vec3f foodPos = randomVec3f(CUBE_SIZE * 0.9);
      food.push_back(foodPos);
      state().food[i] = foodPos;
    }

    int predatorCount = 0;
    for (const auto& b : boids) {
      if (b.mode.type == BoidType::PREDATOR) predatorCount++;
    }

    if (predatorCount < 3) {
      int needed = 3 - predatorCount;
      for (int i = 0; i < boids.size() && needed > 0; ++i) {
        if (boids[i].mode.type != BoidType::PREDATOR) {
          boids[i] = Boid(BoidType::PREDATOR);
          randomize(boids[i].bNav);
          state().boid[i] = boids[i].bNav.pos();
          needed--;
        }
      }
    }
  }

  void randomize(Nav& boidNav)
  {
    boidNav.pos(randomVec3f(CUBE_SIZE * 0.95));
    boidNav.quat().set(r(), r(), r(), r()).normalize();
  }

  bool freeze = false;
  void onAnimate(double dt) override
  {
    if (isPrimary()) {
      if (freeze) return;
      dt *= timeStep.get();
      time += dt;

      foodRefresh += dt;
      if (foodRefresh > 20.0) {
        for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
          if (rnd::uniform() < 0.3) {
            Vec3f newPos = randomVec3f(CUBE_SIZE * 0.9);
            food[i] = newPos;
            state().food[i] = newPos;
          }
        }
        foodRefresh = 0;
      }

      boidTree->build(boids);
      foodTree->build(food);

      // Vec3d boidCenterOfMass(0, 0, 0);
      int i = 0;
      for (auto& b : boids) {
        b.handleBoundary(CUBE_SIZE);
        b.originAvoidance(2.0f);

        if (b.mode.type == BoidType::PREDATOR) {
          boidTree->queryRegion(b.bNav.pos(), Vec3f(predatorVision.get()),
                                b.i_boids);
        }
        else {
          boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), b.i_boids);
        }

        float visionRadius = (b.mode.type == BoidType::PREDATOR)
                                 ? predatorVision.get()
                                 : bRadius.get();
        b.boidForces(boids, food, alignmentForce.get(), cohesionForce.get(),
                     separationForce.get(), CUBE_SIZE, visionRadius);

        b.updatePosition(dt, 0.67);
        state().boid[i].set(b.bNav);
        state().mode[i] = b.mode;
        i++;
      }
      // boidCenterOfMass /= boids.size();
      nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0), 0.2);
      state().pose = nav();
      state().pointSize = pointSize.get();
    }
    else {
      nav().set(state().pose);

      int i = 0;
      for (auto& b : boids) {
        b.bNav.set(state().boid[i]);
        b.mode = state().mode[i];
        i++;
      }
    }

    for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
      food[i] = state().food[i];
    }
  }

  void onDraw(Graphics& g) override
  {
    g.clear(0);
    g.meshColor();
    g.pointSize(10);

    for (auto& b : boids) {
      {
        const Nav& a(b.bNav);
        g.pushMatrix();
        g.translate(a.pos());
        g.rotate(a.quat());

        switch (b.mode.type) {
          case BoidType::SMALL_PREY:
            g.scale(0.08);
            g.draw(b.mode.panicMode ? smallBoidPanicMesh : smallBoidMesh);
            break;
          case BoidType::LARGE_PREY:
            g.scale(0.13);
            g.draw(b.mode.panicMode ? largeBoidPanicMesh : largeBoidMesh);
            break;
          case BoidType::PREDATOR:
            g.scale(0.21);
            g.draw(b.mode.huntingMode ? predatorHuntMesh : predatorMesh);
            break;
        }

        g.popMatrix();
      }

      Mesh m{Mesh::LINES};

      if (b.mode.type == BoidType::PREDATOR) {
        if (b.mode.huntingMode && b.target.mag() > 0.001f) {
          m.vertex(b.bNav.pos());
          m.color(0.8, 0.2, 0.2);
          m.vertex(b.target);
          m.color(0.5, 0.05, 0.05);
        }
      }
      else {
        if (b.mode.foragingMode && b.target.mag() > 0.001f) {
          m.vertex(b.bNav.pos());
          m.vertex(b.target);

          if (b.mode.type == BoidType::SMALL_PREY) {
            m.color(0.15, 0.4, 0.05);
            m.color(0.15, 0.4, 0.05);
          }
          else if (b.mode.type == BoidType::LARGE_PREY) {
            m.color(0.0, 0.2, 0.5);
            m.color(0.0, 0.2, 0.5);
          }
        }
      }

      if (m.vertices().size() > 0) {
        g.draw(m);
      }
    }

    g.shader(pointShader);
    g.shader().uniform("pointSize", state().pointSize);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    for (unsigned int i = 0; i < N_FOOD_PARTICLES; ++i) {
      g.pushMatrix();
      g.translate(food[i]);
      g.draw(foodMesh);
      g.popMatrix();
    }
    g.shader().end();
    g.blending(false);
  }

  void onInit() override
  {
    auto cuttleboneDomain =
        CuttleboneStateSimulationDomain<CommonState>::enableCuttlebone(this);

    if (!cuttleboneDomain) {
      std::cerr << "ERROR: Could not start Cuttlebone. Quitting." << std::endl;
      quit();
    }

    if (isPrimary()) {
      auto guiDomain = GUIDomain::enableGUI(defaultWindowDomain());
      auto& gui = guiDomain->newGUI();
      gui.add(timeStep);
      gui.add(pointSize);
      gui.add(cohesionForce);
      gui.add(separationForce);
      gui.add(alignmentForce);
      gui.add(bRadius);
      gui.add(predatorVision);
    }
  }
};

int main()
{
  MyApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}
