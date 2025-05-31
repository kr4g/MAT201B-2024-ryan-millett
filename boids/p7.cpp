// Ryan Millett
// MAT201B-2024

#include <fstream>

#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/graphics/al_VAOMesh.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"
#include "octree.hpp"

constexpr int CUBE_SIZE = 40;
constexpr int MAX_BOIDS = 1500;
constexpr int NEIGHBOR_LIMIT = 100;

constexpr int N_FOOD_PARTICLES = 25;

using namespace al;

double r() { return rnd::uniformS(); }
Vec3f randomVec3f(float scale = 1.0) { return Vec3f(r(), r(), r()) * scale; }

// struct Axes {
//   void draw(Graphics& g)
//   {
//     Mesh mesh(Mesh::LINES);
//     // x axis
//     mesh.vertex(-CUBE_SIZE, 0, 0);
//     mesh.color(1, 0, 0);
//     mesh.vertex(CUBE_SIZE, 0, 0);
//     mesh.color(1, 0, 0);

//     // y axis
//     mesh.vertex(0, -CUBE_SIZE, 0);
//     mesh.color(0, 1, 0);
//     mesh.vertex(0, CUBE_SIZE, 0);
//     mesh.color(0, 1, 0);

//     // z axis
//     mesh.vertex(0, 0, -CUBE_SIZE);
//     mesh.color(0, 0, 1);
//     mesh.vertex(0, 0, CUBE_SIZE);
//     mesh.color(0, 0, 1);

//     g.draw(mesh);
//   }
// };

struct CommonState {
  float pointSize;
  Pose boid[MAX_BOIDS];
  int i_boids[MAX_BOIDS][NEIGHBOR_LIMIT];
  Vec3f food[N_FOOD_PARTICLES];
  Pose pose;
  Vec3f boidCenterMass;
};

struct MyApp : DistributedAppWithState<CommonState> {
  Parameter timeStep{"Time Step", "", 2.0, "", 0.0333, 3.0};
  Parameter pointSize{"/pointSize", "", 0.5, 0.05, 6.0};
  Parameter bRadius{"/Boid Vision Radius", "", 2.0, 0.05, 5.0};
  Parameter cohesionForce{"/Cohesion Force", "", 0.15, 0.0, 1.0};
  Parameter separationForce{"Separation Force", "", 0.45, 0.0, 1.0};
  Parameter alignmentForce{"Alignment Force", "", 0.5, 0.0, 1.0};
  Parameter turnRate{"Turn Rate", "", 0.33, 0.0001, 1.0};

  std::vector<Boid> boids;
  std::vector<Vec3f> food;
  std::vector<Vec3f> velocity;
  std::vector<Vec3f> force;
  std::vector<float> mass;
  Octree* boidTree{nullptr};
  Octree* foodTree{nullptr};

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  // Axes axes;
  VAOMesh preyMeshMale;
  VAOMesh preyMeshFemale;
  VAOMesh foodMesh;

  ShaderProgram pointShader;

  std::string slurp(std::string fileName)
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
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    nav().pos(Vec3f(5.0, 0.0, 0.0));
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    preyMeshMale.primitive(Mesh::TRIANGLE_FAN);
    preyMeshMale.vertex(0, 0, -3);
    preyMeshMale.color(0, 0.5, 1.0);
    preyMeshMale.vertex(0, 1, 0);
    preyMeshMale.color(0.45, 0.17, 0.28);
    preyMeshMale.vertex(-3, -1, 0);
    preyMeshMale.color(0, 0.15, 0.7);
    preyMeshMale.vertex(3, -1, 0);
    preyMeshMale.color(0.08, 0.08, 0.60);
    preyMeshMale.vertex(0, 1, 0);
    preyMeshMale.color(0.45, 0.17, 0.28);
    preyMeshMale.update();

    preyMeshFemale.primitive(Mesh::TRIANGLE_FAN);
    preyMeshFemale.vertex(0, 0, -5);
    preyMeshFemale.color(0.6, 1.0, 0.2);
    preyMeshFemale.vertex(0, 0.5, 0);
    preyMeshFemale.color(0.2, 0.7, 0.1);
    preyMeshFemale.vertex(-1, 0, 0);
    preyMeshFemale.color(0.3, 0.8, 0.2);
    preyMeshFemale.vertex(1, 0, 0);
    preyMeshFemale.color(0.3, 0.8, 0.2);
    preyMeshFemale.vertex(0, 0.5, 0);
    preyMeshFemale.color(0.2, 0.7, 0.1);
    preyMeshFemale.update();

    foodMesh.primitive(Mesh::POINTS);
    for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
      foodMesh.vertex(0, 0, 0);
      foodMesh.color(1.0, 0.2, 0.2);
      foodMesh.texCoord(0.5, 0);
    }
    foodMesh.update();

    boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);
    foodTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);

    boids.clear();
    for (int i = 0; i < MAX_BOIDS; ++i) {
      Boid b;
      randomize(b.bNav);
      state().boid[i] = b.bNav.pos();
      boids.push_back(b);
    }

    for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
      Vec3f foodPos = randomVec3f(CUBE_SIZE * 0.9);
      food.push_back(foodPos);
      state().food[i] = foodPos;
    }

    if (isPrimary()) {
    }
  }

  void setUp()
  {
    boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);
    boids.clear();
    for (int i = 0; i < MAX_BOIDS; ++i) {
      Boid b;
      randomize(b.bNav);
      state().boid[i] = b.bNav.pos();
      boids.push_back(b);
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
      if (foodRefresh > 15.0) {
        for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
          if (rnd::uniform() < 0.5) {
            Vec3f newPos = randomVec3f(CUBE_SIZE * 0.9);
            food[i] = newPos;
            state().food[i] = newPos;
          }
        }
        foodRefresh = 0;
      }

      boidTree->build(boids);
      foodTree->build(food);

      Vec3d boidCenterOfMass(0, 0, 0);
      int i = 0;
      for (auto& b : boids) {
        boidCenterOfMass += b.bNav.pos();
        b.handleBoundary(CUBE_SIZE);

        boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), b.i_boids);

        b.boidForces(boids, alignmentForce.get(), cohesionForce.get(), 
                     separationForce.get(), turnRate.get());
        
        if (!food.empty()) {
          std::vector<int> nearbyFood;
          foodTree->queryRegion(b.bNav.pos(), Vec3f(5.0f), nearbyFood);
          
          if (!nearbyFood.empty()) {
            Vec3f closestFood = food[nearbyFood[0]];
            float minDist = al::dist(b.bNav.pos(), food[nearbyFood[0]]);
            int closestIdx = nearbyFood[0];
            
            for (int j : nearbyFood) {
              float d = al::dist(b.bNav.pos(), food[j]);
              if (d < minDist) {
                minDist = d;
                closestFood = food[j];
                closestIdx = j;
              }
            }
            
            if (minDist < bRadius.get()) {
              Vec3f newPos = randomVec3f(CUBE_SIZE * 0.9);
              food[closestIdx] = newPos;
              state().food[closestIdx] = newPos;
            } else {
              b.seek(closestFood, 0.2);
            }
          }
        }

        b.updatePosition(dt);
        state().boid[i].set(b.bNav);
        i++;
      }
      boidCenterOfMass /= boids.size();
      nav().faceToward(boidCenterOfMass, Vec3d(0, 1, 0), 0.2);
      state().pose = nav();

      for (int i = 0; i < boids.size(); i++) {
        for (int j = 0; j < NEIGHBOR_LIMIT; j++) {
          state().i_boids[i][j] = -1;
        }
        for (int j = 0; j < boids[i].i_boids.size() && j < NEIGHBOR_LIMIT;
             j++) {
          state().i_boids[i][j] = boids[i].i_boids[j];
        }
      }
    }
    else {
      nav().set(state().pose);

      int i = 0;
      for (auto& b : boids) {
        b.bNav.set(state().boid[i]);
        i++;
      }

      for (int i = 0; i < boids.size(); i++) {
        boids[i].i_boids.clear();
        for (int j = 0; j < NEIGHBOR_LIMIT; j++) {
          int n = state().i_boids[i][j];
          if (n == -1) break;
          boids[i].i_boids.push_back(n);
        }
      }
    }
    
    // for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
    // }
    
    foodMesh.reset();
    foodMesh.primitive(Mesh::POINTS);
    for (int i = 0; i < N_FOOD_PARTICLES; ++i) {
      food[i] = state().food[i];
      foodMesh.vertex(state().food[i]);
      foodMesh.color(1.0, 0.2, 0.2);
      foodMesh.texCoord(0.5, 0.1);
    }
    foodMesh.update();
  }

  bool onKeyDown(Keyboard const& k) override
  {
    switch (k.key()) {
      case ' ':
        break;
    }
    return true;
  }

  void onDraw(Graphics& g) override
  {
    g.clear(0);
    g.meshColor();
    g.pointSize(10);

    int i = 0;
    for (auto& b : boids) {
      {
        Nav& a(b.bNav);
        g.pushMatrix();
        g.translate(a.pos());
        g.rotate(a.quat());
        g.scale(
            (i % 11 != 0) ? 0.08 : 0.05);
        g.draw((i % 11 != 0) ? preyMeshMale : preyMeshFemale);
        g.popMatrix();
      }
      Mesh m{Mesh::LINES};
      for (int j : b.i_boids) {
        if (i < j) {
          m.vertex(b.bNav.pos());
          m.color(0.1, 0.1, 0.1);
          m.vertex(boids[j].bNav.pos());
          m.color(0.1, 0.1, 0.1);
        }
      }
      if (m.vertices().size() > 0) {
        g.draw(m);
      }
      ++i;
    }

    g.shader(pointShader);
    g.shader().uniform("pointSize", 3.0f);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(foodMesh);
    
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
      gui.add(turnRate);
      gui.add(bRadius);
    }
  }
};

int main()
{
  MyApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}
