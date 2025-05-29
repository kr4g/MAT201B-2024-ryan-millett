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

constexpr int CUBE_SIZE = 20;
constexpr int MAX_BOIDS = 5000;
constexpr int NEIGHBOR_LIMIT = 100;
// constexpr float MAX_BOID_RADIUS = CUBE_SIZE * 0.1;

constexpr int N_PARTICLES = 1500;

using namespace al;

double r() { return rnd::uniformS(); }
Vec3f randomVec3f(float scale = 1.0) { return Vec3f(r(), r(), r()) * scale; }

struct Axes {
  void draw(Graphics& g)
  {
    Mesh mesh(Mesh::LINES);
    // x axis
    mesh.vertex(-CUBE_SIZE, 0, 0);
    mesh.color(1, 0, 0);
    mesh.vertex(CUBE_SIZE, 0, 0);
    mesh.color(1, 0, 0);

    // y axis
    mesh.vertex(0, -CUBE_SIZE, 0);
    mesh.color(0, 1, 0);
    mesh.vertex(0, CUBE_SIZE, 0);
    mesh.color(0, 1, 0);

    // z axis
    mesh.vertex(0, 0, -CUBE_SIZE);
    mesh.color(0, 0, 1);
    mesh.vertex(0, 0, CUBE_SIZE);
    mesh.color(0, 0, 1);

    g.draw(mesh);
  }
};

struct CommonState {
  // particles
  float pointSize;
  // boids
  Pose boid[MAX_BOIDS];

  int i_boids[MAX_BOIDS][NEIGHBOR_LIMIT];
  // XXX - boid vertex colors???
  Pose pose;
  Vec3f boidCenterMass;
};

struct MyApp : DistributedAppWithState<CommonState> {
  Parameter timeStep{"Time Step", "", 0.75, "", 0.0333, 3.0};
  Parameter pointSize{"/pointSize", "", 0.5, 0.05, 6.0};
  Parameter bRadius{"/Boid Vision Radius", "", 0.45, 0.05, 1.5};
  // Parameter cohesionThresh{"/Cohesion Threshold", "", 0.96, 0.0001,
  // MAX_BOID_RADIUS};
  Parameter cohesionForce{"/Cohesion Force", "", 0.35, 0.0, 1.0};
  // Parameter separationThresh{"/Separation Threshold", "", 0.75, 0.0001,
  // MAX_BOID_RADIUS};
  Parameter separationForce{"Separation Force", "", 0.75, 0.0, 1.0};
  // Parameter alignmentThresh{"Alignment Threshold", "", 1.1, 0.0001,
  // MAX_BOID_RADIUS};
  Parameter alignmentForce{"Alignment Force", "", 0.25, 0.0, 1.0};
  Parameter turnRate{"Turn Rate", "", 0.025, 0.0001, 1.0};

  std::vector<Boid> boids;
  std::vector<Vec3f> food;
  std::vector<Vec3f> velocity;
  std::vector<Vec3f> force;
  std::vector<float> mass;
  Octree* boidTree{nullptr};

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  Axes axes;
  // VAOMesh predMesh;
  VAOMesh preyMeshMale;
  VAOMesh preyMeshFemale;
  // VAOMesh boidMesh;
  VAOMesh foodMesh;
  // VAOMesh lineMesh{Mesh::LINES};

  // Nav point;

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

    // place the camera so that we can see the axes
    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    // nav().pos(CUBE_SIZE, CUBE_SIZE * 0.5, CUBE_SIZE * 1.5);
    // nav().pos(Vec3f(0.0));
    nav().pos(Vec3f(5.0, 0.0, 0.0));
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    // Don't do this:
    // nav().faceToward(0, 0, 0);
    // because it will be interpreted as this:
    // nav().faceToward(Vec3d(0), Vec3d(0), 0);
    // which has no effect because of the final 0!

    // Male Prey Body
    preyMeshMale.primitive(Mesh::TRIANGLE_FAN);
    preyMeshMale.vertex(0, 0, -3);  // Nose
    preyMeshMale.color(0, 0.5, 1.0);
    preyMeshMale.vertex(0, 1, 0);  // Top center edge ("back")
    preyMeshMale.color(0.45, 0.17, 0.28);
    preyMeshMale.vertex(-3, -1, 0);  // Left edge
    preyMeshMale.color(0, 0.15, 0.7);
    preyMeshMale.vertex(3, -1, 0);  // Right edge
    preyMeshMale.color(0.08, 0.08, 0.60);
    preyMeshMale.vertex(0, 1, 0);  // Top center edge, closing the fan
    preyMeshMale.color(0.45, 0.17, 0.28);

    preyMeshMale.update();

    // Female Prey Body
    preyMeshFemale.primitive(Mesh::TRIANGLE_FAN);
    preyMeshFemale.vertex(0, 0, -5);  // Nose
    preyMeshFemale.color(0.6, 1.0, 0.2);
    preyMeshFemale.vertex(0, 0.5, 0);  // Top center edge ("back")
    preyMeshFemale.color(0.2, 0.7, 0.1);
    preyMeshFemale.vertex(-1, 0, 0);  // Left edge
    preyMeshFemale.color(0.3, 0.8, 0.2);
    preyMeshFemale.vertex(1, 0, 0);  // Right edge
    preyMeshFemale.color(0.3, 0.8, 0.2);
    preyMeshFemale.vertex(0, 0.5, 0);  // Top center edge, closing the fan
    preyMeshFemale.color(0.2, 0.7, 0.1);

    preyMeshFemale.update();

    // setUp(); // init boids

    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };
    foodMesh.primitive(Mesh::POINTS);

    boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);

    boids.clear();
    for (int i = 0; i < MAX_BOIDS; ++i) {
      Boid b;
      randomize(b.bNav);
      state().boid[i] = b.bNav.pos();
      boids.push_back(b);
    }
    for (int i = 0; i < N_PARTICLES; ++i) {
    }

    if (isPrimary()) {
    }

    for (int i = 0; i < N_PARTICLES; ++i) {
      foodMesh.vertex(randomVec3f(CUBE_SIZE));
      foodMesh.color(randomColor());
      float m = rnd::uniform(8.0, 0.5);
      // float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);
      // using a simplified volume/size relationship
      foodMesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t
    }

    foodMesh.update();
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
    // boidNav.faceToward(randomVec3f(CUBE_SIZE), 0.5);
  }

  bool freeze = false;
  void onAnimate(double dt) override
  {
    // std::cout << nav().pos() << std::endl;
    if (isPrimary()) {
      if (freeze) return;
      dt *= timeStep.get();
      time += dt;
      boidTree->build(boids);

      Vec3d boidCenterOfMass(0, 0, 0);
      int i = 0;
      for (auto& b : boids) {
        boidCenterOfMass += b.bNav.pos();
        // b.originAvoidance(0.5, 2.0);
        b.handleBoundary(CUBE_SIZE);

        boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), b.i_boids);

        b.boidForces(boids, alignmentForce.get(), cohesionForce.get(),
                     separationForce.get(), turnRate.get());
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

      // cout << boids[0].i_boids.size() << endl;
    }
  }

  bool onKeyDown(Keyboard const& k) override
  {
    switch (k.key()) {
      case ' ':
        // reset the simulation
        // setUp();
        break;
    }
    return true;
  }

  void onDraw(Graphics& g) override
  {
    // graphics / drawing settings
    g.clear(0);
    g.meshColor();
    g.pointSize(10);
    // axes.draw(g);

    int i = 0;
    for (auto& b : boids) {
      {
        Nav& a(b.bNav);
        g.pushMatrix();
        g.translate(a.pos());
        g.rotate(a.quat());
        g.scale(
            // (i % 11 != 0) ? 0.01 : 0.005
            (i % 11 != 0) ? 0.03 : 0.02);
        g.draw((i % 11 != 0) ? preyMeshMale : preyMeshFemale);
        g.popMatrix();
      }
      Mesh m{Mesh::LINES};
      for (int j : b.i_boids) {
        if (i < j) {
          m.vertex(b.bNav.pos());
          m.vertex(boids[j].bNav.pos());
          m.color(1.0, 1.0, 1.0);
        }
      }
      if (m.vertices().size() > 0) {
        g.draw(m);
      }
      ++i;
    }

    g.shader(pointShader);
    g.shader().uniform("pointSize", state().pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(foodMesh);
    // g.draw(lineMesh);
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
      // gui.add(cohesionThresh);
      gui.add(cohesionForce);
      // gui.add(separationThresh);
      gui.add(separationForce);
      // gui.add(alignmentThresh);
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
