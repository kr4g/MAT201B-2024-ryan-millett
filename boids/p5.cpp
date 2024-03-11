// Ryan Millett
// MAT201B-2024

#include "al/app/al_App.hpp"
#include "al/app/al_DistributedApp.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Functions.hpp"

#include "al_ext/statedistribution/al_CuttleboneDomain.hpp"
#include "al_ext/statedistribution/al_CuttleboneStateSimulationDomain.hpp"

#include "../utils/octree.cpp"
// #include "classes/boid_3.cpp"

const int CUBE_SIZE = 10;

const int MAX_BOIDS = 4000;
const float MAX_BOID_RADIUS = CUBE_SIZE * 0.1;

const int N_PARTICLES = 1500;

using namespace al;

double r() { return rnd::uniformS() * CUBE_SIZE; }
Vec3f randomVec3f(float scale = CUBE_SIZE) {
  return Vec3f(r(), r(), r());
}

struct Axes {
  void draw(Graphics &g) {
    Mesh mesh(Mesh::LINES);
    // x axis
    mesh.vertex(-CUBE_SIZE, 0, 0);
    // mesh.color(1, 1, 1);  // white
    mesh.color(1, 0, 0);
    mesh.vertex(CUBE_SIZE, 0, 0);
    mesh.color(1, 0, 0);

    // y axis
    mesh.vertex(0, -CUBE_SIZE, 0);
    // mesh.color(1, 1, 1);  // white
    mesh.color(0, 1, 0);
    mesh.vertex(0, CUBE_SIZE, 0);
    mesh.color(0, 1, 0);

    // z axis
    mesh.vertex(0, 0, -CUBE_SIZE);
    // mesh.color(1, 1, 1);  // white
    mesh.color(0, 0, 1);
    mesh.vertex(0, 0, CUBE_SIZE);
    mesh.color(0, 0, 1);

    g.draw(mesh);
  }
};

Vec3f calculateCenterOfMass(const std::vector<Vec3f>& positions) {
    Vec3f center(0, 0, 0);
    for (auto& pos : positions) {
        center += pos;
    }
    center /= positions.size();
    return center;
}

string slurp(string fileName);  // forward declaration

struct CommonState {
  // particles
  float pointSize;
  Vec3f particlePositions[N_PARTICLES];
  HSV particleColors[N_PARTICLES];
  // boids
  Vec3f boidPositions[MAX_BOIDS];
  HSV boidColors[MAX_BOIDS];
};

struct MyApp : DistributedAppWithState<CommonState> {
  
  Parameter timeStep{"Time Step", "", 1.0, "", 0.0333, 5.0};
  Parameter pointSize{"/pointSize", "", 0.5, 0.05, 6.0};
  Parameter bRadius{"/Boid Vision Radius", "", 0.25, 0.125, MAX_BOID_RADIUS};
  // Parameter cohesionThresh{"/Cohesion Threshold", "", 0.96, 0.0001, MAX_BOID_RADIUS};
  Parameter cohesionForce{"/Cohesion Force", "", 0.001, 0.0001, 1.0};
  // Parameter separationThresh{"/Separation Threshold", "", 0.75, 0.0001, MAX_BOID_RADIUS};
  Parameter separationForce{"Separation Force", "", 0.001, 0.0001, 1.0};
  // Parameter alignmentThresh{"Alignment Threshold", "", 1.1, 0.0001, MAX_BOID_RADIUS};
  Parameter alignmentForce{"Alignment Force", "", 0.833, 0.0001, 1.0};
  
  std::vector<Boid> boids;    
  std::vector<Vec3f> food;
  vector<Vec3f> velocity;
  vector<Vec3f> force;
  vector<float> mass;
  // Octree foodTree;
  Octree* boidTree{nullptr};
  // std::vector<int>* i_boids;

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  Axes axes;
  // Mesh predMesh;
  // Mesh preyMeshMale;
  // Mesh preyMeshFemale;
  Mesh boidMesh;
  Mesh foodMesh;
  // Mesh lineMesh{Mesh::LINES};

  // Nav point;

  ShaderProgram pointShader;

  void onCreate() override {
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    boidSetUp();

    for (int i = 0; i < MAX_BOIDS; ++i) {
      int Nv = rnd::prob(0.5) ? (rnd::prob(0.5) ? addCube(boidMesh)
                                                : addDodecahedron(boidMesh))
                              : addIcosahedron(boidMesh);

      boids[i].Nv = Nv;
      boids[i].meshIdx = boidMesh.vertices().size() - Nv;
      // Scale and translate the newly added shape
      Mat4f xfm;
      xfm.setIdentity();
      xfm.scale(Vec3f(0.05));
      xfm.translate(Vec3f(boids[i].bNav.pos()));
      boidMesh.transform(xfm, boidMesh.vertices().size() - boids[i].Nv);

      // Color newly added vertices
      for (int i = 0; i < Nv; ++i) {
        float f = float(i) / Nv;
        HSV color = HSV(f * 0.1 + 0.2, 1, 1);
        state().boidColors[i] = color;
        boidMesh.color(color);
      }
    }

    // Convert to non-indexed triangles to get flat shading
    boidMesh.decompress();
    boidMesh.generateNormals();

    // place the camera so that we can see the axes
    // nav().pos(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE * 1.167);
    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    nav().pos(CUBE_SIZE, CUBE_SIZE * 0.5, CUBE_SIZE * 1.5);
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    // Don't do this:
    // nav().faceToward(0, 0, 0);
    // because it will be interpreted as this:
    // nav().faceToward(Vec3d(0), Vec3d(0), 0);
    // which has no effect because of the final 0!

    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };
    foodMesh.primitive(Mesh::POINTS);

    if (isPrimary()) {
      for (int i = 0; i < N_PARTICLES; ++i) {
        state().particlePositions[i] = randomVec3f(CUBE_SIZE);
        state().particleColors[i] = randomColor();
      }
    }

    for (int i = 0; i < N_PARTICLES; ++i) {
      foodMesh.vertex(state().particlePositions[i]);
      foodMesh.color(state().particleColors[i]);

      float m = rnd::uniform(8.0, 0.5);
      // float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);
      // using a simplified volume/size relationship
      foodMesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t
      // velocity.push_back(randomVec3f(-0.025));
      // force.push_back(randomVec3f(-0.000001));
    }
  }  
  
  Vec3d target = Vec3d(r(), r(), r());
  void boidSetUp() {         
      boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE), 0.01f);
      boids.clear();
      for (int i = 0; i < MAX_BOIDS; i++) {        
        Boid b;
        randomize(b.bNav);
        // b.seek(target, rnd::uniform(0.005, 0.09), rnd::uniform(0.05, 0.75));
        state().boidPositions[i] = b.bNav.pos();
        boids.push_back(b);
      }
  }
  
  void randomize(Nav& boidNav) {
    boidNav.pos(randomVec3f(CUBE_SIZE*0.75));
    boidNav.quat().set(r(), r(), r(), r()).normalize();
  }
  
  bool freeze = false;
  double phase = 0;
  double trackingReset = 0.0;
  void onAnimate(double dt) override {
    if (freeze) return;
    dt *= timeStep.get();
    time += dt;
    boidTree->build(boids);
    Vec3d boidCenterOfMass(0, 0, 0);
    int i = 0;
    for (auto& b : boids) {
      boidCenterOfMass += b.bNav.pos();      
      b.handleBoundary(CUBE_SIZE);
      boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), b.i_boids);
      b.boidForces(boids, alignmentForce.get(), cohesionForce.get(), separationForce.get());
      b.updatePosition(dt);
      state().boidPositions[i] = b.bNav.pos();
      
      // XXX - this is wrong
      Mat4f xfm;
      xfm.setIdentity();
      xfm.translate(Vec3f(b.bNav.pos()));
      boidMesh.transform(xfm, b.meshIdx, b.meshIdx + b.Nv);
      
      ++i;
    }
    boidCenterOfMass /= boids.size();
    nav().faceToward(boidCenterOfMass, Vec3d(0, 1, 0), 0.2);
  }

  bool onKeyDown(Keyboard const& k) override {
    switch (k.key()) {
      case ' ':
        // reset the simulation
        boidSetUp();
        break;
    }
    return true;
  }

  void onDraw(Graphics& g) override {
    // graphics / drawing settings
    g.clear(0);
    g.meshColor();
    g.pointSize(10);    
    // g.rotate(angle, Vec3d(0, 1, 0));
    // axes.draw(g);
    int i = 0;
    for (auto& b : boids) {
      // {
      //   Nav& a(b.bNav);
      //   g.pushMatrix();
      //   g.translate(a.pos());
      //   g.rotate(a.quat());
      //   g.scale(
      //     // (i % 11 != 0) ? 0.025 : 0.01
      //     (i % 11 != 0) ? 0.01 : 0.005
      //   );
      //   g.draw(
      //     (i % 11 != 0) ? preyMeshMale : preyMeshFemale
      //   );
      //   g.popMatrix();
      // }
      Mesh m{Mesh::LINES};
      for (int j : b.i_boids) {
        if (i == j) continue;
        m.vertex(b.bNav.pos());
        m.vertex(boids[j].bNav.pos());
        m.color(1.0, 1.0, 1.0);
        g.draw(m);
      }
      ++i;
    }

    g.draw(boidMesh);
    g.shader(pointShader);
    g.shader().uniform("pointSize", state().pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(foodMesh);
    // g.draw(lineMesh);
  }

  void onInit() override {
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto& gui = GUIdomain->newGUI();
    gui.add(timeStep);
    gui.add(pointSize);
    // gui.add(cohesionThresh);
    gui.add(cohesionForce);
    // gui.add(separationThresh);
    gui.add(separationForce);
    // gui.add(alignmentThresh);
    gui.add(alignmentForce);
    gui.add(bRadius);
  }
};

int main() {
  MyApp app;
  app.configureAudio(48000, 512, 2, 0);
  app.start();
}

string slurp(string fileName) {
  fstream file(fileName);
  string returnValue = "";
  while (file.good()) {
    string line;
    getline(file, line);
    returnValue += line + "\n";
  }
  return returnValue;
}
