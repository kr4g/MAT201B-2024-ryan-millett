// Ryan Millett
// MAT201B-2024
//
#include "al/app/al_App.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Functions.hpp"

// #include "../utils/octtree.cpp"
#include "classes/boid.cpp"

const int CUBE_SIZE = 13;

const int MAX_BOIDS = 6000;
// const float MAX_PREDATORS = MAX_BOIDS * 0.1;

const int N_PARTICLES = 500;

using namespace al;

double r() { return rnd::uniformS() * CUBE_SIZE * rnd::uniform(0.72, 0.833); }
Vec3f randomVec3f(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
}
struct Axes {
  void draw(Graphics &g) {
    Mesh mesh(Mesh::LINES);
    // x axis
    mesh.vertex(0, 0, 0);
    // mesh.color(1, 1, 1);  // white
    mesh.color(1, 0, 0);
    mesh.vertex(1, 0, 0);
    mesh.color(1, 0, 0);

    // y axis
    mesh.vertex(0, 0, 0);
    // mesh.color(1, 1, 1);  // white
    mesh.color(0, 1, 0);
    mesh.vertex(0, 1, 0);
    mesh.color(0, 1, 0);

    // z axis
    mesh.vertex(0, 0, 0);
    // mesh.color(1, 1, 1);  // white
    mesh.color(0, 0, 1);
    mesh.vertex(0, 0, 1);
    mesh.color(0, 0, 1);

    g.draw(mesh);
  }
};

string slurp(string fileName);  // forward declaration

struct MyApp : App {
  
  Parameter timeStep{"Time Step", "", 3.0, "", 0.008333, 5.0};
  Parameter pointSize{"/pointSize", "", 3.0, 0.1, 6.0};
  std::vector<Boid> boids{MAX_BOIDS};
  std::vector<Nav*> navPtrs;
  
  std::vector<Vec3f> food{N_PARTICLES};
  vector<Vec3f> velocity;
  vector<Vec3f> force;
  vector<float> mass;

  double time{0};
  double foodRefresh{0};
  double boidRespawn{0};
  double timeScale{1.0};
  double angle{0};

  double initDist;

  Axes axes;
  Mesh predMesh;
  Mesh preyMesh;
  // Mesh boidMesh;
  Mesh foodMesh;


  // Nav point;

  ShaderProgram pointShader;

  void onCreate() override {
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    setUp();

    // place the camera so that we can see the axes
    nav().pos(0.7, 0.7, CUBE_SIZE * 4.0);
    initDist = al::dist(nav().pos(), Vec3d(0, 0, 0));
    // nav().pos(0, 0, CUBE_SIZE * 2.5);
    nav().faceToward(Vec3d(0, 0, 0), Vec3d(0, 1, 0));

    // Don't do this:
    // nav().faceToward(0, 0, 0);
    // because it will be interpreted as this:
    // nav().faceToward(Vec3d(0), Vec3d(0), 0);
    // which has no effect because of the final 0!

    // create a prototype predator body
    predMesh.primitive(Mesh::TRIANGLE_FAN);
    predMesh.vertex(0, 0, -2);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.0, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(0, 1, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.0, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(-1, 0, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.0, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(1, 0, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.0, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(0, 1, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.0, 0.67), rnd::uniform(0.0, 0.33));

    // create a prototype prey body
    preyMesh.primitive(Mesh::TRIANGLE_FAN);
    preyMesh.vertex(0, 0, -2);
    preyMesh.color(0, rnd::uniform(0.5, 1.0), 0);
    preyMesh.vertex(0, 1, 0);
    preyMesh.color(0, rnd::uniform(0.5, 1.0), 0);
    preyMesh.vertex(-1, 0, 0);
    preyMesh.color(0, rnd::uniform(0.5, 1.0), 0);
    preyMesh.vertex(1, 0, 0);
    preyMesh.color(0, 0, rnd::uniform(0.5, 1.0));
    preyMesh.vertex(0, 1, 0);
    preyMesh.color(0, 0, rnd::uniform(0.5, 1.0));

    // foodMesh.primitive(Mesh::POINTS);
    // for (int i = 0; i < food.size(); i++) {
    //   food[i] = Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * CUBE_SIZE;
    //   foodMesh.vertex(food[i]);
    //   foodMesh.color(0.5, 0.25, 0);
    // }
    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };
    foodMesh.primitive(Mesh::POINTS);
    for (int _ = 0; _ < food.size(); _++) {
      foodMesh.vertex(randomVec3f(CUBE_SIZE));
      foodMesh.color(randomColor());

      // float m = rnd::uniform(3.0, 0.5);
      float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);

      // using a simplified volume/size relationship
      foodMesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      velocity.push_back(randomVec3f(2.5));
      force.push_back(randomVec3f(2.0));
    }
  }
  
  // void randomizeFoodList() {
  //   // randomize food positions
  //   // and update mesh vertices
  //   for (int i = 0; i < food.size(); i++) {
  //     randomizeFoodParticle(food[i]);
  //     foodMesh.vertices()[i] = food[i];
  //   }
  // }

  // void randomizeFoodParticle(Vec3f& f) {
  //   f.set(r(), r(), r());
  // }

  void setUp() {
      navPtrs.clear();
      for (auto& b : boids) {
          randomize(b.bNav);
          navPtrs.push_back(&b.bNav); // address of the nav
      }
  }
  
  void randomize(Nav& boidNav) {
    boidNav.pos(r(), r(), r());
    boidNav.quat().set(r(), r(), r(), r()).normalize();
  }
  
  bool freeze = false;
  double phase = 0;
  Vec3d target = Vec3d(r(), r(), r());
  void onAnimate(double dt) override {
    if (freeze) return;
    dt *= timeStep.get();
    time += dt;


    vector<Nav*> &position(navPtrs);

    Octree tree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.15f);
    tree.build(position);

    phase += dt;
    if (phase >= 30) {
        phase -= 30;
        target = Vec3d(r(), r(), r());
    }
    for (auto& b : boids) {
      // boid self-orientation algorithm
      b.seek(target, rnd::uniform(0.001, 0.008), rnd::uniform(0.05, 0.5));
      b.detectSurroundings(tree, CUBE_SIZE, position);
      b.updatePosition(rnd::uniform(0.667, 0.833), dt);
    }

    nav().smooth(0.9);
    nav().faceToward(Vec3d(0, 0, 0));
  }

  bool onKeyDown(Keyboard const& k) override {
    switch (k.key()) {
      case ' ':
        // reset the simulation
        setUp();
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

    {
      Mesh mesh(Mesh::LINES);
      // draw the axes
      mesh.vertex(-CUBE_SIZE, 0, 0);
      mesh.vertex(CUBE_SIZE, 0, 0);
      mesh.vertex(0, -CUBE_SIZE, 0);
      mesh.vertex(0, CUBE_SIZE, 0);
      mesh.vertex(0, 0, -CUBE_SIZE);
      mesh.vertex(0, 0, CUBE_SIZE);
      for (int i = 0; i < CUBE_SIZE; i++) mesh.color(1,1,1);

      g.draw(mesh);
    }

    // draw a body for each agent
    for (auto& b : boids) {
      {
        Nav& a(b.bNav);
        // randomize(a);
        g.pushMatrix();  // push()
        g.translate(a.pos());
        g.rotate(a.quat());  // rotate using the quat
        g.scale(
          // predators can be up to 1.5x larger than prey
          // (b.type == 0) ? 0.03 * rnd::uniform(0.83, 1.0) : 0.09 * rnd::uniform(0.5, 1.0)
          0.09
        );
        g.draw(
          // prey are blue/green, predators are red/orange
          // (b.type == 0) ? preyMesh : predMesh
          preyMesh
        );
        g.popMatrix();  // pop()
      }
    }

    axes.draw(g);
    
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(foodMesh);
  }

  void onInit() override {
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto& gui = GUIdomain->newGUI();
    // gui.add(t);
    // gui.add(pPredators);
    // gui.add(foodResetRate);
    // gui.add(boidRespawnRate);
    gui.add(timeStep);
    gui.add(pointSize);
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
