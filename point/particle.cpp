// Karl Yerkes
// 2022-01-20

#include "al/app/al_App.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"

#include "/Users/ryanmillett/MAT276IA/allolib_playground/MAT201B-2024-ryan-millett/point/octtree.cpp"

using namespace al;

#include <fstream>
#include <vector>
using namespace std;

float n_particles = 50000;//1670;

Vec3f randomVec3f(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
}
string slurp(string fileName);  // forward declaration

struct AlloApp : App {
  Parameter pointSize{"/pointSize", "", 3.0, 0.1, 6.0};
  Parameter timeStep{"/timeStep", "", 0.01, 0.0, 0.1};
  Parameter dragFactor{"/dragFactor", "", 0.667, 0.0, 5.0};
  Parameter sphereRadius{"sphereRadius", "", 15.0, 0.5, 30.0};
  Parameter k{"k", "", 0.0, 0.0, 5.0};
  Parameter q{"q", "", 0.0, 0.0, 1.0};
  //

  ShaderProgram pointShader;

  //  simulation state
  Mesh mesh;  // position *is inside the mesh* mesh.vertices() are the positions
  vector<Vec3f> velocity;
  vector<Vec3f> force;
  vector<float> mass;

  void onInit() override {
    // set up GUI
    auto GUIdomain = GUIDomain::enableGUI(defaultWindowDomain());
    auto &gui = GUIdomain->newGUI();
    gui.add(pointSize);  // add parameter to GUI
    gui.add(timeStep);
    gui.add(dragFactor);
    gui.add(sphereRadius);
    gui.add(k);
    gui.add(q);
  }

  void onCreate() override {
    // compile shaders
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    // set initial conditions of the simulation
    //

    // c++11 "lambda" function
    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };

    mesh.primitive(Mesh::POINTS);
    // does 1000 work on your system? how many can you make before you get a low
    // frame rate? do you need to use <1000?
    for (int _ = 0; _ < n_particles; _++) {
      mesh.vertex(randomVec3f(25));
      mesh.color(randomColor());

      // float m = rnd::uniform(3.0, 0.5);
      float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);

      // using a simplified volume/size relationship
      mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      velocity.push_back(randomVec3f(2.5));
      force.push_back(randomVec3f(20));
    }

    nav().pos(0, 0, 40);
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze) return;

    vector<Vec3f> &position(mesh.vertices());

    Octree tree(Vec3f(0, 0, 0), Vec3f(10, 10, 10), 1.0f);
    tree.build(position);

    for (int i = 0; i < position.size(); i++) {
      float currentDistance = position[i].mag();
      float displacement = currentDistance - sphereRadius;
      Vec3f springForce = Vec3f(-position[i]).normalize() * (k * displacement);
      force[i] += springForce;// + q * displacement; // spring force
      
      force[i] += - velocity[i] * dragFactor;     // drag force

      // "semi-implicit" Euler integration
      velocity[i] += force[i] / mass[i] * timeStep;
      position[i] += velocity[i] * timeStep;

      vector<int> nearbyParticles;
      tree.queryRegion(position[i], Vec3f(1, 1, 1), nearbyParticles); 

      // Repulsion :: [Coulombs law](https://en.wikipedia.org/wiki/Coulomb%27s_law)* :: $F = k_e \frac{q_1 q_2}{r^2}$
      float ke = 8.987551787e9; // Coulomb's constant in N·m²/C²
      HSV q1 = mesh.colors()[i];
      for (int j : nearbyParticles) {
        if ( i == j ) continue;
        HSV q2 = mesh.colors()[j];
        float charge = q1.h * q2.h;
        float u = 100.0 * abs(q1.h - q2.h);
        Vec3f r = position[j] - position[i];
        // if (r < 0.333) { charge *= 1.667; }
        Vec3f F = (Vec3f(r).normalize() * charge * q) / (r.magSqr() + 0.001);
        F = F * ke * u;
        force[i] -= F;// + rnd::uniformS() * (1.0 - u) * q * 0.001;
        force[j] += F;// + rnd::uniformS() * (1.0 - u) * q * 0.001;
      }
    }

    // clear all accelerations (IMPORTANT!!)
    for (auto &a : force) a.set(0);
    
    nav().smooth(0.9);
    nav().faceToward(Vec3d(0, 0, 0));
  }

  bool onKeyDown(const Keyboard &k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == '1') {
      // introduce some "random" forces
      for (int i = 0; i < velocity.size(); i++) {
        // F = ma
        force[i] += randomVec3f(60);
      }
    }

    return true;
  }

  void onDraw(Graphics &g) override {
    g.clear(0.0);
    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
    g.blending(true);
    g.blendTrans();
    g.depthTesting(true);
    g.draw(mesh);
  }
};

int main() {
  AlloApp app;
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
