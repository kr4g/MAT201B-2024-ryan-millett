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

float n_particles = 15000;//1670;

Vec3f randomVec3f(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
}
string slurp(string fileName);  // forward declaration

struct AlloApp : App {
  Parameter pointSize{"/pointSize", "", 1.0, 0.0, 4.0};
  Parameter timeStep{"/timeStep", "", 0.1, 0.01, 0.6};
  Parameter dragFactor{"/dragFactor", "", 0.1, 0.0, 0.9};
  Parameter sphereRadius{"sphereRadius", "", 5.0, 0.0, 10.0};
  Parameter k{"k", "", 0.0, 0.0, 5.0};
  Parameter q{"q", "", 0.0, 0.0, 0.1};
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
    //
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
      mesh.vertex(randomVec3f(5));
      mesh.color(randomColor());

      // float m = rnd::uniform(3.0, 0.5);
      float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);

      // using a simplified volume/size relationship
      mesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      velocity.push_back(randomVec3f(0.1));
      force.push_back(randomVec3f(1));
    }

    nav().pos(0, 0, 10);
  }

  bool freeze = false;
  void onAnimate(double dt) override {
    if (freeze) return;

    // Calculate forces

    // XXX you put code here that calculates gravitational forces and sets
    // accelerations These are pair-wise. Each unique pairing of two particles
    // These are equal but opposite: A exerts a force on B while B exerts that
    // same amount of force on A (but in the opposite direction!) Use a nested
    // for loop to visit each pair once The time complexity is O(n*n)
    //
    // Vec3f has lots of operations you might use...
    // • +=
    // • -=
    // • +
    // • -
    // • .normalize() ~ Vec3f points in the direction as it did, but has length 1
    // • .normalize(float scale) ~ same but length `scale`
    // • .mag() ~ length of the Vec3f
    // • .magSqr() ~ squared length of the Vec3f
    // • .dot(Vec3f f) 
    // • .cross(Vec3f f)

    // float sphereRadius = 5.0; // example radius
    // float k = 0.01; // spring constant, adjust as needed

    vector<Vec3f> &position(mesh.vertices());

    Octree tree(Vec3f(0, 0, 0), Vec3f(10, 10, 10), 1.0f);
    tree.build(position);

    for (int i = 0; i < position.size(); i++) {
      HSV q1 = mesh.colors()[i];
      float currentDistance = position[i].mag();
      float displacement = currentDistance - sphereRadius;
      Vec3f springForce = Vec3f(-position[i]).normalize() * (k * displacement);
      force[i] += springForce + q * displacement;
      

      vector<int> nearbyParticles;
      tree.queryRegion(position[i], Vec3f(10, 10, 10), nearbyParticles); 


      // Repulsion :: [Coulombs law](https://en.wikipedia.org/wiki/Coulomb%27s_law)* :: $F = k_e \frac{q_1 q_2}{r^2}$
      float ke = 8.987551787e9; // Coulomb's constant in N·m²/C²
      // for (int i = 0; i < position.size(); i++) {
      // for (int j = i + 1; j < position.size(); j++) {
      for (int j : nearbyParticles) {
        if ( i == j ) continue;
        HSV q2 = mesh.colors()[j];
        float charge = q1.h * q2.h;
        float u = abs(q1.h - q2.h);
        Vec3f r = position[j] - position[i];
        if (r < 0.333) { charge *= 1.667; }
        Vec3f F = (Vec3f(r).normalize() * charge * u * q * 0.001) / (r.magSqr() + 0.001);
        F = F * ke * 0.0001;
        force[i] -= F * 0.001 + rnd::uniformS() * (1.0 - u) * q * 0.001;
        force[j] += F * 0.001 + rnd::uniformS() * (1.0 - u) * q * 0.001;
      }
      // }
    }


    // drag
    for (int i = 0; i < velocity.size(); i++) {
      force[i] += - velocity[i] * dragFactor;
    }

    // Integration
    // vector<Vec3f> &position(mesh.vertices());
    for (int i = 0; i < velocity.size(); i++) {
      // "semi-implicit" Euler integration
      velocity[i] += force[i] / mass[i] * timeStep;
      position[i] += velocity[i] * timeStep;
    }

    // clear all accelerations (IMPORTANT!!)
    for (auto &a : force) a.set(0);
  }

  bool onKeyDown(const Keyboard &k) override {
    if (k.key() == ' ') {
      freeze = !freeze;
    }

    if (k.key() == '1') {
      // introduce some "random" forces
      for (int i = 0; i < velocity.size(); i++) {
        // F = ma
        force[i] += randomVec3f(1);
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
