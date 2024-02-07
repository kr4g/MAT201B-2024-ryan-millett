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

const int CUBE_SIZE = 25;

const int MAX_BOIDS = 6000;
// const float MAX_PREDATORS = MAX_BOIDS * 0.1;

const int N_PARTICLES = 1600;

using namespace al;

double r() { return rnd::uniformS() * CUBE_SIZE; }
Vec3f randomVec3f(float scale) {
  return Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * scale;
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

// Assuming Vec3f is the vector class used for positions and that you have a list or array of object positions
Vec3f calculateCenterOfMass(const std::vector<Vec3f>& positions) {
    Vec3f center(0, 0, 0);
    for (auto& pos : positions) {
        center += pos;
    }
    center /= positions.size();
    return center;
}

void updateCameraPosition(al::Nav& nav, const Vec3f& centerOfMass, float distance) {
    // This example assumes you have an al::Nav object for the camera (`nav`)
    // and a desired distance from the center of mass to maintain.

    // Calculate a simple orbit around the center of mass
    static float angle = 0.0f;  // Static to keep increasing each frame
    angle += 0.01;  // Adjust rotation speed as needed

    // Calculate new camera position
    Vec3f newPosition = centerOfMass + Vec3f(cos(angle) * distance, 0, sin(angle) * distance);

    // Update camera position and orientation
    nav.pos(newPosition);
    nav.faceToward(centerOfMass);  // Make the camera face towards the center of mass
}

string slurp(string fileName);  // forward declaration

struct MyApp : App {
  
  Parameter timeStep{"Time Step", "", 5.0, "", 0.08333, 5.0};
  Parameter pointSize{"/pointSize", "", 1.667, 0.1, 6.0};
  std::vector<Boid> boids;
  std::vector<Nav*> navPtrs;
  
  std::vector<Vec3f> food;
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
  Mesh preyMeshMale;
  Mesh preyMeshFemale;
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
    nav().pos(CUBE_SIZE, CUBE_SIZE * 0.833, CUBE_SIZE * 1.0833);
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
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.1, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(0, 1, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.1, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(-1, 0, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.1, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(1, 0, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.1, 0.67), rnd::uniform(0.0, 0.33));
    predMesh.vertex(0, 1, 0);
    predMesh.color(rnd::uniform(0.5, 1.0), rnd::uniform(0.1, 0.67), rnd::uniform(0.0, 0.33));

    // Male Prey Body: Enhanced Blue Spectrum with Variation and Specific Highlights
    preyMeshMale.primitive(Mesh::TRIANGLE_FAN);
		preyMeshMale.vertex(0, 0, -2); // Tip of the body ("nose")
		preyMeshMale.color(0.2, 0.6, 1.0); // Brightest blue for the nose
		preyMeshMale.vertex(0, 1, 0); // Top center edge ("back")
		preyMeshMale.color(0, 0.2, 0.7); // Darker blue for the back
		preyMeshMale.vertex(-1, 0, 0); // Left edge
		preyMeshMale.color(0, 0.4, 0.9); // Royal blue for depth
		preyMeshMale.vertex(1, 0, 0); // Right edge
		preyMeshMale.color(0, 0.4, 0.9); // Royal blue for depth
		preyMeshMale.vertex(0, 1, 0); // Top center edge, closing the fan
		preyMeshMale.color(0, 0.2, 0.7); // Darker blue for the back

    // Female Prey Body: Warm Tones with Enhanced Variation and Specific Highlights
    preyMeshFemale.primitive(Mesh::TRIANGLE_FAN);
		preyMeshFemale.vertex(0, 0, -2); // Tip of the body ("nose")
		preyMeshFemale.color(0.6, 1.0, 0.2); // Brightest lime green for the nose
		preyMeshFemale.vertex(0, 1, 0); // Top center edge ("back")
		preyMeshFemale.color(0.2, 0.7, 0.1); // Dark green for the back
		preyMeshFemale.vertex(-1, 0, 0); // Left edge
		preyMeshFemale.color(0.3, 0.8, 0.2); // Olive green, shadow effect
		preyMeshFemale.vertex(1, 0, 0); // Right edge
		preyMeshFemale.color(0.3, 0.8, 0.2); // Olive green, shadow effect
		preyMeshFemale.vertex(0, 1, 0); // Top center edge, closing the fan
		preyMeshFemale.color(0.2, 0.7, 0.1); // Dark green for the back

    
    // foodMesh.primitive(Mesh::POINTS);
    // for (int i = 0; i < food.size(); i++) {
    //   food[i] = Vec3f(rnd::uniformS(), rnd::uniformS(), rnd::uniformS()) * CUBE_SIZE;
    //   foodMesh.vertex(food[i]);
    //   foodMesh.color(0.5, 0.25, 0);
    // }
    auto randomColor = []() { return HSV(rnd::uniform(), 1.0f, 1.0f); };
    foodMesh.primitive(Mesh::POINTS);
    for (int _ = 0; _ < N_PARTICLES; _++) {
      foodMesh.vertex(randomVec3f(CUBE_SIZE));
      foodMesh.color(randomColor());

      float m = rnd::uniform(8.0, 0.5);
      // float m = 3 + rnd::normal() / 2;
      if (m < 0.5) m = 0.5;
      mass.push_back(m);

      // using a simplified volume/size relationship
      foodMesh.texCoord(pow(m, 1.0f / 3), 0);  // s, t

      // separate state arrays
      velocity.push_back(randomVec3f(0.025));
      force.push_back(randomVec3f(0.000001));
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

  Vec3d target = Vec3d(r(), r(), r());
  void setUp() {
      navPtrs.clear();
      boids.clear();
      for (int i = 0; i < MAX_BOIDS; i++) {        
        Boid b;
        randomize(b.bNav);
        b.seek(target, rnd::uniform(0.0001, 0.01), rnd::uniform(0.05, 0.75));
        navPtrs.push_back(&b.bNav); // address of the nav
        boids.push_back(b);
      }
  }
  
  void randomize(Nav& boidNav) {
    boidNav.pos(randomVec3f(CUBE_SIZE*0.167));
    boidNav.quat().set(r(), r(), r(), r()).normalize();
  }
  
  bool freeze = false;
  double phase = 0;
  void onAnimate(double dt) override {
    if (freeze) return;
    dt *= timeStep.get();
    time += dt;

    vector<Nav*> &boidPosition(navPtrs);
    vector<Vec3f> &foodPosition(foodMesh.vertices());

    Octree foodTree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.5f);
    foodTree.build(foodPosition);

    Octree boidTree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.15f);
    boidTree.build(boidPosition);  

    // bool findFood = false;
    phase += dt;
    float phaseReset = 5 * timeStep.get();
    if (phase > phaseReset) {
      phase -= phaseReset;
      // findFood = true;
      target = randomVec3f(CUBE_SIZE*0.667);
    } else {
      std::vector<int> i_boids;
      boidTree.queryRegion(target, Vec3f(15, 15, 15), i_boids);
      if (i_boids.size() > 100) {
        target = randomVec3f(CUBE_SIZE*0.833);
      }
    }

    Vec3d boidCenterOfMass(0, 0, 0);
    for (auto& b : boids) {
      boidCenterOfMass += b.bNav.pos();
      float dist = (b.bNav.pos() - b.target).mag();
      if (dist < 3.5) {
        b.findFood(foodTree, 15, foodPosition, mass);
        // target = Vec3d(r(), r(), r());
      } else if (dist < 5.5) {
        // b.seek(b.target, rnd::uniform(0.01, 0.05), rnd::uniform(0.05, 0.45));
        b.seek(randomVec3f(CUBE_SIZE), rnd::uniform(0.001, 0.08), rnd::uniform(0.15, 0.95));
      } else if (dist < 10.5) {
        // if the targed is too croweded, go elsewhere - XXX: change to a queryRegion, go to nearest low-desire food near the target
        std::vector<int> i_boids;
        boidTree.queryRegion(b.target, Vec3f(9, 9, 9), i_boids);
        if (i_boids.size() > 50) {
          // b.findFood(foodTree, 10, foodPosition, mass);          
          b.seek(randomVec3f(CUBE_SIZE), rnd::uniform(0.001, 0.05), rnd::uniform(0.15, 0.95));
        } 
      } else {        
        b.seek(target, rnd::uniform(0.0008333, 0.008333), rnd::uniform(0.15, 0.95));
      }
      b.detectSurroundings(boidTree, CUBE_SIZE, boidPosition);
      b.updatePosition(rnd::uniform(0.667, 1.0), dt);
      b.updateParams(timeStep.get());
    }
    boidCenterOfMass /= boids.size();    

    for (int i = 0; i < foodPosition.size(); i++) {
      float currentDistance = foodPosition[i].mag();
      float displacement = currentDistance - CUBE_SIZE;
      Vec3f springForce = (displacement < CUBE_SIZE) ? 0.0 : Vec3f(-foodPosition[i]).normalize() * (0.001 * displacement);
      force[i] += springForce * 0.25 + springForce * randomVec3f(0.05) * 0.1; // spring force
      if (displacement > CUBE_SIZE*0.43) {
        force[i] += -velocity[i] * 4.97;     // drag force
      }
      force[i] += -velocity[i] * 0.7;     // drag force

      // if (currentDistance < CUBE_SIZE * 0.1) {
      //   force[i] += Vec3f(-foodPosition[i]).normalize() * 0.000001;
      //   force[i] += -velocity[i] * 0.1;     // drag force
      // }

      // force toward the target
      force[i] += Vec3f(target - foodPosition[i]).normalize() * 0.00012;


      // "semi-implicit" Euler integration
      velocity[i] += force[i] / mass[i] * timeStep;
      foodPosition[i] += velocity[i] * timeStep;

      // vector<int> nearbyParticles;
      // foodTree.queryRegion(foodPosition[i], Vec3f(10, 10, 10), nearbyParticles); 

      // // Repulsion :: [Coulombs law](https://en.wikipedia.org/wiki/Coulomb%27s_law)* :: $F = k_e \frac{q_1 q_2}{r^2}$
      // float ke = 8.987551787e9; // Coulomb's constant in N·m²/C²
      // HSV q1 = foodMesh.colors()[i];
      // for (int j : nearbyParticles) {
      //   if ( i == j ) continue;
      //   HSV q2 = foodMesh.colors()[j];
      //   float charge = q1.h * q2.h;
      //   float u = 0.0001 * abs(q1.h - q2.h);
      //   Vec3f r = foodPosition[j] - foodPosition[i];
      //   // if (r < 0.333) { charge *= 1.667; }
      //   Vec3f F = (Vec3f(r).normalize() * charge) / (r.magSqr() + 0.0000001);
      //   F = F * ke * u * 0.0001;
      //   force[i] -= F * 0.0001;// + rnd::uniformS() * (1.0 - u) * q * 0.001;
      //   force[j] += F * 0.0001;// + rnd::uniformS() * (1.0 - u) * q * 0.001;
      // }
    }

    // clear all accelerations (IMPORTANT!!)
    for (auto &a : force) a.set(0);

    nav().smooth(0.16);
    nav().faceToward(boidCenterOfMass);
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
    // axes.draw(g);

    // {
    //   Mesh mesh(Mesh::LINES);
    //   // draw the axes
    //   mesh.vertex(-CUBE_SIZE, 0, 0);
    //   mesh.vertex(CUBE_SIZE, 0, 0);
    //   mesh.vertex(0, -CUBE_SIZE, 0);
    //   mesh.vertex(0, CUBE_SIZE, 0);
    //   mesh.vertex(0, 0, -CUBE_SIZE);
    //   mesh.vertex(0, 0, CUBE_SIZE);
    //   for (int i = 0; i < CUBE_SIZE; i++) mesh.color(1,1,1);

    //   g.draw(mesh);
    // }

    // draw a body for each agent
    int i = 0;
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
          (i % 13 != 0) ? 0.125 : 0.112
          // 0.167
        );
        g.draw(
          // prey are blue/green, predators are red/orange
          (i % 13 != 0) ? preyMeshMale : preyMeshFemale
          // preyMeshMale
        );
        g.popMatrix();  // pop()
      }
      ++i;
    }
    
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
