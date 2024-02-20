// Ryan Millett
// MAT201B-2024
//
#include "al/app/al_App.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"
#include "al/math/al_Vec.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Functions.hpp"

#include "../utils/octtree.cpp"
// #include "classes/boid_3.cpp"

const int CUBE_SIZE = 10;

const int MAX_BOIDS = 350;
const float MAX_BOID_RADIUS = CUBE_SIZE / 3.0;
// const float MAX_PREDATORS = MAX_BOIDS * 0.1;

const int N_PARTICLES = 2500;

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

struct MyApp : App {
  
  Parameter timeStep{"Time Step", "", 2.0, "", 0.0333, 5.0};
  Parameter pointSize{"/pointSize", "", 0.5, 0.05, 6.0};
  Parameter cohesionThresh{"Cohesion Threshold", "", 0.96, 0.0001, MAX_BOID_RADIUS};
  Parameter cohesionForce{"Cohesion Force", "", 0.125, 0.0001, 1.0};
  Parameter separationThresh{"Separation Threshold", "", 0.75, 0.0001, MAX_BOID_RADIUS};
  Parameter separationForce{"Separation Force", "", 0.75, 0.0001, 1.0};
  Parameter alignmentThresh{"Alignment Threshold", "", 1.1, 0.0001, MAX_BOID_RADIUS};
  Parameter alignmentForce{"Alignment Force", "", 0.25, 0.0001, 1.0};
  Parameter bRadius{"Boid Radius", "", 1.25, 0.005, MAX_BOID_RADIUS};
  
  std::vector<Boid> boids;    
  std::vector<Vec3f> food;
  vector<Vec3f> velocity;
  vector<Vec3f> force;
  vector<float> mass;
  // Octree foodTree;
  Octree* boidTree{nullptr};

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
  // Mesh lineMesh{Mesh::LINES};


  // Nav point;

  ShaderProgram pointShader;

  void onCreate() override {
    pointShader.compile(slurp("../point-vertex.glsl"),
                        slurp("../point-fragment.glsl"),
                        slurp("../point-geometry.glsl"));

    setUp();

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

    // Male Prey Body
    preyMeshMale.primitive(Mesh::TRIANGLE_FAN);
		preyMeshMale.vertex(0, 0, -3);        // Noise
		preyMeshMale.color(0, 0.5, 1.0);
		preyMeshMale.vertex(0, 1, 0);         // Top center edge ("back")
		preyMeshMale.color(0.45, 0.17, 0.28);
		preyMeshMale.vertex(-3, -1, 0);       // Left edge
		preyMeshMale.color(0, 0.15, 0.7);
		preyMeshMale.vertex(3, -1, 0);        // Right edge
		preyMeshMale.color(0.08, 0.08, 0.60);
		preyMeshMale.vertex(0, 1, 0);         // Top center edge, closing the fan
		preyMeshMale.color(0.45, 0.17, 0.28);

    // Female Prey Body
    preyMeshFemale.primitive(Mesh::TRIANGLE_FAN);
		preyMeshFemale.vertex(0, 0, -5);      // Noise
		preyMeshFemale.color(0.6, 1.0, 0.2);
		preyMeshFemale.vertex(0, 0.5, 0);     // Top center edge ("back")
		preyMeshFemale.color(0.2, 0.7, 0.1);
		preyMeshFemale.vertex(-1, 0, 0);      // Left edge
		preyMeshFemale.color(0.3, 0.8, 0.2);
		preyMeshFemale.vertex(1, 0, 0);       // Right edge
		preyMeshFemale.color(0.3, 0.8, 0.2);
		preyMeshFemale.vertex(0, 0.5, 0);     // Top center edge, closing the fan
		preyMeshFemale.color(0.2, 0.7, 0.1);

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
      velocity.push_back(randomVec3f(-0.025));
      force.push_back(randomVec3f(-0.000001));
    }
  }  
  
  Vec3d target = Vec3d(r(), r(), r());
  void setUp() {         
      boidTree = new Octree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.5f);
      boids.clear();
      for (int i = 0; i < MAX_BOIDS; i++) {        
        Boid b;
        randomize(b.bNav);
        // b.seek(target, rnd::uniform(0.005, 0.09), rnd::uniform(0.05, 0.75));
        boids.push_back(b);
      }
  }
  
  void randomize(Nav& boidNav) {
    boidNav.pos(randomVec3f(CUBE_SIZE));
    boidNav.quat().set(r(), r(), r(), r()).normalize();
  }
  
  bool freeze = false;
  double phase = 0;
  double trackingReset = 0.0;
  void onAnimate(double dt) override {
    if (freeze) return;
    dt *= timeStep.get();
    time += dt;
    
    // makeTrees();

    // // vector<Nav*> &boidPosition(navPtrs);
    vector<Vec3f> &foodPosition(foodMesh.vertices());

    // // Octree foodTree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.05f);
    // // foodTree.build(foodPosition);
    
    boidTree->build(boids);

    // // bool findFood = false;
    // phase += dt;
    // float phaseReset = 15 * timeStep.get();
    // if (phase > phaseReset) {
    //   phase -= phaseReset;
    //   // findFood = true;
    //   target = randomVec3f(CUBE_SIZE*0.833);
    // } else {
    //   std::vector<int> i_boids;
    //   boidTree.queryRegion(target, Vec3f(5, 5, 5), i_boids);
    //   if (i_boids.size() > MAX_BOIDS * 0.33) {
    //     target = randomVec3f(CUBE_SIZE);
    //   }
    // }

    // lineMesh.reset();
    Vec3d boidCenterOfMass(0, 0, 0);
    for (auto& b : boids) {
      boidCenterOfMass += b.bNav.pos();
      
      b.handleBoundary(CUBE_SIZE);
      
      vector<int> i_boids;
      boidTree->queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), i_boids);

      // for (int i : i_boids) {        
      //   lineMesh.vertex(b.bNav.pos());
      //   lineMesh.vertex(boids[i].bNav.pos());
      //   lineMesh.color(1.0, 1.0, 1.0);
      // }

      b.alignment(boids, i_boids, alignmentThresh.get(), alignmentForce.get());
      b.cohesion(boids, i_boids, cohesionThresh.get(), cohesionForce.get());
      b.separation(boids, i_boids, separationThresh.get(), separationForce.get());
      
      // b.seek(target, rnd::uniform(0.01, 0.9), rnd::uniform(0.05, 0.75));
      // float distance = (b.bNav.pos() - target).mag();
      // if (distance > CUBE_SIZE * 0.75) {
      //   b.findFood(foodTree, CUBE_SIZE*0.25, foodPosition, mass);
      // } else {
      // }
      // float dist = (b.bNav.pos() - b.target).mag();
      // if (dist > CUBE_SIZE * 0.5) {
      //   double proximity = (CUBE_SIZE - dist) / CUBE_SIZE;
      //   b.seek(target, rnd::uniform(0.006*proximity, 0.1*proximity));
      // }
      // b.findFood(foodTree, CUBE_SIZE*0.125, foodPosition, mass);
      // b.detectSurroundings(boidTree, 2.0, boidPosition);

      b.updatePosition(0.667, dt);
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

    nav().faceToward(boidCenterOfMass, Vec3d(0, 1, 0), 0.2);
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
    axes.draw(g);

    int i = 0;
    for (auto& b : boids) {
      {
        Nav& a(b.bNav);
        g.pushMatrix();
        g.translate(a.pos());
        g.rotate(a.quat());
        g.scale(
          (i % 11 != 0) ? 0.0125 : 0.005
        );
        g.draw(
          (i % 11 != 0) ? preyMeshMale : preyMeshFemale
        );
        g.popMatrix();
      }
      Octree boidTree(Vec3f(0, 0, 0), Vec3f(CUBE_SIZE, CUBE_SIZE, CUBE_SIZE), 0.5f);
      boidTree.build(boids);    
      vector<int> i_boids;
      boidTree.queryRegion(b.bNav.pos(), Vec3f(bRadius.get()), i_boids);
      Mesh m{Mesh::LINES};
      for (int j : i_boids) {
        if (i == j) continue;  
        m.vertex(b.bNav.pos());
        m.vertex(boids[j].bNav.pos());
        m.color(1.0, 1.0, 1.0);
        g.draw(m);
      }
      ++i;
    }

    g.shader(pointShader);
    g.shader().uniform("pointSize", pointSize / 100);
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
    gui.add(cohesionThresh);
    gui.add(cohesionForce);
    gui.add(separationThresh);
    gui.add(separationForce);
    gui.add(alignmentThresh);
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
