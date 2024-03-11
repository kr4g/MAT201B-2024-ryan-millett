/*
Allocore Example: Many Shape Mesh

Description:
This demonstrates how a single mesh can be used to draw many different shapes.

Author:
Lance Putnam, April 2011
*/

#include "al/app/al_App.hpp"
#include "al/graphics/al_Shapes.hpp"
#include "al/math/al_Random.hpp"
using namespace al;

std::vector<int> shapeIdx;
std::vector<int> shapeNv;
std::vector<Vec3f> shapePos;

struct MyApp : public App {
  Mesh shapes;

  void onCreate() override {
    for (int i = 0; i < 800; ++i) {
      int Nv = rnd::prob(0.5) ? (rnd::prob(0.5) ? addCube(shapes)
                                                : addDodecahedron(shapes))
                              : addIcosahedron(shapes);
      shapeNv.push_back(Nv);
      // Scale and translate the newly added shape
      Mat4f xfm;
      xfm.setIdentity();
      xfm.scale(Vec3f(rnd::uniform(1., 0.1), rnd::uniform(1., 0.1),
                      rnd::uniform(1., 0.1)));
      Vec3f pos(rnd::uniformS(8.), rnd::uniformS(8.), rnd::uniformS(8.));
      shapePos.push_back(pos);
      xfm.translate(pos);
      int startIdx = shapes.vertices().size() - Nv;
      shapeIdx.push_back(startIdx);
      shapes.transform(xfm, startIdx);

      // Color newly added vertices
      for (int i = 0; i < Nv; ++i) {
        float f = float(i) / Nv;
        shapes.color(HSV(f * 0.1 + 0.2, 1, 1));
      }
    }

    // Convert to non-indexed triangles to get flat shading
    // shapes.decompress();
    shapes.generateNormals();

    nav().pos(0, 0, 24);
  }

  void onAnimate(double dt) override {
    // Rotate the shapes
    for (int i = 0; i < shapeIdx.size(); ++i) {
      Mat4f xfm;
      xfm.setIdentity();
      // xfm.rotate(0.3, 0.5, 0.7);
      xfm.translate(0.1);
      shapes.transform(xfm, shapeIdx[i], shapeIdx[i] + shapeNv[i]);
    }
  }

  void onDraw(Graphics& g) override {
    g.clear(0);
    gl::depthTesting(true);
    g.lighting(true);
    g.meshColor();  // with lighting, use mesh's color array
    g.draw(shapes);
  }
};

int main() {
  MyApp app;
  app.start();
}