// Ryan Millett
// MAT201B - 2024

#include "al/app/al_App.hpp"
#include "al/app/al_GUIDomain.hpp"
#include "al/math/al_Random.hpp"

using namespace al;

#include <fstream>
#include <vector>
using namespace std;

struct OctreeNode {
    Vec3f center;  // Center of the node
    Vec3f halfSize;  // Half the size of the node
    vector<int> particleIndices;  // Indices of particles in this node
    OctreeNode* children[8]{};

    OctreeNode(const Vec3f& c, const Vec3f& hs) : center(c), halfSize(hs) {}

    ~OctreeNode() {
        for (auto& child : children) delete child;
    }

    bool isLeaf() const {
        return !particleIndices.empty();
    }

    bool contains(const Vec3f& point) const {
        return point.x >= center.x - halfSize.x && point.x <= center.x + halfSize.x &&
               point.y >= center.y - halfSize.y && point.y <= center.y + halfSize.y &&
               point.z >= center.z - halfSize.z && point.z <= center.z + halfSize.z;
    }

    int getOctantContainingPoint(const Vec3f& point) const {
        int octant = 0;
        if (point.x >= center.x) octant |= 4;
        if (point.y >= center.y) octant |= 2;
        if (point.z >= center.z) octant |= 1;
        return octant;
    }
};

class Octree {
    OctreeNode* root;
    float minimumSize;

public:
    Octree(const Vec3f& center, const Vec3f& halfSize, float minSize)
        : root(new OctreeNode(center, halfSize)), minimumSize(minSize) {}

    ~Octree() { clear(root); }

    void insertPosition(int particleIndex, const Vec3f& position) {
        insert(root, particleIndex, position);
    }

    void build(const vector<Vec3f>& positions) {
        clear(root);
        for (int i = 0; i < positions.size(); ++i) {
            insertPosition(i, positions[i]);
        }
        // std::cout << "Root: " << root->center << std::endl;
        // std::cout << "n children: " << root->particleIndices.size() << std::endl;
        // std::cout << std::endl;
        // for (int i = 0; i < 8; ++i) {
        //     OctreeNode* child = root->children[i];
        //     while (child) {
        //         std::cout << "Oct " << i + 1 << ", Center: " << child->center << std::endl;
        //         std::cout << "n children: " << child->particleIndices.size() << std::endl;
        //         std::cout << std::endl;
        //         child = child->children[i];
        //     }
        //     // std::cout << "Oct " << i + 1 << ", Center: " << root->children[i]->center << std::endl;
        //     // std::cout << "n children: " << root->children[i]->particleIndices.size() << std::endl;
        //     // std::cout << std::endl;
        // }
    }
    
    void queryRegion(const Vec3f& center, const Vec3f& halfSize, vector<int>& found) const {
        queryRegion(root, center, halfSize, found);
    }

private:
    void insert(OctreeNode* node, int particleIndex, const Vec3f& position) {
        if (!node->contains(position)) return;

        if (node->halfSize.x <= minimumSize) {
            node->particleIndices.push_back(particleIndex);
            // std::cout << "insert: " << particleIndex << " " << node->particleIndices.size() << std::endl;
            return;
        }

        int octant = node->getOctantContainingPoint(position);
        // std::cout << "octant: " << octant << std::endl;
        if (!node->children[octant]) {
            Vec3f newCenter = node->center;
            newCenter.x += node->halfSize.x * (octant & 4 ? 0.5f : -0.5f);
            newCenter.y += node->halfSize.y * (octant & 2 ? 0.5f : -0.5f);
            newCenter.z += node->halfSize.z * (octant & 1 ? 0.5f : -0.5f);
            node->children[octant] = new OctreeNode(newCenter, node->halfSize * 0.5f);
        }

        insert(node->children[octant], particleIndex, position);
    }

    void clear(OctreeNode* node) {
        if (!node) return;
        for (auto& child : node->children) {
            clear(child);
            child = nullptr;
        }
        node->particleIndices.clear();
    }

    void queryRegion(OctreeNode* node, const Vec3f& center, const Vec3f& halfSize, vector<int>& found) const {
        if (!node) return;

        if (!intersects(node->center, node->halfSize, center, halfSize)) {
            return;
        }

        if (node->isLeaf()) {
            // std::cout << "!!!!!!!!" << std::endl;
            for (int particleIndex : node->particleIndices) {
                // std::cout << "found: " << particleIndex << std::endl;
                found.push_back(particleIndex);
            }
        } else {
            for (int i = 0; i < 8; ++i) {
                if (node->children[i]) {
                    queryRegion(node->children[i], center, halfSize, found);
                }
            }
        }
    }

    bool intersects(const Vec3f& center1, const Vec3f& halfSize1, const Vec3f& center2, const Vec3f& halfSize2) const {
        return (abs(center1.x - center2.x) <= (halfSize1.x + halfSize2.x)) &&
               (abs(center1.y - center2.y) <= (halfSize1.y + halfSize2.y)) &&
               (abs(center1.z - center2.z) <= (halfSize1.z + halfSize2.z));
    }
};
