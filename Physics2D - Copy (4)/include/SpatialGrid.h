#pragma once
#include <vector>

namespace Physics2D {

struct QueryScratchpad {
    std::vector<int> lastSeen;
    int queryVersion = 0;

    void reset(int capacity) {
        if (lastSeen.size() != (size_t)capacity) {
            lastSeen.assign(capacity, 0);
        }
        queryVersion++;
    }
};

struct SpatialGrid {
    int gridW, gridH, cellSize;
    int capacity;
    std::vector<int> head;
    struct GridNode {
        int id;   // The PhysicsWorld row index
        int next; // Index of the next node in this cell's linked list
    };
    std::vector<GridNode> nodes;

    // capacity should be PhysicsWorld::maxEntities (the base row count) --
    // SpatialGrid indexes by base row index throughout, never by entity/platform slot.
    SpatialGrid(int worldW, int worldH, int cellSize, int capacity);
    void clear();
    void insert(int idx, float x, float y);
    void insert(int idx, float x, float y, float w, float h);
    void query(int idx, float x, float y, float w, float h, std::vector<int>& neighbors, QueryScratchpad& scratch);
};

} // namespace Physics2D
