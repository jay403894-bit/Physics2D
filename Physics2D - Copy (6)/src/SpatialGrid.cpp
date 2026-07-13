#include "../include/SpatialGrid.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

namespace Physics2D {

SpatialGrid::SpatialGrid(int worldW, int worldH, int cellSize, int capacity)
    : cellSize(cellSize), capacity(capacity) {
    gridW = worldW / cellSize + 1;
    gridH = worldH / cellSize + 1;
    head.resize(gridW * gridH);
    nodes.reserve(capacity * 4);
}

void SpatialGrid::clear() {
    std::fill(head.begin(), head.end(), -1);
    nodes.clear();
}

void SpatialGrid::insert(int idx, float x, float y) {
    int gx = std::max(0, std::min((int)(x / cellSize), gridW - 1));
    int gy = std::max(0, std::min((int)(y / cellSize), gridH - 1));
    int cellIdx = gx + gy * gridW;

    int nodeIdx = (int)nodes.size();
    nodes.push_back({ idx, head[cellIdx] });
    head[cellIdx] = nodeIdx;
}

void SpatialGrid::insert(int idx, float x, float y, float w, float h) {
    float halfW = w * 0.5f;
    float halfH = h * 0.5f;
    // Each bound clamped independently to [0, gridDim-1] -- a one-sided clamp (e.g. only
    // std::max(0,...) on the min side) lets minX end up ABOVE maxX whenever the whole rect
    // falls outside the grid on one side, silently emptying the cx/cy loop below.
    int minX = std::clamp((int)((x - halfW) / cellSize), 0, gridW - 1);
    int maxX = std::clamp((int)((x + halfW) / cellSize), 0, gridW - 1);
    int minY = std::clamp((int)((y - halfH) / cellSize), 0, gridH - 1);
    int maxY = std::clamp((int)((y + halfH) / cellSize), 0, gridH - 1);

    for (int cy = minY; cy <= maxY; ++cy) {
        for (int cx = minX; cx <= maxX; ++cx) {
            int cellIdx = cy * gridW + cx;

            int nodeIdx = (int)nodes.size();
            nodes.push_back({ idx, head[cellIdx] });
            head[cellIdx] = nodeIdx;
        }
    }
}

void SpatialGrid::query(int idx, float x, float y, float w, float h, std::vector<int>& neighbors, QueryScratchpad& scratch) {
    scratch.reset(capacity);
    neighbors.clear();

    // Protection against NaN or Infinite positions dragging down integer casting
    if (std::isnan(x) || std::isnan(y) || std::isnan(w) || std::isnan(h)) return;

    // Calculate the grid cell range for the query rectangle -- center position plus half-extents,
    // clamped to grid bounds. (x,y,w,h) are (centerX, centerY, halfWidth, halfHeight), same
    // convention as insert() above -- must be symmetric around the center on both axes, or cells
    // to the left/above the center get silently skipped. Each bound is ALSO clamped independently
    // to [0, gridDim-1] (not one-sided max(0,...)/min(gridDim-1,...)) -- otherwise once the query
    // rect falls entirely outside the grid on one side, minX can end up above maxX and the loop
    // below silently runs zero times (this is what caused collision to vanish well before an
    // actor ever reached a tile's true edge, once its position exceeded the grid's nominal size).
    int minX = std::clamp((int)((x - w) / cellSize), 0, gridW - 1);
    int maxX = std::clamp((int)((x + w) / cellSize), 0, gridW - 1);
    int minY = std::clamp((int)((y - h) / cellSize), 0, gridH - 1);
    int maxY = std::clamp((int)((y + h) / cellSize), 0, gridH - 1);

    if (scratch.lastSeen.size() != (size_t)capacity) {
        scratch.lastSeen.assign(capacity, 0);
    }
    scratch.queryVersion++;

    for (int cy = minY; cy <= maxY; ++cy) {
        for (int cx = minX; cx <= maxX; ++cx) {
            int cellIdx = cy * gridW + cx;

            if (cellIdx < 0 || cellIdx >= (int)head.size()) {
                printf("[Physics2D::SpatialGrid] cellIdx %d out of bounds for head size %zu!\n", cellIdx, head.size());
                return;
            }

            int nodeIdx = head[cellIdx];

            while (nodeIdx != -1) {
                if (nodeIdx < 0 || nodeIdx >= (int)nodes.size()) {
                    printf("[Physics2D::SpatialGrid] nodeIdx %d out of bounds for nodes size %zu! (Did you forget to reset head to -1 on clear?)\n", nodeIdx, nodes.size());
                    return;
                }

                int entityId = nodes[nodeIdx].id;

                if (entityId < 0 || entityId >= (int)scratch.lastSeen.size()) {
                    printf("[Physics2D::SpatialGrid] entityId %d out of bounds for scratch size %zu! Max allowed is %d\n", entityId, scratch.lastSeen.size(), capacity);
                    return;
                }

                if (entityId != idx && scratch.lastSeen[entityId] != scratch.queryVersion) {
                    neighbors.push_back(entityId);
                    scratch.lastSeen[entityId] = scratch.queryVersion;
                }
                nodeIdx = nodes[nodeIdx].next;
            }
        }
    }
}

} // namespace Physics2D
