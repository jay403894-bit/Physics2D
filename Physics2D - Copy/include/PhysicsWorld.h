#pragma once
#include <vector>
#include <cstdint>
#include <mutex>
#include <functional>
#include "Vec2.h"
#include "CollisionLayer.h"
#include "EntityTable.h"
#include "PlatformTable.h"
#include "ParticleTable.h"

namespace Physics2D {

struct PhysicsWorld;
using CollisionCallback = std::function<void(int, int, PhysicsWorld&, float, float)>;

// Dense, swap-compacted SoA for every physical cell (tiles, particles, and entities
// alike). Actor-only and platform-only behavior state lives in the small stable-slot
// `entities`/`platforms` pools instead, reached via the `entitySlot`/`platformSlot`
// back-pointers below -- see EntityTable.h/PlatformTable.h for why those pools use a
// freelist rather than dense compaction.
struct PhysicsWorld {
    std::vector<ShapeType> shapeType;
    std::vector<float> gravity;
    std::vector<float> posX, posY;
    std::vector<float> velX, velY;
    std::vector<float> life;
    std::vector<Vec2> size;
    std::vector<size_t> age;
    std::vector<uint8_t> mask;
    std::vector<uint8_t> layer;
    mutable std::mutex particle_mutex_; // Protect activeCount and spawn/kill operations
    std::vector<float> rotation;
    std::vector<float> angularVel;
    std::vector<uint8_t> isEntity;
    std::vector<uint8_t> isTile;
    std::vector<uint8_t> isParticle;
    std::vector<uint8_t> isPlatform;
    std::vector<uint8_t> isGhosted;
    std::vector<uint8_t> isSlope;
    std::vector<SlopeType> slopeType;
    std::vector<uint8_t> health;     // Durability (so particles can chip away at tiles)
    std::vector<float> friction;
    std::vector<float> restitution;
    std::vector<CollisionCallback> onCollision;
    std::vector<uint8_t> hasCollisionEvent;
    std::vector<Vec2> prevPos;
    std::vector<uint32_t> tileID;
    std::vector<int32_t> entitySlot;   // -1 if this cell is not an actor; else index into `entities`
    std::vector<int32_t> platformSlot; // -1 if this cell is not a behavior-platform; else index into `platforms`
    std::vector<int32_t> particleSlot; // -1 if this cell is not a particle; else index into `particles`

    size_t activeCount = 0;
    size_t maxEntities = 0;

    EntityTable entities;
    PlatformTable platforms;
    ParticleTable particles;

    PhysicsWorld() : entities(0), platforms(0), particles(0) {}
    PhysicsWorld(size_t maxCells, size_t maxActors, size_t maxPlatforms, size_t maxParticles)
        : maxEntities(maxCells), entities(maxActors), platforms(maxPlatforms), particles(maxParticles) {
        reset();
    }

    void reset();

    int spawn(ShapeType shape, Vec2 size, float x, float y, float vx, float vy, float lifetime,
              float gravity = 0.0f, uint8_t layer = 0, uint8_t mask = 0,
              bool isEntity = false, bool isPlatform = false, bool isParticle = false);

    void compressArrays();
};

} // namespace Physics2D
