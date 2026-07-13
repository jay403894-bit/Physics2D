#pragma once
#include <vector>
#include <functional>
#include "Vec2.h"
#include "CollisionLayer.h"

// Forward declarations only! DO NOT include PhysicsWorld.h/EntityTable.h/PlatformTable.h here.
namespace Physics2D {
struct PhysicsWorld;
struct SpatialGrid;
class EntityTable;
class PlatformTable;
}
namespace JLib {
class TaskScheduler;
}

namespace Physics2D {

struct CollisionConstraint {
    bool resolve = false;
    float deltaY = 0.0f;
    float deltaX = 0.0f;
    bool killVelY = false;
    bool killVelX = false;
};

// Bundles the four pieces every non-trivial physics function needs, so call
// sites don't have to thread world/grid/entities/platforms through separately.
struct PhysicsContext {
    PhysicsWorld& world;
    SpatialGrid& grid;
    EntityTable& entities;
    PlatformTable& platforms;
};

class PhysicsSystem {
public:
    static int TileAboveSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);
    static int LeftTileSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);
    static int RightTileSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);
    static int TileBelowSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);
    // Wider, slope-robust alternative to TileBelowSensor -- see its comment in the .cpp for why.
    // Used by CheckGrounding; TileBelowSensor itself is left as-is (and still used elsewhere,
    // e.g. CheckHorizontalCollision's adjacentToSlope check) since that usage doesn't need it.
    static int FindFloorMultiTile(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);
    static int PlatformSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid);

    static void ApplyPhysics(int i, PhysicsContext& ctx, float dt);
    static void ResolveParticle(int i, PhysicsContext& ctx);
    static void SyncPrevPositions(PhysicsWorld& world);

    // Single-threaded chunk update (was UpdateParticleChunk in the raylib source).
    static void UpdateChunk(int start, int end, float dt, PhysicsContext& ctx, bool gridMovement = false);
    // Dispatches UpdateChunk across [0, ctx.world.activeCount) via JLib::TaskScheduler::ParallelFor.
    static void UpdateWorldParallel(PhysicsContext& ctx, float dt, JLib::TaskScheduler& sched,
                                     int chunkSize, bool gridMovement = false);

    static void UpdateGroups(PhysicsContext& ctx, float dt);
    static void LinkGroups(PhysicsContext& ctx);
    static void UpdatePlatforms(PhysicsContext& ctx, float dt);

private:
    static float Lerp(float a, float b, float t);
    static void UpdateEntityRotation(int index, PhysicsWorld& world, Vec2 floorNormal);
    static void WallSlideCheck(int i, PhysicsContext& ctx);
    static void PlatformPhysics(int i, PhysicsContext& ctx, float dt);
    static Vec2 GetSlopeNormal(SlopeType type);
    static void HandleSlopeSnap(int i, PhysicsContext& ctx, float dx);
    static CollisionConstraint CheckSlopeSnap(int i, PhysicsContext& ctx, float dx);
    static CollisionConstraint CheckHorizontalCollision(int i, PhysicsContext& ctx, float dx);
    static CollisionConstraint CheckGrounding(int i, PhysicsContext& ctx);
    static CollisionConstraint CheckCeilingCollision(int i, PhysicsContext& ctx);
    static int GetFloorCandidate(int entityIdx, float testX, float testY, const PhysicsWorld& world, SpatialGrid& grid);
    static int GetHorizontalSensor(int i, PhysicsWorld& world, SpatialGrid& grid, float dx);
    static bool IsBlocked(int i, PhysicsContext& ctx, float dir);
};

} // namespace Physics2D
