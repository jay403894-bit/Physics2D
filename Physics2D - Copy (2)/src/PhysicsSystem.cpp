#include "../include/PhysicsSystem.h"
#include "../include/PhysicsWorld.h"
#include "../include/SpatialGrid.h"
#include "../include/EntityTable.h"
#include "../include/PlatformTable.h"
#include <TaskScheduler.h>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <numbers>
#include <cstdio>

namespace Physics2D {

namespace {
constexpr float kPi = std::numbers::pi_v<float>;

// Free helper, ported verbatim from the source project (not currently called
// anywhere in PhysicsSystem, kept for parity).
bool IsPlayerOnPlatform(float playerX, float centerX) {
    // 80.0f + 2.0f buffer to prevent edge-snapping jitter
    return std::abs(playerX - centerX) <= (80.0f + 2.0f);
}

float GetSlopeSurfaceY(int slopeID, float playerX, const PhysicsWorld& world) {
    float left = world.posX[slopeID] - world.size[slopeID].x * 0.5f;
    float right = world.posX[slopeID] + world.size[slopeID].x * 0.5f;
    float top = world.posY[slopeID] - world.size[slopeID].y * 0.5f;
    float bottom = world.posY[slopeID] + world.size[slopeID].y * 0.5f;
    float height = world.size[slopeID].y;

    float clampedX = std::clamp(playerX, left, right);
    float t = (clampedX - left) / world.size[slopeID].x;

    if (world.slopeType[slopeID] == SlopeType::UP_RIGHT)
        return top + (1.0f - t) * height;
    if (world.slopeType[slopeID] == SlopeType::UP_LEFT)
        return top + t * height;
    return bottom - t * height; // DOWN_LEFT / DOWN_RIGHT
}
} // namespace

int PhysicsSystem::TileAboveSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid) {
    float hw = world.size[entityIdx].x * 0.5f;
    float hh = world.size[entityIdx].y * 0.5f;

    float sensorX = world.posX[entityIdx];
    float sensorY = world.posY[entityIdx] - hh - 2.0f;
    float sensorHW = 1.0f;
    float sensorHH = hh * 0.8f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(entityIdx, sensorX, sensorY, sensorHW, sensorHH, neighbors, scratch);

    for (int otherID : neighbors) {
        if (!world.isTile[otherID] || world.layer[otherID] == 0 || world.life[otherID] <= 0.0f)
            continue;

        float tileMinX = world.posX[otherID] - (world.size[otherID].x * 0.5f);
        float tileMaxX = world.posX[otherID] + (world.size[otherID].x * 0.5f);
        float tileMinY = world.posY[otherID] - (world.size[otherID].y * 0.5f);
        float tileMaxY = world.posY[otherID] + (world.size[otherID].y * 0.5f);

        float sMinX = sensorX - sensorHW;
        float sMaxX = sensorX + sensorHW;
        float sMinY = sensorY - sensorHH;
        float sMaxY = sensorY + sensorHH;

        if (sMaxX > tileMinX && sMinX < tileMaxX && sMaxY > tileMinY && sMinY < tileMaxY) {
            return otherID;
        }
    }
    return -1;
}

int PhysicsSystem::LeftTileSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid) {
    float hw = world.size[entityIdx].x * 0.5f;
    float hh = world.size[entityIdx].y * 0.5f;

    float sensorX = world.posX[entityIdx] - hw - 2.0f;
    float sensorY = world.posY[entityIdx];
    float sensorHW = 1.0f;
    float sensorHH = hh * 0.8f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(entityIdx, sensorX, sensorY, sensorHW, sensorHH, neighbors, scratch);

    for (int otherID : neighbors) {
        if (!world.isTile[otherID] || world.layer[otherID] == 0 || world.life[otherID] <= 0.0f)
            continue;

        float tileMinX = world.posX[otherID] - (world.size[otherID].x * 0.5f);
        float tileMaxX = world.posX[otherID] + (world.size[otherID].x * 0.5f);
        float tileMinY = world.posY[otherID] - (world.size[otherID].y * 0.5f);
        float tileMaxY = world.posY[otherID] + (world.size[otherID].y * 0.5f);

        float sMinX = sensorX - sensorHW;
        float sMaxX = sensorX + sensorHW;
        float sMinY = sensorY - sensorHH;
        float sMaxY = sensorY + sensorHH;

        if (sMaxX > tileMinX && sMinX < tileMaxX && sMaxY > tileMinY && sMinY < tileMaxY) {
            return otherID;
        }
    }
    return -1;
}

int PhysicsSystem::RightTileSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid) {
    float hw = world.size[entityIdx].x * 0.5f;
    float hh = world.size[entityIdx].y * 0.5f;

    float sensorX = world.posX[entityIdx] + hw + 2.0f;
    float sensorY = world.posY[entityIdx];
    float sensorHW = 1.0f;
    float sensorHH = hh * 0.8f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(entityIdx, sensorX, sensorY, sensorHW, sensorHH, neighbors, scratch);

    for (int otherID : neighbors) {
        if (!world.isTile[otherID] || world.layer[otherID] == 0 || world.life[otherID] <= 0.0f)
            continue;

        float tileMinX = world.posX[otherID] - (world.size[otherID].x * 0.5f);
        float tileMaxX = world.posX[otherID] + (world.size[otherID].x * 0.5f);
        float tileMinY = world.posY[otherID] - (world.size[otherID].y * 0.5f);
        float tileMaxY = world.posY[otherID] + (world.size[otherID].y * 0.5f);

        float sMinX = sensorX - sensorHW;
        float sMaxX = sensorX + sensorHW;
        float sMinY = sensorY - sensorHH;
        float sMaxY = sensorY + sensorHH;

        if (sMaxX > tileMinX && sMinX < tileMaxX && sMaxY > tileMinY && sMinY < tileMaxY) {
            return otherID;
        }
    }
    return -1;
}

int PhysicsSystem::TileBelowSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid) {
    float hw = world.size[entityIdx].x * 0.5f;
    float hh = world.size[entityIdx].y * 0.5f;

    float sensorX = world.posX[entityIdx];
    float sensorY = world.posY[entityIdx] + hh + 2.0f;
    float sensorHW = 1.0f;
    float sensorHH = hh * 0.8f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(entityIdx, sensorX, sensorY, sensorHW, sensorHH, neighbors, scratch);

    for (int otherID : neighbors) {
        if (!world.isTile[otherID] || world.layer[otherID] == 0 || world.life[otherID] <= 0.0f)
            continue;

        float tileMinX = world.posX[otherID] - (world.size[otherID].x * 0.5f);
        float tileMaxX = world.posX[otherID] + (world.size[otherID].x * 0.5f);
        float tileMinY = world.posY[otherID] - (world.size[otherID].y * 0.5f);
        float tileMaxY = world.posY[otherID] + (world.size[otherID].y * 0.5f);

        float sMinX = sensorX - sensorHW;
        float sMaxX = sensorX + sensorHW;
        float sMinY = sensorY - sensorHH;
        float sMaxY = sensorY + sensorHH;

        if (sMaxX > tileMinX && sMinX < tileMaxX && sMaxY > tileMinY && sMinY < tileMaxY) {
            return otherID;
        }
    }
    return -1;
}

int PhysicsSystem::PlatformSensor(int entityIdx, const PhysicsWorld& world, SpatialGrid& grid) {
    float sensorHW = world.size[entityIdx].x * 0.3f;
    float sensorHH = 6.0f;
    float sensorX = world.posX[entityIdx];
    float sensorY = world.posY[entityIdx] + (world.size[entityIdx].y * 0.5f) + 2.0f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(entityIdx, sensorX, sensorY, sensorHW, sensorHH, neighbors, scratch);

    int bestTile = -1;
    float closestDist = 9999.0f;

    for (int otherID : neighbors) {
        if (world.isPlatform[otherID]) {
            float platformHW = world.size[otherID].x * 0.5f;
            float platformMinX = world.posX[otherID] - platformHW;
            float platformMaxX = world.posX[otherID] + platformHW;
            float sensorMinX = sensorX - sensorHW;
            float sensorMaxX = sensorX + sensorHW;

            if (platformMaxX > sensorMinX && platformMinX < sensorMaxX) {
                float dist = std::abs(world.posX[otherID] - sensorX);
                if (dist < closestDist) {
                    closestDist = dist;
                    bestTile = otherID;
                }
            }
        }
    }
    return bestTile;
}

float PhysicsSystem::Lerp(float a, float b, float t) {
    return a + t * (b - a);
}

void PhysicsSystem::UpdateEntityRotation(int index, PhysicsWorld& world, Vec2 floorNormal) {
    // atan2(y, x) gives the angle; adjust by PI/2 because the sprite's "up" is usually at angle 0.
    float targetRotation = atan2(floorNormal.y, floorNormal.x) + (kPi / 2.0f);
    world.rotation[index] = Lerp(world.rotation[index], targetRotation, 0.1f);
}

int PhysicsSystem::GetHorizontalSensor(int i, PhysicsWorld& world, SpatialGrid& grid, float dx) {
    float dir = (dx > 0) ? 1.0f : -1.0f;
    float checkX = world.posX[i] + (world.size[i].x * 0.5f) * dir;
    float stripHW = 0.1f;
    float stripHH = world.size[i].y * 0.4f;

    float sMinX = checkX - stripHW;
    float sMaxX = checkX + stripHW;
    float sMinY = world.posY[i] - stripHH;
    float sMaxY = world.posY[i] + stripHH;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    grid.query(i, checkX, world.posY[i], stripHW, stripHH, neighbors, scratch);

    for (int otherID : neighbors) {
        if (i == otherID || !world.isTile[otherID]) continue;
        if (world.layer[otherID] == 0 || world.life[otherID] <= 0.0f) continue;
        if (world.isGhosted[otherID]) continue;
        if (world.slopeType[otherID] != SlopeType::NONE) {
            // Only skip (let CheckSlopeSnap/CheckHorizontalCollision's slope-climb path handle
            // it) when approaching from the slope's LOW side in its "uphill" direction -- e.g.
            // moving right into an UP_RIGHT slope. Approaching from the HIGH/back side (moving
            // left into an UP_RIGHT slope) must still register as a normal wall hit here, or the
            // entity walks straight through the vertical edge with no horizontal resistance and
            // gets vacuumed up onto the surface by the grounding sensor instead of being blocked.
            bool climbableFromThisDirection =
                (world.slopeType[otherID] == SlopeType::UP_RIGHT && dir > 0.0f) ||
                (world.slopeType[otherID] == SlopeType::UP_LEFT && dir < 0.0f);
            if (climbableFromThisDirection) continue;
        }

        float tMinX = world.posX[otherID] - world.size[otherID].x * 0.5f;
        float tMaxX = world.posX[otherID] + world.size[otherID].x * 0.5f;
        float tMinY = world.posY[otherID] - world.size[otherID].y * 0.5f;
        float tMaxY = world.posY[otherID] + world.size[otherID].y * 0.5f;

        if (sMaxX > tMinX && sMinX < tMaxX && sMaxY > tMinY && sMinY < tMaxY) {
            return otherID;
        }
    }
    return -1;
}

bool PhysicsSystem::IsBlocked(int i, PhysicsContext& ctx, float dir) {
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];
    int parent = (slot != -1) ? ctx.entities.parentEntity[slot] : -1;

    float halfWidth = world.size[i].x * 0.5f;
    float halfHeight = world.size[i].y * 0.5f;
    float edgeX = (dir > 0) ? (world.posX[i] + halfWidth) : (world.posX[i] - halfWidth);

    float probeX = edgeX;
    float probeWidth = 1.0f;
    float probeHeight = halfHeight * 0.8f;

    QueryScratchpad scratch;
    std::vector<int> neighbors;
    ctx.grid.query(i, probeX, world.posY[i], probeWidth, probeHeight, neighbors, scratch);

    for (int otherID : neighbors) {
        if (i == otherID || otherID == parent ||
            !world.isTile[otherID] || world.isGhosted[otherID] ||
            world.isPlatform[otherID] || world.layer[otherID] == 0 ||
            world.life[otherID] <= 0.0f) continue;

        float otherHW = world.size[otherID].x * 0.5f;
        float otherHH = world.size[otherID].y * 0.5f;

        float probeMinX = probeX - probeWidth * 0.5f;
        float probeMaxX = probeX + probeWidth * 0.5f;
        float probeMinY = world.posY[i] - probeHeight;
        float probeMaxY = world.posY[i] + probeHeight;

        float tileMinX = world.posX[otherID] - otherHW;
        float tileMaxX = world.posX[otherID] + otherHW;
        float tileMinY = world.posY[otherID] - otherHH;
        float tileMaxY = world.posY[otherID] + otherHH;

        if (probeMaxX > tileMinX && probeMinX < tileMaxX &&
            probeMaxY > tileMinY && probeMinY < tileMaxY) {
            float overlapX = (dir > 0) ? (probeMaxX - tileMinX) : (tileMaxX - probeMinX);
            world.posX[i] -= (dir > 0) ? overlapX : -overlapX;
            return true;
        }
    }
    return false;
}

int PhysicsSystem::GetFloorCandidate(int entityIdx, float testX, float testY, const PhysicsWorld& world, SpatialGrid& grid) {
    return TileBelowSensor(entityIdx, world, grid);
}

void PhysicsSystem::WallSlideCheck(int i, PhysicsContext& ctx) {
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];
    float intent = (slot != -1) ? ctx.entities.moveIntent[slot] : 0.0f;

    int leftTile = LeftTileSensor(i, world, ctx.grid);
    int rightTile = RightTileSensor(i, world, ctx.grid);
    bool onWall = (leftTile >= 0 && world.velY[i] > 0.0f && intent < 0.0f && world.slopeType[leftTile] == SlopeType::NONE)
        || (rightTile >= 0 && world.velY[i] > 0.0f && intent > 0.0f && world.slopeType[rightTile] == SlopeType::NONE);

    if (onWall) {
        float maxSlideSpeed = world.gravity[i] / 15.0f;
        if (world.velY[i] > maxSlideSpeed) world.velY[i] = maxSlideSpeed;
        if (slot != -1) ctx.entities.isSliding[slot] = true;
    } else if (slot != -1) {
        ctx.entities.isSliding[slot] = false;
    }
}

void PhysicsSystem::PlatformPhysics(int i, PhysicsContext& ctx, float dt) {
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];
    if (slot == -1) return; // not an actor, nothing to ride with

    if (world.velY[i] < 0.0f) {
        int p = ctx.entities.parentEntity[slot];
        if (p != -1 && world.platformSlot[p] != -1) {
            ctx.platforms.childEntity[world.platformSlot[p]] = -1;
        }
        ctx.entities.parentEntity[slot] = -1;
    }

    if (ctx.entities.parentEntity[slot] != -1) {
        int p = ctx.entities.parentEntity[slot];
        if (PlatformSensor(i, world, ctx.grid) == -1) {
            ctx.entities.parentEntity[slot] = -1;
            if (world.platformSlot[p] != -1) ctx.platforms.childEntity[world.platformSlot[p]] = -1;
        } else {
            // Carries BOTH the rider's own intended movement AND the platform's own horizontal
            // motion in one shot -- this is now the ONLY horizontal carry mechanism (the caller's
            // per-frame Update no longer does a second position-delta carry on top of this; that
            // was true double-counting for a velocity-driven master. This term alone was
            // insufficient on its own for real-time horizontal riding when tried in isolation, so
            // it's back, with the external carry removed instead of both mechanisms coexisting.
            float moveX = world.velX[i] * dt + world.velX[p] * dt;
            if (!IsBlocked(i, ctx, (moveX > 0) ? 1.0f : -1.0f)) {
                world.posX[i] += moveX;
            }

            if (ctx.entities.parentEntity[slot] != -1) {
                int p2 = ctx.entities.parentEntity[slot];
                int pSlot = world.platformSlot[p2];
                bool platformFalling = (pSlot != -1) && (ctx.platforms.platformState[pSlot] == 2);

                if (!ctx.entities.isJumping[slot] && !platformFalling) {
                    world.posY[i] = world.posY[p2] - (world.size[p2].y * 0.5f) - (world.size[i].y * 0.5f);
                    world.velY[i] = 0.0f;
                } else {
                    if (pSlot != -1) ctx.platforms.childEntity[pSlot] = -1;
                    ctx.entities.parentEntity[slot] = -1;
                }
            }
        }
    }
}

Vec2 PhysicsSystem::GetSlopeNormal(SlopeType type) {
    switch (type) {
    case SlopeType::NONE:       return { 0.0f, -1.0f };
    case SlopeType::UP_RIGHT:   return { -0.707f, -0.707f };
    case SlopeType::UP_LEFT:    return { 0.707f, -0.707f };
    case SlopeType::DOWN_LEFT:  return { 0.707f, 0.707f };
    case SlopeType::DOWN_RIGHT: return { -0.707f, 0.707f };
    default:                    return { 0.0f, -1.0f };
    }
}

void PhysicsSystem::HandleSlopeSnap(int i, PhysicsContext& ctx, float dx) {
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];
    bool jumping = (slot != -1) && ctx.entities.isJumping[slot];

    int potentialSlope = (world.velX[i] > 0.0f) ? RightTileSensor(i, world, ctx.grid) : LeftTileSensor(i, world, ctx.grid);
    // Only snap up when approaching the slope's LOW side in its uphill direction -- same
    // direction check as GetHorizontalSensor. Without this, approaching from the tall/back
    // side (e.g. moving left into an UP_RIGHT slope) still snaps you onto the surface even
    // though the horizontal sensor now correctly walls you.
    bool climbableFromThisDirection = potentialSlope != -1 &&
        ((world.velX[i] > 0.0f && world.slopeType[potentialSlope] == SlopeType::UP_RIGHT) ||
         (world.velX[i] < 0.0f && world.slopeType[potentialSlope] == SlopeType::UP_LEFT));
    if (climbableFromThisDirection) {
        if (!jumping && world.velY[i] >= 0.0f) {
            float slopeSurfaceY = GetSlopeSurfaceY(potentialSlope, world.posX[i] + dx, world);
            float feetY = world.posY[i] + (world.size[i].y * 0.5f);

            if (slopeSurfaceY < feetY + 8.0f) {
                world.posY[i] = slopeSurfaceY - (world.size[i].y * 0.5f);
            }
        }
    }
}

CollisionConstraint PhysicsSystem::CheckSlopeSnap(int i, PhysicsContext& ctx, float dx) {
    CollisionConstraint c;
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];
    bool jumping = (slot != -1) && ctx.entities.isJumping[slot];

    int potentialSlope = (world.velX[i] > 0.0f) ? RightTileSensor(i, world, ctx.grid) : LeftTileSensor(i, world, ctx.grid);

    // Same uphill-direction check as HandleSlopeSnap/GetHorizontalSensor -- otherwise this
    // still snaps the entity onto the slope's surface even when approaching from its
    // tall/back side, undoing the wall block GetHorizontalSensor now correctly applies there.
    bool climbableFromThisDirection = potentialSlope != -1 &&
        ((world.velX[i] > 0.0f && world.slopeType[potentialSlope] == SlopeType::UP_RIGHT) ||
         (world.velX[i] < 0.0f && world.slopeType[potentialSlope] == SlopeType::UP_LEFT));
    if (climbableFromThisDirection) {
        if (!jumping && world.velY[i] >= 0.0f) {
            float slopeSurfaceY = GetSlopeSurfaceY(potentialSlope, world.posX[i] + dx, world);
            float feetY = world.posY[i] + (world.size[i].y * 0.5f);

            if (slopeSurfaceY < feetY + 8.0f) {
                c.resolve = true;
                c.deltaY = (slopeSurfaceY - (world.size[i].y * 0.5f)) - world.posY[i];
            }
        }
    }
    return c;
}

CollisionConstraint PhysicsSystem::CheckHorizontalCollision(int i, PhysicsContext& ctx, float dx) {
    CollisionConstraint c;
    PhysicsWorld& world = ctx.world;
    int hitID = GetHorizontalSensor(i, world, ctx.grid, dx);

    if (hitID == -1) return c;

    int floorID = TileBelowSensor(i, world, ctx.grid);
    // Direction-gated, same as GetHorizontalSensor/CheckSlopeSnap -- otherwise this branch
    // still lets the entity climb through a slope beneath its feet regardless of which way
    // it's moving, overriding the wall block GetHorizontalSensor already correctly applied
    // to `hitID` when approaching from the slope's tall/back side.
    bool adjacentToSlope = floorID != -1 &&
        ((dx > 0.0f && world.slopeType[floorID] == SlopeType::UP_RIGHT) ||
         (dx < 0.0f && world.slopeType[floorID] == SlopeType::UP_LEFT));

    if (world.isTile[hitID] && !adjacentToSlope) {
        bool movingInto = (dx > 0 && world.posX[i] < world.posX[hitID]) ||
            (dx < 0 && world.posX[i] > world.posX[hitID]);

        if (movingInto) {
            c.resolve = true;
            c.killVelX = true;
        }
    } else if (adjacentToSlope) {
        float floorY = GetSlopeSurfaceY(floorID, world.posX[i] + dx, world);
        float feetY = world.posY[i] + world.size[i].y * 0.5f;

        if (feetY >= floorY) {
            c.resolve = true;
            c.deltaY = (floorY - world.size[i].y * 0.5f) - world.posY[i];
            c.deltaX = dx;
        } else {
            c.resolve = true;
            c.killVelX = true;
        }
        return c;
    } else if (world.isEntity[hitID]) {
        float overlapY = (world.posY[i] + world.size[i].y * 0.5f)
            - (world.posY[hitID] - world.size[hitID].y * 0.5f);

        if (i < (int)world.hasCollisionEvent.size() && world.hasCollisionEvent[hitID]) {
            world.onCollision[hitID](i, hitID, world, dx, overlapY);
        }
    }
    return c;
}

CollisionConstraint PhysicsSystem::CheckGrounding(int i, PhysicsContext& ctx) {
    CollisionConstraint c;
    PhysicsWorld& world = ctx.world;
    int slot = world.entitySlot[i];

    int floorID = TileBelowSensor(i, world, ctx.grid);
    float feetY = world.posY[i] + (world.size[i].y * 0.5f);
    float epsilon = 0.5f;

    if (floorID != -1) {
        float floorY = (world.slopeType[floorID] != SlopeType::NONE)
            ? GetSlopeSurfaceY(floorID, world.posX[i], world)
            : (world.posY[floorID] - world.size[floorID].y * 0.5f);

        if (feetY >= (floorY - epsilon) && world.velY[i] >= 0.0f) {
            c.resolve = true;
            c.deltaY = (floorY - (world.size[i].y * 0.5f)) - world.posY[i];
            c.killVelY = true;
            world.velY[i] = 0.0f;

            if (slot != -1) {
                ctx.entities.isGrounded[slot] = true;
                ctx.entities.isJumping[slot] = false;
                ctx.entities.jumpsRemaining[slot] = 2;
            }

            if (world.isPlatform[floorID]) {
                if (slot != -1) ctx.entities.parentEntity[slot] = floorID;
                int pSlot = world.platformSlot[floorID];
                if (pSlot != -1) ctx.platforms.childEntity[pSlot] = i;
            }
        }
        return c;
    } else {
        if (slot != -1) {
            ctx.entities.isGrounded[slot] = false;
            if (ctx.entities.parentEntity[slot] != -1) {
                int oldParent = ctx.entities.parentEntity[slot];
                ctx.entities.parentEntity[slot] = -1;
                if (oldParent >= 0 && oldParent < (int)world.activeCount) {
                    int pSlot = world.platformSlot[oldParent];
                    if (pSlot != -1) ctx.platforms.childEntity[pSlot] = -1;
                }
            }
        }
    }
    return c;
}

CollisionConstraint PhysicsSystem::CheckCeilingCollision(int i, PhysicsContext& ctx) {
    PhysicsWorld& world = ctx.world;
    int ceilingID = TileAboveSensor(i, world, ctx.grid);
    CollisionConstraint result;
    if (ceilingID != -1 && world.velY[i] < 0.0f) {
        float ceilingY = world.posY[ceilingID] + (world.size[ceilingID].y * 0.5f);
        result.deltaY = (ceilingY + (world.size[i].y * 0.5f)) - world.posY[i];
        result.killVelY = true;
    }
    return result;
}

void PhysicsSystem::ApplyPhysics(int i, PhysicsContext& ctx, float dt) {
    PhysicsWorld& world = ctx.world;
    float dx = world.velX[i] * dt;

    CollisionConstraint horizontal = CheckHorizontalCollision(i, ctx, dx);
    CollisionConstraint slope = CheckSlopeSnap(i, ctx, dx);
    CollisionConstraint ground = CheckGrounding(i, ctx);
    CollisionConstraint ceiling = CheckCeilingCollision(i, ctx);

    int slot = world.entitySlot[i];
    int p = (slot != -1) ? ctx.entities.parentEntity[slot] : -1;
    bool onPlatform = (p != -1) && world.isPlatform[p];

    WallSlideCheck(i, ctx);
    if (onPlatform) {
        PlatformPhysics(i, ctx, dt);
    }

    if (horizontal.resolve) {
        int hitID = GetHorizontalSensor(i, world, ctx.grid, dx);
        bool jumping = (slot != -1) && ctx.entities.isJumping[slot];

        // Fourth spot that needed the same uphill-direction gate: this ran the "deflect along
        // the slope's normal" treatment for ANY slope hitID regardless of approach direction,
        // which takes priority over (and never falls through to) horizontal.killVelX below --
        // so even with GetHorizontalSensor/CheckHorizontalCollision correctly reporting a wall
        // hit for a wrong-direction approach, this still let the entity slide along the slope's
        // surface instead of actually stopping. Only take the slope-deflection path when the
        // hit is genuinely climbable from this direction; otherwise fall through to killVelX.
        bool climbableFromThisDirection = hitID != -1 && world.slopeType[hitID] != SlopeType::NONE &&
            ((dx > 0.0f && world.slopeType[hitID] == SlopeType::UP_RIGHT) ||
             (dx < 0.0f && world.slopeType[hitID] == SlopeType::UP_LEFT));

        if (climbableFromThisDirection && !jumping) {
            Vec2 normal = GetSlopeNormal(world.slopeType[hitID]);
            float dot = (world.velX[i] * normal.x) + (world.velY[i] * normal.y);

            if (dot < 0) {
                world.velX[i] -= dot * normal.x;
                world.velY[i] -= dot * normal.y;
            }
        } else if (horizontal.killVelX) {
            world.velX[i] = 0.0f;
        }
    }

    // PlatformPhysics already advanced posX by velX*dt itself (plus its own IsBlocked check)
    // when riding a platform -- applying the generic velX*dt integration again here as well
    // was double-moving the rider horizontally every frame, exactly why platform movement felt
    // much faster than walking on the ground.
    if (!onPlatform) {
        world.posX[i] += world.velX[i] * dt;
    }

    world.velY[i] += world.gravity[i] * dt;
    world.posY[i] += world.velY[i] * dt;

    if (slope.resolve) {
        world.posY[i] += slope.deltaY;
    }

    if (ground.resolve) {
        world.posY[i] += ground.deltaY;
        world.velY[i] = 0.0f;
        if (slot != -1) {
            ctx.entities.isGrounded[slot] = true;
            ctx.entities.isJumping[slot] = false;
        }

        // Ground friction -- decelerates horizontal velocity whenever nothing is actively
        // driving it this frame (moveIntent == 0): a player releasing A/D, an idle AI entity,
        // a resting particle (entitySlot == -1, so this is always "not driven"). GameplayScene
        // now accelerates the player's velX toward a target instead of hard-setting it, and
        // leaves velX alone entirely while moveIntent == 0 -- exactly the gap this fills.
        // world.friction[floorID] is a small per-tile coefficient (MapReader: .15 for solid
        // ground, .01 for a slippery moving platform), scaled up here into an actual px/s^2
        // deceleration -- using it directly as the decel rate would be imperceptible.
        bool activelyDriven = slot != -1 && ctx.entities.moveIntent[slot] != 0.0f;
        if (!activelyDriven) {
            int floorID = TileBelowSensor(i, world, ctx.grid);
            if (floorID != -1) {
                constexpr float kFrictionDecelScale = 2000.0f;
                float decel = world.friction[floorID] * kFrictionDecelScale * dt;
                if (std::fabs(world.velX[i]) <= decel) world.velX[i] = 0.0f;
                else world.velX[i] -= decel * (world.velX[i] > 0.0f ? 1.0f : -1.0f);
            }
        }
    }

    if (ceiling.resolve) {
        world.posY[i] += ceiling.deltaY;
        world.velY[i] = 0.0f;
    }

    if (slot != -1 && ctx.entities.isGrounded[slot]) {
        int floorID = TileBelowSensor(i, world, ctx.grid);
        if (floorID != -1 && world.slopeType[floorID] != SlopeType::NONE) {
            Vec2 normal = GetSlopeNormal(world.slopeType[floorID]);
            UpdateEntityRotation(i, world, normal);
        } else {
            UpdateEntityRotation(i, world, Vec2{ 0.0f, -1.0f });
        }
    }
}

void PhysicsSystem::ResolveParticle(int i, PhysicsContext& ctx) {
    PhysicsWorld& world = ctx.world;
    QueryScratchpad scratch;
    std::vector<int> neighbors;

    ctx.grid.query(i, world.posX[i], world.posY[i], world.size[i].x, world.size[i].y, neighbors, scratch);

    int slot = world.entitySlot[i];

    for (int otherID : neighbors) {
        if (i == otherID) continue;
        if ((world.layer[otherID] & world.mask[i]) == 0) continue;
        if (world.layer[otherID] == 0) continue;
        if (world.isGhosted[otherID]) continue;

        float dx = world.posX[i] - world.posX[otherID];
        float dy = world.posY[i] - world.posY[otherID];
        float combinedHW = (world.size[i].x + world.size[otherID].x) * 0.5f;
        float combinedHH = (world.size[i].y + world.size[otherID].y) * 0.5f;

        float overlapX = combinedHW - std::abs(dx);
        float overlapY = combinedHH - std::abs(dy);

        if (overlapX <= 0 || overlapY <= 0) continue;

        if (!world.isTile[otherID]) {
            // Non-tile hit (e.g. a bullet vs. an entity) -- no physical bounce/landing here,
            // PhysicsSystem doesn't know or care what a "hit" means gameplay-wise. Just fire
            // whichever side(s) registered a callback and let THEM sort it out (damage,
            // despawn, pickup, etc.) -- the original layer/mask + onCollision pattern.
            if (i < (int)world.hasCollisionEvent.size() && world.hasCollisionEvent[i]) {
                world.onCollision[i](i, otherID, world, dx, dy);
            }
            if (otherID < (int)world.hasCollisionEvent.size() && world.hasCollisionEvent[otherID]) {
                world.onCollision[otherID](otherID, i, world, -dx, -dy);
            }
            continue;
        }

        if (overlapX < overlapY) {
            world.posX[i] += (dx > 0) ? overlapX : -overlapX;

            if ((dx < 0 && world.velX[i] > 0.0f) || (dx > 0 && world.velX[i] < 0.0f)) {
                world.velX[i] *= -world.restitution[otherID];
                if (i < (int)world.hasCollisionEvent.size() && world.hasCollisionEvent[i]) {
                    world.onCollision[i](i, otherID, world, dx, dy);
                }
            }
        } else {
            // ONE-WAY LOGIC: only resolve if we are above the object and moving down.
            bool onTop = (world.posY[i] < world.posY[otherID]);
            int otherPlatformSlot = world.platformSlot[otherID];

            if (onTop && world.velY[i] >= 0.0f) {
                world.posY[i] = (world.posY[otherID] - (world.size[otherID].y * 0.5f)) - (world.size[i].y * 0.5f);
                world.velY[i] = 0.0f;
                if (slot != -1) {
                    ctx.entities.isGrounded[slot] = true;
                    ctx.entities.parentEntity[slot] = otherID;
                }
                // Ported behavior change (see plan): the source project wrote
                // childEntity on ANY tile landed on, but every reader
                // (UpdateGroups/LinkGroups/UpdatePlatforms/PlatformPhysics) only
                // ever consulted it in an isPlatform-gated context -- so only
                // platforms need to track their riding child now.
                if (otherPlatformSlot != -1) {
                    ctx.platforms.childEntity[otherPlatformSlot] = i;
                }
            } else if (!onTop && world.velY[i] < 0.0f) {
                world.posY[i] += overlapY;
                world.velY[i] *= -world.restitution[otherID];
                if (slot != -1) ctx.entities.parentEntity[slot] = -1;
                if (otherPlatformSlot != -1) ctx.platforms.childEntity[otherPlatformSlot] = -1;
            }
        }
    }
}

void PhysicsSystem::SyncPrevPositions(PhysicsWorld& world) {
    for (size_t i = 0; i < world.activeCount; ++i) {
        world.prevPos[i] = Vec2{ world.posX[i], world.posY[i] };
    }
}

void PhysicsSystem::UpdateChunk(int start, int end, float dt, PhysicsContext& ctx, bool gridMovement) {
    PhysicsWorld& world = ctx.world;
    for (int i = start; i < end; ++i) {
        if (i >= (int)world.life.size()) break;
        if (world.life[i] <= 0.0f) continue;

        int slot = world.entitySlot[i];
        bool isPlayerControlled = (slot != -1) && ctx.entities.isPlayerControlled[slot];
        if (isPlayerControlled && !gridMovement) continue;

        if (world.life[i] > 0.0f && !world.isEntity[i] && !world.isTile[i]) {
            world.life[i] -= dt;
            if (world.life[i] <= 0.0f) world.life[i] = 0.0f;
        }

        if (slot != -1 && ctx.entities.gridMovement[slot]) {
            float tdx = ctx.entities.targetX[slot] - world.posX[i];
            float tdy = ctx.entities.targetY[slot] - world.posY[i];

            float speed = (ctx.entities.moveSpeed[slot] > 0.0f) ? ctx.entities.moveSpeed[slot] : 128.0f;
            float step = speed * dt;

            if (fabsf(tdx) <= step) world.posX[i] = ctx.entities.targetX[slot];
            else world.posX[i] += (tdx > 0 ? 1 : -1) * step;

            if (fabsf(tdy) <= step) world.posY[i] = ctx.entities.targetY[slot];
            else world.posY[i] += (tdy > 0 ? 1 : -1) * step;

            if (world.posX[i] == ctx.entities.targetX[slot] && world.posY[i] == ctx.entities.targetY[slot]) {
                ctx.entities.isMoving[slot] = false;
            }
        }

        // Platforms are entirely moved by UpdatePlatforms (called once per frame with its own
        // fixed step, driving velX/velY toward targetA/targetB) -- integrating velX/velY*dt
        // here TOO, using this call's (possibly different) dt, silently double-moved every
        // platform every frame. This is the actual cause of "the platform outruns the rider":
        // PlatformPhysics's carry only ever accounts for the ONE known movement from
        // UpdatePlatforms, completely unaware of this second, uncoordinated kick landing on
        // the platform (never the rider) a moment later in the same frame.
        if (!world.isPlatform[i]) {
            world.velY[i] += world.gravity[i] * dt;
            world.posX[i] += world.velX[i] * dt;
            world.posY[i] += world.velY[i] * dt;
        }
        world.age[i] += 1;
        PhysicsSystem::ResolveParticle(i, ctx);

        world.rotation[i] += world.angularVel[i] * dt;
        world.angularVel[i] *= 0.99f; // "air resistance" so spin doesn't last forever
    }
}

void PhysicsSystem::UpdateWorldParallel(PhysicsContext& ctx, float dt, JLib::TaskScheduler& sched,
                                        int chunkSize, bool gridMovement) {
    sched.ParallelFor(0, (int)ctx.world.activeCount, chunkSize,
        [&](int start, int end) {
            // Each chunk runs on a worker thread/fiber with no handler of its own -- an
            // uncaught exception there terminates the ENTIRE process (std::terminate),
            // not just this chunk. Catch, report, and drop the chunk instead.
            try {
                UpdateChunk(start, end, dt, ctx, gridMovement);
            } catch (const std::exception& e) {
                FILE* f = nullptr;
                fopen_s(&f, "physics2d_crash.log", "a");
                if (f) { fprintf(f, "[Physics2D] UpdateChunk(%d,%d) threw: %s\n", start, end, e.what()); fclose(f); }
            } catch (...) {
                FILE* f = nullptr;
                fopen_s(&f, "physics2d_crash.log", "a");
                if (f) { fprintf(f, "[Physics2D] UpdateChunk(%d,%d) threw a non-std exception\n", start, end); fclose(f); }
            }
        });
}

void PhysicsSystem::UpdateGroups(PhysicsContext& ctx, float dt) {
    PhysicsWorld& world = ctx.world;
    PlatformTable& plat = ctx.platforms;
    int active = (int)world.activeCount;

    for (int i = 0; i < active; ++i) {
        if (!world.isPlatform[i] || world.platformSlot[i] == -1) continue;
        int slotI = world.platformSlot[i];
        if (plat.parentID[slotI] != -1) continue;

        bool hasCollision = (plat.childEntity[slotI] != -1);
        for (int j = 0; j < active; ++j) {
            if (!world.isPlatform[j] || world.platformSlot[j] == -1) continue;
            int slotJ = world.platformSlot[j];
            if (plat.masterID[slotJ] >= 0 && plat.masterID[slotJ] < active && plat.masterID[slotJ] == i) {
                if (plat.childEntity[slotJ] != -1) {
                    hasCollision = true;
                    break;
                }
            }
        }

        if (hasCollision) {
            plat.timerStarted[slotI] = true;
            plat.trigger[slotI] = true;
        }

        for (int j = 0; j < active; ++j) {
            if (!world.isPlatform[j] || world.platformSlot[j] == -1) continue;
            int slotJ = world.platformSlot[j];
            if (plat.masterID[slotJ] >= 0 && plat.masterID[slotJ] < active && plat.masterID[slotJ] == i) {
                plat.timerStarted[slotJ] = plat.timerStarted[slotI];
                plat.trigger[slotJ] = plat.trigger[slotI];
            }
        }
    }
}

void PhysicsSystem::LinkGroups(PhysicsContext& ctx) {
    PhysicsWorld& world = ctx.world;
    PlatformTable& plat = ctx.platforms;

    // Master = the member closest to the group's own centroid, not just whichever one happens
    // to be spawned/iterated first (for a straight row that would always be one end-piece).
    // A centered master gives a group's own targetA/targetB (defined relative to whichever
    // tile is master -- see MapReader) a path centered on the platform's visual middle, and
    // keeps the master/slave relativeOffset split more even instead of all-on-one-side.
    std::unordered_map<int, std::vector<int>> groupMembers;
    std::unordered_map<int, Vec2> groupPosSum;
    for (int i = 0; i < (int)world.activeCount; ++i) {
        if (!world.isPlatform[i] || world.platformSlot[i] == -1) continue;
        int gID = plat.groupID[world.platformSlot[i]];
        groupMembers[gID].push_back(i);
        groupPosSum[gID].x += world.posX[i];
        groupPosSum[gID].y += world.posY[i];
    }

    std::unordered_map<int, int> groupToMaster;
    for (auto& entry : groupMembers) {
        int gID = entry.first;
        std::vector<int>& members = entry.second;
        Vec2 centroid = groupPosSum[gID];
        centroid.x /= (float)members.size();
        centroid.y /= (float)members.size();

        int best = members[0];
        float bestDistSq = -1.0f;
        for (int m : members) {
            float dx = world.posX[m] - centroid.x;
            float dy = world.posY[m] - centroid.y;
            float distSq = dx * dx + dy * dy;
            if (bestDistSq < 0.0f || distSq < bestDistSq) {
                bestDistSq = distSq;
                best = m;
            }
        }
        groupToMaster[gID] = best;
        plat.parentID[world.platformSlot[best]] = -1;
    }

    for (int i = 0; i < (int)world.activeCount; ++i) {
        if (!world.isPlatform[i] || world.platformSlot[i] == -1) continue;
        int slotI = world.platformSlot[i];
        int gID = plat.groupID[slotI];
        int masterIdx = groupToMaster[gID];

        if (i != masterIdx) {
            plat.parentID[slotI] = masterIdx;
            plat.masterID[slotI] = masterIdx;
            plat.relativeOffsetX[slotI] = world.posX[i] - world.posX[masterIdx];
            plat.relativeOffsetY[slotI] = world.posY[i] - world.posY[masterIdx];
        } else {
            plat.masterID[slotI] = i;
        }
    }
}

void PhysicsSystem::UpdatePlatforms(PhysicsContext& ctx, float dt) {
    PhysicsWorld& world = ctx.world;
    PlatformTable& plat = ctx.platforms;

    for (int i = 0; i < (int)world.activeCount; ++i) {
        if (!world.isPlatform[i] || world.platformSlot[i] == -1) continue;
        int slotI = world.platformSlot[i];
        if (plat.parentID[slotI] != -1) continue; // Skip Slaves (they stay dormant)

        // --- MASTER BRAIN ---
        bool groupTouched = (plat.childEntity[slotI] != -1);
        for (int j = 0; j < (int)world.activeCount; ++j) {
            if (!world.isPlatform[j] || world.platformSlot[j] == -1) continue;
            int slotJ = world.platformSlot[j];
            if (plat.masterID[slotJ] == i && plat.childEntity[slotJ] != -1) {
                groupTouched = true;
                break;
            }
        }

        int originalChild = plat.childEntity[slotI];
        if (groupTouched) plat.childEntity[slotI] = 1; // Lie to the code so it thinks it's touched

        if (plat.canFall[slotI]) {
            if (plat.childEntity[slotI] != -1 || plat.timerStarted[slotI]) {
                plat.timerStarted[slotI] = true;
                plat.platformTimer[slotI] -= dt;
                if (plat.platformTimer[slotI] < 1.0f && plat.platformState[slotI] == 0) {
                    plat.platformState[slotI] = 1;
                    plat.shakeTimer[slotI] = 1.0f;
                }
                if (plat.platformState[slotI] == 1) { // SHAKING
                    plat.shakeTimer[slotI] -= dt;
                    float offset = (std::sin(plat.shakeTimer[slotI] * 50.0f)) * 2.0f;
                    world.posX[i] += offset;

                    if (plat.shakeTimer[slotI] <= 0.0f) {
                        plat.platformState[slotI] = 2; // FALLING
                    }
                }
            }

            if (plat.platformState[slotI] == 2) {
                world.posY[i] += 500.0f * dt;

                // FORCE DETACH: if a platform falls, nothing can stay on it
                if (plat.childEntity[slotI] != -1) {
                    int child = plat.childEntity[slotI];
                    int childSlot = world.entitySlot[child];
                    if (childSlot != -1) ctx.entities.parentEntity[childSlot] = -1;
                    plat.childEntity[slotI] = -1;
                }
            }
        }

        bool shouldMove = false;
        if (plat.isTriggered[slotI]) {
            shouldMove = (plat.trigger[slotI] || plat.childEntity[slotI] != -1);
        } else {
            shouldMove = (plat.childEntity[slotI] != -1);
        }

        if (shouldMove) {
            Vec2 target = plat.movingForward[slotI] ? plat.targetB[slotI] : plat.targetA[slotI];
            float dx = target.x - world.posX[i];
            float dy = target.y - world.posY[i];
            float dist = sqrtf(dx * dx + dy * dy);
            float step = plat.moveSpeed[slotI] * dt;

            if (dist <= step) {
                world.posX[i] = target.x;
                world.posY[i] = target.y;
                world.velX[i] = 0.0f;
                world.velY[i] = 0.0f;
                plat.movingForward[slotI] = !plat.movingForward[slotI];
            } else {
                world.velX[i] = (dx / dist) * plat.moveSpeed[slotI];
                world.velY[i] = (dy / dist) * plat.moveSpeed[slotI];
                world.posX[i] += world.velX[i] * dt;
                world.posY[i] += world.velY[i] * dt;
            }
        }

        // RESTORE: put the original value back so we don't mess up other systems
        plat.childEntity[slotI] = originalChild;

        // BROADCAST: sync the slaves
        for (int j = 0; j < (int)world.activeCount; ++j) {
            if (!world.isPlatform[j] || world.platformSlot[j] == -1 || j == i) continue;
            int slotJ = world.platformSlot[j];
            if (plat.masterID[slotJ] == i) {
                world.posX[j] = world.posX[i] + plat.relativeOffsetX[slotJ];
                world.posY[j] = world.posY[i] + plat.relativeOffsetY[slotJ];
                plat.platformState[slotJ] = plat.platformState[slotI];
                plat.timerStarted[slotJ] = plat.timerStarted[slotI];
            }
        }
    }
}

} // namespace Physics2D
