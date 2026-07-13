// Throwaway smoke test for the Physics2D port -- not deployed, not part of the
// library. Links Physics2D.lib + Threads.lib directly. See the port plan for context.
#include <cstdio>
#include <cassert>
#include "PhysicsWorld.h"
#include "SpatialGrid.h"
#include "PhysicsSystem.h"
#include <TaskScheduler.h>

using namespace Physics2D;

// NOTE (Stage 3): there is no RebuildGrid helper anymore -- world.commit(&grid) IS the
// grid build now (fused with compaction + cell-sort). Because the sort reorders rows,
// every test that holds a reference across a commit resolves it through a Handle.

static void TestLanding() {
    PhysicsWorld world(64, 8, 8, 8);
    SpatialGrid grid(1920, 1080, 64, 64);

    int groundIdx = world.spawn(ShapeType::RECT, Vec2{ 400.0f, 32.0f }, 640.0f, 700.0f, 0.0f, 0.0f, 1.0f,
                                 0.0f, LAYER_PLAYER, LAYER_PLAYER, false, false);
    world.isTile[groundIdx] = 1;
    world.health[groundIdx] = 255;

    int playerIdx = world.spawn(ShapeType::RECT, Vec2{ 32.0f, 48.0f }, 640.0f, 500.0f, 0.0f, 0.0f, 1.0f,
                                 900.0f, LAYER_PLAYER, LAYER_PLAYER, true, false);
    assert(playerIdx != -1 && groundIdx != -1);
    assert(world.entitySlot[playerIdx] != -1);
    Handle hPlayer = world.handleOf(playerIdx);

    PhysicsContext ctx{ world, grid, world.entities, world.platforms };

    const float dt = 1.0f / 60.0f;
    bool landed = false;
    int p = -1;
    for (int frame = 0; frame < 180; ++frame) {
        world.commit(&grid);              // compact + sort + grid build in one pass
        p = world.resolve(hPlayer);       // sort moved the rows -- re-resolve
        assert(p != -1);
        PhysicsSystem::ApplyPhysics(p, ctx, dt);

        int slot = world.entitySlot[p];
        if (slot != -1 && world.entities.isGrounded[slot]) { landed = true; break; }
    }

    int slot = world.entitySlot[p];
    printf("[TestLanding] landed=%d posY=%.2f isGrounded=%d\n", landed, world.posY[p],
           slot != -1 ? (int)world.entities.isGrounded[slot] : -1);
    assert(landed);
}

static void TestSlotReuse() {
    PhysicsWorld world(16, 4, 4, 4);
    int e1 = world.spawn(ShapeType::RECT, Vec2{ 8,8 }, 0,0,0,0, 1.0f, 0,0,0, true, false);
    int e2 = world.spawn(ShapeType::RECT, Vec2{ 8,8 }, 0,0,0,0, 1.0f, 0,0,0, true, false);
    int e3 = world.spawn(ShapeType::RECT, Vec2{ 8,8 }, 0,0,0,0, 1.0f, 0,0,0, true, false);
    assert(e1 != -1 && e2 != -1 && e3 != -1);

    int freedSlot = world.entitySlot[e2];
    world.life[e2] = -1.0f; // kill the middle one
    world.compressArrays();
    assert(world.activeCount == 2);

    int e4 = world.spawn(ShapeType::RECT, Vec2{ 8,8 }, 0,0,0,0, 1.0f, 0,0,0, true, false);
    assert(e4 != -1);
    assert(world.entitySlot[e4] == freedSlot); // freelist is LIFO -- must hand the just-freed slot back
    printf("[TestSlotReuse] OK -- freed slot %d reused, activeCount=%zu\n", freedSlot, world.activeCount);
}

// Regression test for a real bug hit in Game01: a ground tile wider than the SpatialGrid's
// nominal size (the grid is sized off screen/world width, not the tile's own extent), with the
// player walked far enough right that its sensor queries fall outside the grid's nominal columns
// entirely. Before the fix, SpatialGrid::query()/insert() only floor-clamped minX and ceiling-
// clamped maxX (one-sided), so once BOTH ends of a query rect landed past the grid's right edge,
// minX stayed unclamped-upward while maxX clamped down, producing minX > maxX -- an empty loop,
// i.e. collision silently vanishing well before the tile's actual edge.
static void TestWideGroundBeyondGridBounds() {
    PhysicsWorld world(64, 8, 8, 8);
    // Grid nominally sized for a 640-wide world -- deliberately much narrower than the ground.
    SpatialGrid grid(640, 1080, 64, 64);

    int groundIdx = world.spawn(ShapeType::RECT, Vec2{ 4000.0f, 40.0f }, 2000.0f, 700.0f, 0.0f, 0.0f, 1.0f,
                                 0.0f, LAYER_PLAYER, LAYER_PLAYER, false, false);
    world.isTile[groundIdx] = 1;
    world.health[groundIdx] = 255;

    // Spawned far to the right -- well past the grid's nominal [0, 640) column range.
    int playerIdx = world.spawn(ShapeType::RECT, Vec2{ 32.0f, 48.0f }, 3000.0f, 500.0f, 0.0f, 0.0f, 1.0f,
                                 900.0f, LAYER_PLAYER, LAYER_PLAYER, true, false);
    assert(playerIdx != -1 && groundIdx != -1);

    Handle hPlayer = world.handleOf(playerIdx);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };

    const float dt = 1.0f / 60.0f;
    bool landed = false;
    int p = -1;
    for (int frame = 0; frame < 180; ++frame) {
        world.commit(&grid);
        p = world.resolve(hPlayer);
        assert(p != -1);
        PhysicsSystem::ApplyPhysics(p, ctx, dt);

        int slot = world.entitySlot[p];
        if (slot != -1 && world.entities.isGrounded[slot]) { landed = true; break; }
    }

    printf("[TestWideGroundBeyondGridBounds] landed=%d posY=%.2f\n", landed, world.posY[p]);
    assert(landed); // would fail pre-fix: the player falls straight through
}

// Regression test: UpdateChunk must never move a platform via its generic velX/velY*dt
// integration -- UpdatePlatforms owns platform movement entirely (its own fixed step, driven
// toward targetA/targetB), and this same integration running a second time inside UpdateChunk
// (with whatever dt THAT call happens to use) was silently double-moving every platform every
// frame. PlatformPhysics's rider-carry only ever accounts for the ONE known UpdatePlatforms
// movement, so the platform visibly outran the rider it was supposed to be carrying.
static void TestUpdateChunkDoesNotMovePlatforms() {
    PhysicsWorld world(8, 4, 4, 4);
    SpatialGrid grid(1024, 1024, 64, 8);

    int platIdx = world.spawn(ShapeType::RECT, Vec2{ 32.0f, 16.0f }, 100.0f, 100.0f, 64.0f, 0.0f, 1.0f,
                               0.0f, LAYER_PLAYER, LAYER_PLAYER, false, true, false);
    Handle hPlat = world.handleOf(platIdx);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };
    world.commit(&grid);
    int p = world.resolve(hPlat);
    assert(p != -1);

    float before = world.posX[p];
    PhysicsSystem::UpdateChunk(0, (int)world.activeCount, 1.0f / 60.0f, ctx);

    printf("[TestUpdateChunkDoesNotMovePlatforms] before=%.4f after=%.4f\n", before, world.posX[p]);
    assert(world.posX[p] == before); // UpdateChunk must leave platform position untouched
}

// Regression test for the slope-direction collision bug (took 4 separate fixes to fully pin
// down: GetHorizontalSensor, CheckSlopeSnap/HandleSlopeSnap, CheckHorizontalCollision's
// adjacentToSlope, and ApplyPhysics's own slope-normal-deflection branch all independently
// let an entity approaching an UP_RIGHT slope's tall/back side from the right, moving left,
// pass straight through instead of being blocked like a wall).
static void TestSlopeBlocksFromWrongDirection() {
    PhysicsWorld world(16, 4, 4, 4);
    SpatialGrid grid(1024, 1024, 64, 16);

    int slopeIdx = world.spawn(ShapeType::RECT, Vec2{ 32.0f, 32.0f }, 500.0f, 100.0f, 0.0f, 0.0f, 1.0f,
                                0.0f, LAYER_PLAYER, LAYER_PLAYER, false, false, false);
    world.isTile[slopeIdx] = 1;
    world.isSlope[slopeIdx] = 1;
    world.slopeType[slopeIdx] = SlopeType::UP_RIGHT; // high side is on the right

    // Approaching from the RIGHT (the tall/back side for UP_RIGHT), moving LEFT -- must wall.
    int actorIdx = world.spawn(ShapeType::RECT, Vec2{ 16.0f, 16.0f }, 540.0f, 100.0f, -80.0f, 0.0f, 1.0f,
                                0.0f, LAYER_PLAYER, LAYER_PLAYER, true, false, false);
    Handle hSlope = world.handleOf(slopeIdx);
    Handle hActor = world.handleOf(actorIdx);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };

    const float dt = 1.0f / 60.0f;
    int a = -1;
    for (int frame = 0; frame < 120; ++frame) {
        world.commit(&grid);
        a = world.resolve(hActor);
        assert(a != -1);
        PhysicsSystem::ApplyPhysics(a, ctx, dt);
    }

    int s = world.resolve(hSlope);
    printf("[TestSlopeBlocksFromWrongDirection] final posX=%.2f (slope center at %.2f)\n",
           world.posX[a], world.posX[s]);
    assert(world.posX[a] > world.posX[s]); // must NOT tunnel past the slope's center
}

// Exercises the new ParticleTable: a bullet particle carries a game-defined type tag +
// damage payload, and its onCollision callback (fired by ResolveParticle's non-tile branch)
// reads them back to apply damage -- Physics2D itself never interprets particleType/damage,
// it just stores them and dispatches the callback.
static void TestParticleTypeCollision() {
    enum GameParticleType { PT_NONE = 0, PT_PLAYER_BULLET = 1 };

    PhysicsWorld world(16, 4, 4, 4);
    SpatialGrid grid(1024, 1024, 64, 16);

    int enemyIdx = world.spawn(ShapeType::RECT, Vec2{ 32,32 }, 100.0f, 100.0f, 0.0f, 0.0f, 1.0f,
                                0.0f, LAYER_ENEMY, LAYER_PLAYER_BULLET, false, false, false);
    int bulletIdx = world.spawn(ShapeType::RECT, Vec2{ 8,8 }, 100.0f, 100.0f, 0.0f, 0.0f, 1.0f,
                                 0.0f, LAYER_PLAYER_BULLET, LAYER_ENEMY, false, false, true);
    assert(enemyIdx != -1 && bulletIdx != -1);

    int particleSlot = world.particleSlot[bulletIdx];
    assert(particleSlot != -1);
    world.particles.particleType[particleSlot] = PT_PLAYER_BULLET;
    world.particles.damage[particleSlot] = 25.0f;

    float appliedDamage = 0.0f;
    world.hasCollisionEvent[bulletIdx] = 1;
    world.onCollision[bulletIdx] = [&appliedDamage](int self, int /*other*/, PhysicsWorld& w, float, float) {
        int slot = w.particleSlot[self];
        if (slot != -1 && w.particles.particleType[slot] == PT_PLAYER_BULLET) {
            appliedDamage = w.particles.damage[slot];
        }
    };

    Handle hBullet = world.handleOf(bulletIdx);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };
    world.commit(&grid);
    int b = world.resolve(hBullet);
    assert(b != -1);
    PhysicsSystem::ResolveParticle(b, ctx);

    printf("[TestParticleTypeCollision] appliedDamage=%.2f\n", appliedDamage);
    assert(appliedDamage == 25.0f);
}

static void TestParallelDispatch() {
    JLib::TaskScheduler::Init(0); // 0 = auto-detect
    auto& sched = JLib::TaskScheduler::Instance();

    const int N = 2000;
    PhysicsWorld world(N, 64, 16, 64);
    SpatialGrid grid(4096, 4096, 64, N);

    for (int i = 0; i < N; ++i) {
        bool isEnt = (i % 50 == 0);
        world.spawn(ShapeType::RECT, Vec2{ 8,8 }, (float)(i % 64) * 64.0f, (float)(i / 64) * 64.0f,
                    0.0f, 0.0f, 5.0f, 100.0f, LAYER_PLAYER, LAYER_PLAYER, isEnt, false);
    }

    PhysicsContext ctx{ world, grid, world.entities, world.platforms };
    world.commit(&grid);
    PhysicsSystem::UpdateWorldParallel(ctx, 1.0f / 60.0f, sched, 64, false);
    world.commit(&grid);

    printf("[TestParallelDispatch] OK -- ParallelFor ran over %d cells, activeCount now %zu\n", N, world.activeCount);
}

// Exercises the Stage-2 deferred-spawn path: many worker threads call requestSpawn()
// concurrently (per-thread lanes, no locks), nothing is visible until the single-threaded
// commit(), and commit() stamps the ParticleTable payload the way a direct spawner would
// through spawn()'s returned index. Must run AFTER TestParallelDispatch (scheduler Init).
// Note kRequests > 10000 on purpose: ParallelFor runs serial at or below 10k items, so a
// smaller count would never actually test cross-thread lane writes.
static void TestDeferredSpawnCommit() {
    auto& sched = JLib::TaskScheduler::Instance();

    const int kRequests = 20000;
    PhysicsWorld world(24000, 8, 8, 24000);

    sched.ParallelFor(0, kRequests, 64, [&](int s, int e) {
        for (int i = s; i < e; ++i) {
            SpawnRequest req;
            req.shape = ShapeType::CIRCLE;
            req.size = Vec2{ 4.0f, 4.0f };
            req.x = (float)i;
            req.lifetime = 1.0f;
            req.isParticle = true;
            req.particleType = 7;
            req.damage = 3.0f;
            req.sourceEntity = 42;
            world.requestSpawn(req);
        }
    });

    assert(world.activeCount == 0); // nothing published before commit

    world.commit();
    assert(world.activeCount == (size_t)kRequests);

    for (size_t i = 0; i < world.activeCount; ++i) {
        int slot = world.particleSlot[i];
        assert(slot != -1);
        assert(world.particles.particleType[slot] == 7);
        assert(world.particles.damage[slot] == 3.0f);
        assert(world.particles.sourceEntity[slot] == 42);
    }

    printf("[TestDeferredSpawnCommit] OK -- %d worker-side requests, 0 visible pre-commit, %zu after\n",
           kRequests, world.activeCount);
}

// Locks down the Stage-3 fused commit itself: after commit(&grid), (1) every Handle still
// resolves to its row (position payload intact through the scatter), (2) the rows are
// physically bin-contiguous and cellStart/cellCount describe them exactly, (3) a killed
// row's handle goes stale (generation bump) instead of resolving to a recycled row.
static void TestCellSortHandles() {
    constexpr int kRows = 100;
    PhysicsWorld world(256, 8, 8, 256);
    SpatialGrid grid(1024, 1024, 64, 256);

    Handle h[kRows];
    float ex[kRows], ey[kRows];
    for (int i = 0; i < kRows; ++i) {
        ex[i] = (float)((i * 37) % 1024);
        ey[i] = (float)((i * 91) % 1024);
        int idx = world.spawn(ShapeType::CIRCLE, Vec2{ 8, 8 }, ex[i], ey[i], 0, 0, 1.0f,
                              0.0f, LAYER_PLAYER, LAYER_PLAYER, false, false, true);
        assert(idx != -1);
        h[i] = world.handleOf(idx);
    }

    world.commit(&grid);

    // (1) handles track rows through the sort, payload intact
    for (int i = 0; i < kRows; ++i) {
        int idx = world.resolve(h[i]);
        assert(idx != -1);
        assert(world.posX[idx] == ex[i] && world.posY[idx] == ey[i]);
    }

    // (2) physical bin-contiguity: every row sits inside its own bin's advertised run
    for (size_t r = 0; r < world.activeCount; ++r) {
        int b = grid.binOf(world.posX[r], world.posY[r], world.size[r].x, world.size[r].y);
        assert((int)r >= grid.cellStart[b] && (int)r < grid.cellStart[b] + grid.cellCount[b]);
    }

    // (3) table-stored row references survive the sort: an entity riding a platform
    // (EntityTable::parentEntity holds the platform's ROW index) must still point at
    // the platform's NEW row after a commit reorders everything.
    int riderIdx = world.spawn(ShapeType::RECT, Vec2{ 16, 16 }, 500.0f, 500.0f, 0, 0, 1.0f,
                               0.0f, LAYER_PLAYER, LAYER_PLAYER, true, false, false);
    int platIdx = world.spawn(ShapeType::RECT, Vec2{ 32, 16 }, 900.0f, 900.0f, 0, 0, 1.0f,
                              0.0f, LAYER_PLAYER, LAYER_PLAYER, false, true, false);
    assert(riderIdx != -1 && platIdx != -1);
    Handle hRider = world.handleOf(riderIdx);
    Handle hPlat = world.handleOf(platIdx);
    world.entities.parentEntity[world.entitySlot[riderIdx]] = platIdx;
    world.commit(&grid);
    {
        int r2 = world.resolve(hRider);
        int p2 = world.resolve(hPlat);
        assert(r2 != -1 && p2 != -1);
        assert(world.entities.parentEntity[world.entitySlot[r2]] == p2); // remapped, not stale
    }

    // (4) kill one, commit, handle must go stale -- not resolve to some recycled row
    int victim = world.resolve(h[7]);
    world.life[victim] = -1.0f;
    world.commit(&grid);
    assert(world.resolve(h[7]) == -1);
    assert(world.activeCount == kRows + 1); // kRows - 1 particles + rider + platform
    for (int i = 0; i < kRows; ++i) {
        if (i == 7) continue;
        assert(world.resolve(h[i]) != -1); // survivors still tracked
    }

    printf("[TestCellSortHandles] OK -- %d rows sorted bin-contiguous, handles tracked, stale handle caught\n", kRows);
}

// Verifies UpdateChunk's SIMD pass against scalar reference math, exercising every mask:
// normal particles integrate + tick life + spin; a ghosted row is fully inert; a platform
// spins but does NOT integrate position/velocity. Row count deliberately not a multiple
// of 8 so the scalar tail path runs too. Grid is left un-committed (all-zero runs), so
// ResolveParticle sees no neighbors and pure integration is isolated.
static void TestChunkIntegrationSIMD() {
    constexpr int kParticles = 21; // 2 SIMD blocks + a 5-row scalar tail (after +2 rows below)
    constexpr float dt = 0.5f;
    PhysicsWorld world(64, 8, 8, 32);
    SpatialGrid grid(1024, 1024, 64, 64);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };

    for (int i = 0; i < kParticles; ++i) {
        int idx = world.spawn(ShapeType::CIRCLE, Vec2{ 4, 4 }, 10.0f * i, 0.0f, 5.0f, 2.0f, 1.0f,
                              100.0f, LAYER_PLAYER, LAYER_PLAYER, false, false, true);
        assert(idx != -1);
        world.angularVel[idx] = 2.0f;
    }
    int ghostIdx = world.spawn(ShapeType::CIRCLE, Vec2{ 4, 4 }, 500.0f, 0.0f, 5.0f, 2.0f, 1.0f,
                               100.0f, LAYER_PLAYER, LAYER_PLAYER, false, false, true);
    world.isGhosted[ghostIdx] = 1;
    world.angularVel[ghostIdx] = 2.0f;
    int platIdx = world.spawn(ShapeType::RECT, Vec2{ 32, 16 }, 600.0f, 0.0f, 40.0f, 0.0f, 1.0f,
                              0.0f, LAYER_PLAYER, LAYER_PLAYER, false, true, false);
    world.angularVel[platIdx] = 2.0f;

    PhysicsSystem::UpdateChunk(0, (int)world.activeCount, dt, ctx);

    const float eps = 1e-3f;
    for (int i = 0; i < kParticles; ++i) {
        // scalar reference: velY = 2 + 100*0.5 = 52; posX = 10i + 5*0.5; posY = 0 + 52*0.5
        assert(std::fabs(world.velY[i] - 52.0f) < eps);
        assert(std::fabs(world.posX[i] - (10.0f * i + 2.5f)) < eps);
        assert(std::fabs(world.posY[i] - 26.0f) < eps);
        assert(std::fabs(world.life[i] - 0.5f) < eps);          // ticked down (particle)
        assert(std::fabs(world.rotation[i] - 1.0f) < eps);      // 2.0 * 0.5
        assert(std::fabs(world.angularVel[i] - 1.98f) < eps);   // drag
        assert(world.age[i] == 1);
    }
    // ghosted: fully inert on every column
    assert(world.posX[ghostIdx] == 500.0f && world.posY[ghostIdx] == 0.0f);
    assert(world.velY[ghostIdx] == 2.0f && world.rotation[ghostIdx] == 0.0f);
    assert(world.age[ghostIdx] == 0);
    // platform: spins (matches old behavior) but never integrates position/velocity
    assert(world.posX[platIdx] == 600.0f && world.velX[platIdx] == 40.0f);
    assert(std::fabs(world.rotation[platIdx] - 1.0f) < eps);
    assert(world.age[platIdx] == 1);
    // life ticks for anything !isEntity && !isTile -- which includes a bare platform row
    // (matches the ORIGINAL UpdateChunk guard; Game01's map platforms are isTile so they
    // don't tick there, but this test platform isn't).
    assert(std::fabs(world.life[platIdx] - 0.5f) < eps);

    printf("[TestChunkIntegrationSIMD] OK -- %d particles + ghost + platform, SIMD matches scalar reference\n",
           kParticles);
}

// Forces a real hit through ResolveParticle's 8-wide prefilter path (not just the scalar
// run tail): 12+ rows in ONE cell so the run exceeds a SIMD block, with the only matching
// target (the enemy) sitting at lane 3 INSIDE the first block, surrounded by dummies the
// layer/mask lanes must reject. The counting sort is stable, so run order == spawn order.
static void TestResolveParticleSIMDPath() {
    enum { PT_TEST_BULLET = 9 };
    PhysicsWorld world(64, 8, 8, 32);
    SpatialGrid grid(1024, 1024, 64, 64);
    PhysicsContext ctx{ world, grid, world.entities, world.platforms };

    // All rows placed inside cell (0,0) -- positions within [0, 64).
    auto spawnDummy = [&]() {
        // layer = PLAYER_BULLET, so (layer & bulletMask=LAYER_ENEMY) == 0 -> lane rejected
        int d = world.spawn(ShapeType::RECT, Vec2{ 8, 8 }, 20.0f, 20.0f, 0, 0, 1.0f,
                            0.0f, LAYER_PLAYER_BULLET, 0, false, false, false);
        assert(d != -1);
    };
    spawnDummy(); spawnDummy(); spawnDummy();               // lanes 0-2
    int enemyIdx = world.spawn(ShapeType::RECT, Vec2{ 32, 32 }, 20.0f, 20.0f, 0, 0, 1.0f,
                               0.0f, LAYER_ENEMY, LAYER_PLAYER_BULLET, false, false, false); // lane 3
    for (int n = 0; n < 8; ++n) spawnDummy();               // lanes 4-11
    int bulletIdx = world.spawn(ShapeType::RECT, Vec2{ 8, 8 }, 20.0f, 20.0f, 0, 0, 1.0f,
                                0.0f, LAYER_PLAYER_BULLET, LAYER_ENEMY, false, false, true);
    assert(enemyIdx != -1 && bulletIdx != -1);

    int pSlot = world.particleSlot[bulletIdx];
    world.particles.particleType[pSlot] = PT_TEST_BULLET;
    world.particles.damage[pSlot] = 7.0f;

    float applied = 0.0f;
    world.hasCollisionEvent[bulletIdx] = 1;
    world.onCollision[bulletIdx] = [&applied](int self, int /*other*/, PhysicsWorld& w, float, float) {
        int s = w.particleSlot[self];
        if (s != -1 && w.particles.particleType[s] == PT_TEST_BULLET)
            applied = w.particles.damage[s];
    };

    Handle hBullet = world.handleOf(bulletIdx);
    world.commit(&grid);
    int b = world.resolve(hBullet);
    assert(b != -1);
    PhysicsSystem::ResolveParticle(b, ctx);

    printf("[TestResolveParticleSIMDPath] applied=%.2f (enemy at SIMD lane 3 of a 13-row run)\n", applied);
    assert(applied == 7.0f);
}

int main() {
    TestLanding();
    TestSlotReuse();
    TestWideGroundBeyondGridBounds();
    TestUpdateChunkDoesNotMovePlatforms();
    TestSlopeBlocksFromWrongDirection();
    TestParticleTypeCollision();
    TestCellSortHandles();
    TestChunkIntegrationSIMD();
    TestResolveParticleSIMDPath();
    TestParallelDispatch();
    TestDeferredSpawnCommit();
    printf("ALL SMOKE TESTS PASSED\n");
    return 0;
}
