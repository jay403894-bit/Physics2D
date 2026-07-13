#include "../include/PhysicsWorld.h"

namespace Physics2D {

void PhysicsWorld::reset() {
    gravity.clear();
    posX.clear();
    posY.clear();
    velX.clear();
    velY.clear();
    life.clear();
    size.clear();
    age.clear();
    shapeType.clear();
    rotation.clear();
    mask.clear();
    layer.clear();
    isTile.clear();
    health.clear();
    isEntity.clear();
    friction.clear();
    isParticle.clear();
    restitution.clear();
    angularVel.clear();
    onCollision.clear();
    hasCollisionEvent.clear();
    isPlatform.clear();
    prevPos.clear();
    isGhosted.clear();
    isSlope.clear();
    slopeType.clear();
    tileID.clear();
    entitySlot.clear();
    platformSlot.clear();
    particleSlot.clear();
    emitterSlot.clear();

    gravity.resize(maxEntities, 0.0f);
    posX.resize(maxEntities, 0.0f);
    posY.resize(maxEntities, 0.0f);
    velX.resize(maxEntities, 0.0f);
    velY.resize(maxEntities, 0.0f);
    life.resize(maxEntities, 0.0f);
    size.resize(maxEntities, Vec2{ 0.0f, 0.0f });
    age.resize(maxEntities, 0);
    shapeType.resize(maxEntities, ShapeType::RECT);
    rotation.resize(maxEntities, 0.0f);
    mask.resize(maxEntities, 0);
    layer.resize(maxEntities, 0);
    isTile.resize(maxEntities, 0);
    health.resize(maxEntities, 0);
    isEntity.resize(maxEntities, 0);
    friction.resize(maxEntities, 0.0f);
    isParticle.resize(maxEntities, 0);
    restitution.resize(maxEntities, 0.0f);
    angularVel.resize(maxEntities, 0.0f);
    onCollision.resize(maxEntities, nullptr);
    hasCollisionEvent.resize(maxEntities, 0);
    isPlatform.resize(maxEntities, 0);
    prevPos.resize(maxEntities, Vec2{ 0.0f, 0.0f });
    isGhosted.resize(maxEntities, 0);
    isSlope.resize(maxEntities, 0);
    slopeType.resize(maxEntities, SlopeType::NONE);
    tileID.resize(maxEntities, 0);
    entitySlot.resize(maxEntities, -1);
    platformSlot.resize(maxEntities, -1);
    particleSlot.resize(maxEntities, -1);
    emitterSlot.resize(maxEntities, -1);

    activeCount = 0;
}

int PhysicsWorld::spawn(ShapeType shape, Vec2 _size, float x, float y, float vx, float vy, float lifetime,
                         float _gravity, uint8_t _layer, uint8_t _mask, bool _isEntity, bool _isPlatform, bool _isParticle) {
    std::lock_guard<std::mutex> lock(particle_mutex_);
    if (activeCount >= maxEntities) return -1; // Database is full!

    size_t idx = activeCount;
    velX[idx] = vx;
    velY[idx] = vy;
    life[idx] = lifetime;
    size[idx] = _size;
    shapeType[idx] = shape;
    age[idx] = 0;
    posX[idx] = x;
    posY[idx] = y;
    gravity[idx] = _gravity;
    layer[idx] = _layer;
    mask[idx] = _mask;
    isEntity[idx] = _isEntity ? 1 : 0;
    isPlatform[idx] = _isPlatform ? 1 : 0;
    isParticle[idx] = _isParticle ? 1 : 0;

    // Always stamp -1 first: this index may be a reused slot from a previous
    // occupant, and its old back-pointer must never leak into the new cell.
    entitySlot[idx] = -1;
    platformSlot[idx] = -1;
    particleSlot[idx] = -1;
    emitterSlot[idx] = -1; // caller opts an entity into an emitter explicitly after spawn() returns
    if (_isEntity)   entitySlot[idx] = entities.Alloc();
    if (_isPlatform) platformSlot[idx] = platforms.Alloc();
    if (_isParticle) particleSlot[idx] = particles.Alloc();

    activeCount++; // Slide the boundary forward
    return static_cast<int>(idx);
}

void PhysicsWorld::compressArrays() {
    size_t writeIdx = 0;
    for (size_t readIdx = 0; readIdx < activeCount; ++readIdx) {
        if (life[readIdx] > 0.0f) {
            if (readIdx != writeIdx) {
                posX[writeIdx] = posX[readIdx];
                posY[writeIdx] = posY[readIdx];
                velX[writeIdx] = velX[readIdx];
                velY[writeIdx] = velY[readIdx];
                gravity[writeIdx] = gravity[readIdx];
                life[writeIdx] = life[readIdx];
                shapeType[writeIdx] = shapeType[readIdx];
                size[writeIdx] = size[readIdx];
                rotation[writeIdx] = rotation[readIdx];
                age[writeIdx] = age[readIdx];
                layer[writeIdx] = layer[readIdx];
                mask[writeIdx] = mask[readIdx];
                isTile[writeIdx] = isTile[readIdx];
                health[writeIdx] = health[readIdx];
                isEntity[writeIdx] = isEntity[readIdx];
                friction[writeIdx] = friction[readIdx];
                isParticle[writeIdx] = isParticle[readIdx];
                restitution[writeIdx] = restitution[readIdx];
                angularVel[writeIdx] = angularVel[readIdx];
                onCollision[writeIdx] = onCollision[readIdx];
                hasCollisionEvent[writeIdx] = hasCollisionEvent[readIdx];
                isPlatform[writeIdx] = isPlatform[readIdx];
                prevPos[writeIdx] = prevPos[readIdx];
                isGhosted[writeIdx] = isGhosted[readIdx];
                isSlope[writeIdx] = isSlope[readIdx];
                slopeType[writeIdx] = slopeType[readIdx];
                tileID[writeIdx] = tileID[readIdx];
                entitySlot[writeIdx] = entitySlot[readIdx];
                platformSlot[writeIdx] = platformSlot[readIdx];
                particleSlot[writeIdx] = particleSlot[readIdx];
                emitterSlot[writeIdx] = emitterSlot[readIdx];

                life[readIdx] = -1.0f;
                age[readIdx] = 0;
            }
            writeIdx++;
        } else {
            // This cell died this pass -- free its actor/platform slot now, before
            // its storage is potentially overwritten by a later surviving cell.
            // Rows that were merely relocated (the `readIdx != writeIdx` branch
            // above) are NOT touched here; their slot is still owned by the
            // relocated cell at its new position.
            if (entitySlot[readIdx] != -1) {
                entities.Free(entitySlot[readIdx]);
                entitySlot[readIdx] = -1;
            }
            if (platformSlot[readIdx] != -1) {
                platforms.Free(platformSlot[readIdx]);
                platformSlot[readIdx] = -1;
            }
            if (particleSlot[readIdx] != -1) {
                particles.Free(particleSlot[readIdx]);
                particleSlot[readIdx] = -1;
            }
            // emitterSlot is NOT freed here -- Physics2D doesn't own that pool (see the field's
            // comment in PhysicsWorld.h). The game must free its own emitter slot for this row
            // BEFORE calling compressArrays(), by checking (life[i] <= 0 && emitterSlot[i] != -1)
            // for every row first. Just clear the stale index so a future occupant of this slot
            // never inherits it.
            emitterSlot[readIdx] = -1;
        }
    }

    for (size_t i = writeIdx; i < activeCount; ++i) {
        life[i] = -1.0f;
        age[i] = 0;
    }

    activeCount = writeIdx;
}

} // namespace Physics2D
