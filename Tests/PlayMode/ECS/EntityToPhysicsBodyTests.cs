using Unity.Collections;
using Unity.Entities;
using Unity.Transforms;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class EntityToPhysicsBodyTests : DotsPlayModeTestFixture
    {
        [Test]
        public void EntityToPhysicsBodyLookup_PhysicsSystemNotRunYet()
        {
            // Fetch the physics system.
            var physicsWorldSystem = World.GetExistingSystem<PhysicsWorldSystem>();          

            var entity = EntityManager.CreateEntity();
            var physicsBody = physicsWorldSystem.GetPhysicsBody(entity);
            Assert.AreEqual(Entity.Null, physicsBody.Entity, "Expected an invalid PhysicsBody.");

            EntityManager.DestroyEntity(entity);
        }

        [Test]
        public unsafe void EntityToPhysicsBodyLookup_InvalidEntity()
        {
            // Fetch the physics system.
            var physicsWorldSystem = World.GetExistingSystem<PhysicsWorldSystem>();          

            var physicsBody = physicsWorldSystem.GetPhysicsBody(Entity.Null);
            Assert.AreEqual(Entity.Null, physicsBody.Entity, "Expected an invalid PhysicsBody.");
        }

        [Test]
        public void EntityToPhysicsBodyLookup_NoEntities()
        {
            // Run the system.
            MainLoop();

            // Fetch the physics system.
            var physicsWorldSystem = World.GetExistingSystem<PhysicsWorldSystem>();          

            var entity = EntityManager.CreateEntity();
            var physicsBody = physicsWorldSystem.GetPhysicsBody(entity);
            Assert.AreEqual(Entity.Null, physicsBody.Entity, "Expected an invalid PhysicsBody.");

            EntityManager.DestroyEntity(entity);
        }

        [Test]
        public void EntityToPhysicsBodyLookup(
            [Values(0, 133, 533, 1323)]int DynamicBodyCount,
            [Values(0, 74, 483, 3525)]int StaticBodyCount)
        {
            var dynamicBodyArchetype = EntityManager.CreateArchetype(
                typeof(PhysicsVelocity),
                typeof(PhysicsColliderBlob),
                typeof(Translation),
                typeof(Rotation)
                );

            var staticBodyArchetype = EntityManager.CreateArchetype(
                typeof(PhysicsColliderBlob),
                typeof(LocalToWorld));

            // Create the body entities.
            var dynamicBodyEntities = EntityManager.CreateEntity(dynamicBodyArchetype, DynamicBodyCount, Allocator.Temp);
            var staticBodyEntities = EntityManager.CreateEntity(staticBodyArchetype, StaticBodyCount, Allocator.Temp);

            // Run the system.
            MainLoop();

            // Fetch the physics system.
            var physicsWorldSystem = World.GetExistingSystem<PhysicsWorldSystem>();          

            // Validate that all the Dynamic PhysicsBody are referenced correctly.
            for(var i = 0; i < DynamicBodyCount; ++i)
            {
                var expectedEntity = dynamicBodyEntities[i];
                var actualEntity = physicsWorldSystem.GetPhysicsBody(expectedEntity).Entity;

                Assert.AreNotEqual(expectedEntity, Entity.Null);
                Assert.AreNotEqual(actualEntity, Entity.Null);
                Assert.AreEqual(expectedEntity, actualEntity, "Could not find Dynamic Body.");
            };

            // Validate that all the Static PhysicsBody are referenced correctly.
            for(var i = 0; i < StaticBodyCount; ++i)
            {
                var expectedEntity = staticBodyEntities[i];
                var actualEntity = physicsWorldSystem.GetPhysicsBody(expectedEntity).Entity;

                Assert.AreNotEqual(expectedEntity, Entity.Null);
                Assert.AreNotEqual(actualEntity, Entity.Null);
                Assert.AreEqual(expectedEntity, actualEntity, "Could not find Static Body.");
            };

            // Destroy the entities.
            EntityManager.DestroyEntity(dynamicBodyEntities);
            EntityManager.DestroyEntity(staticBodyEntities);

            // Destroy the entity arrays.
            dynamicBodyEntities.Dispose();
            staticBodyEntities.Dispose();
        }
    }
}
