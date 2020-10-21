using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsColliderBlobOwnerTests : DotsPlayModeTestFixture
    {
        [Test]
        public void TestPhysicsColliderBlobOwnerNoAllocation()
        {
            var entity = EntityManager.CreateEntity(typeof(PhysicsColliderBlobOwner));

            Assert.IsTrue(EntityManager.HasComponent<PhysicsColliderBlobOwner>(entity));
        }

        [Test]
        public void TestPhysicsColliderBlobOwner()
        {
            // Create the collider blob.
            var geometry = new CircleGeometry { Radius = 1f };
            var colliderBlob = PhysicsCircleCollider.Create(geometry);

            // Create the entity that owns the collider blob.
            var entity = EntityManager.CreateEntity();
            EntityManager.AddComponentData(entity, new PhysicsColliderBlob { Collider = colliderBlob });
            EntityManager.AddComponentData(entity, new PhysicsColliderBlobOwner { Collider = colliderBlob });

            // The collider blob components should be present.
            Assert.IsTrue(EntityManager.HasComponent<PhysicsColliderBlob>(entity));
            Assert.IsTrue(EntityManager.HasComponent<PhysicsColliderBlobOwner>(entity));

            // Destroy the entity.
            EntityManager.DestroyEntity(entity);

            // The collider blob component should be removed but the blob owner should be present still.
            Assert.IsFalse(EntityManager.HasComponent<PhysicsColliderBlob>(entity));
            Assert.IsTrue(EntityManager.HasComponent<PhysicsColliderBlobOwner>(entity));

            // Run the system.
            MainLoop();

            // The collider blob owner should now be removed indicating that the
            // disposal system has disposed of the blob.
            // Unfortunately I don't believe there's a way to detect the actual deallocation
            // as our instance here still assumes the blob ptr is valid.
            Assert.IsFalse(EntityManager.HasComponent<PhysicsColliderBlobOwner>(entity));
        }
    }
}
