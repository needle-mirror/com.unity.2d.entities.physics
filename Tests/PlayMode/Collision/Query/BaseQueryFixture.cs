using System;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace Unity.U2D.Entities.Physics.Tests
{
    abstract class BaseQueryFixture : DotsPlayModeTestFixture
    {
        public const float QueryEpsilon = 0.001f;

        protected struct TestBox : IDisposable
        {
            public Entity Entity { get; private set; }
            public BoxGeometry Geometry { get; private set; }
            public PhysicsTransform PhysicsTransform { get; private set; }
            public float3 Translation;
            public quaternion Rotation;
            public Aabb Aabb { get; private set; }
            public BlobAssetReference<Collider> ColliderBlob => EntityManager.GetComponentData<PhysicsColliderBlob>(Entity).Collider;

            public TestBox(BoxGeometry geometry, float3 translation, quaternion rotation)
            {
                Entity = EntityManager.CreateEntity(
                    typeof(PhysicsColliderBlob),
                    typeof(LocalToWorld)
                    );

                Geometry = geometry;

                PhysicsTransform = new PhysicsTransform(translation, rotation);
                Translation = translation;
                Rotation = rotation;

                var colliderBlob = PhysicsBoxCollider.Create(Geometry);

                Aabb = colliderBlob.Value.CalculateAabb(PhysicsTransform);

                EntityManager.AddComponentData(
                    Entity,
                    new PhysicsColliderBlob { Collider = colliderBlob }
                    );

                EntityManager.AddComponentData(
                    Entity,
                    new LocalToWorld { Value = new float4x4(rotation, translation) }
                    );
            }

            public void Dispose()
            {
                ColliderBlob.Dispose();
                EntityManager.DestroyEntity(Entity);
            }
        };

        protected ref PhysicsWorld PhysicsWorld => ref World.GetExistingSystem<PhysicsWorldSystem>().PhysicsWorld;

        protected void SimulatePhysics()
        {
            MainLoop();

            // Ensure the physics system has finished.
            World.GetExistingSystem<EndFramePhysicsSystem>().FinalJobHandle.Complete();
        }
    }
}
