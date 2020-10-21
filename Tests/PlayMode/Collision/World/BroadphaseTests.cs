using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class BroadPhaseTests
    {
        NativeList<BlobAssetReference<Collider>> m_ColliderBlobs;

        [SetUp]
        public void Setup()
        {
            m_ColliderBlobs = new NativeList<BlobAssetReference<Collider>>(150, Allocator.Temp);
        }

        [TearDown]
        public void TearDown()
        {
            // Dispose all ColliderBlobs.
            for(var i = 0; i < m_ColliderBlobs.Length; ++i)
            {
                m_ColliderBlobs[i].Dispose();
            }

            m_ColliderBlobs.Dispose();
        }

        [Test]
        public void Broadphase_InitializeTest()
        {
            var broadphase = new Broadphase(0, 0);
            broadphase.Dispose();
        }

        [Test]
        public void Broadphase_ScheduleBuildJobs_StaticBoxTest([Values(0, 1, 10, 100)] int staticBodyCount)
        {
            var physicsWorld = CreatePhysicsWorld(staticBodies : staticBodyCount, dynamicBodies : 0);

            for (var i = 0; i < staticBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * 11f, 0f), Size = new float2(10f) };
                AddStaticBoxToWorld(ref physicsWorld, i, geometry);
            }

            using (var buildStaticTree = new NativeArray<int>(new[] { 1 }, Allocator.TempJob))
            {
                physicsWorld.CollisionWorld.Broadphase.ScheduleBuildJobs(ref physicsWorld, buildStaticTree, default).Complete();
                physicsWorld.Dispose();
            }
        }

        [Test]
        public void Broadphase_ScheduleBuildJobs_DynamicBoxTest([Values(0, 1, 10, 100)] int dynamicBodyCount)
        {
            var physicsWorld = CreatePhysicsWorld(staticBodies : 1, dynamicBodies : dynamicBodyCount);

            {
                var geometry = new BoxGeometry { Center = new float2(-10f, 0f), Size = new float2(10f) };
                AddStaticBoxToWorld(ref physicsWorld, 0, geometry);
            }

            for (var i = 0; i < dynamicBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * 11f, 0f), Size = new float2(10f) };
                AddDynamicBoxToWorld(ref physicsWorld, i, geometry);
            }

            using (var buildStaticTree = new NativeArray<int>(new[] { 1 }, Allocator.TempJob))
            {
                physicsWorld.CollisionWorld.Broadphase.ScheduleBuildJobs(ref physicsWorld, buildStaticTree, default).Complete();
                physicsWorld.Dispose();
            }
        }

        [Test]
        public void Broadphase_ImmediateBuild_StaticBoxTest([Values(0, 1, 10, 100)] int staticBodyCount)
        {
            var physicsWorld = CreatePhysicsWorld(staticBodies : staticBodyCount, dynamicBodies : 0);

            for (var i = 0; i < staticBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * 11f, 0f), Size = new float2(10f) };
                AddStaticBoxToWorld(ref physicsWorld, i, geometry);
            }

            physicsWorld.CollisionWorld.Broadphase.Build(
                physicsWorld.StaticBodies,
                physicsWorld.DynamicBodies,
                physicsWorld.BodyMotionData,
                physicsWorld.BodyMotionVelocity,
                PhysicsSettings.Constants.CollisionTolerance,
                physicsWorld.TimeStep,
                physicsWorld.Settings.Gravity);

            physicsWorld.Dispose();
        }

        [Test]
        public void Broadphase_ImmediateBuild_DynamicBoxTest([Values(0, 1, 10, 100)] int dynamicBodyCount)
        {
            var physicsWorld = CreatePhysicsWorld(staticBodies : 1, dynamicBodies : dynamicBodyCount);

            {
                var geometry = new BoxGeometry { Center = new float2(-10f, 0f), Size = new float2(10f) };
                AddStaticBoxToWorld(ref physicsWorld, 0, geometry);
            }

            for (var i = 0; i < dynamicBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * 11f, 0f), Size = new float2(10f) };
                AddDynamicBoxToWorld(ref physicsWorld, i, geometry);
            }

            physicsWorld.CollisionWorld.Broadphase.Build(
                physicsWorld.StaticBodies,
                physicsWorld.DynamicBodies,
                physicsWorld.BodyMotionData,
                physicsWorld.BodyMotionVelocity,
                PhysicsSettings.Constants.CollisionTolerance,
                physicsWorld.TimeStep,
                physicsWorld.Settings.Gravity);

            physicsWorld.Dispose();
        }

        [Test]
        public void Broadphase_ScheduleBuildJobs_StaticAndDynamicBoxTest([Values(1, 10, 100)] int staticBodyCount, [Values(1, 10, 100)] int dynamicBodyCount)
        {
            var physicsWorld = CreatePhysicsWorld(staticBodies : staticBodyCount, dynamicBodies : dynamicBodyCount);

            for (var i = 0; i < staticBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * -11f, 0f), Size = new float2(10f) };
                AddStaticBoxToWorld(ref physicsWorld, i, geometry);
            }

            for (var i = 0; i < dynamicBodyCount; ++i)
            {
                var geometry = new BoxGeometry { Center = new float2(i * 11f, 0f), Size = new float2(10f) };
                AddDynamicBoxToWorld(ref physicsWorld, i, geometry);
            }

            using (var buildStaticTree = new NativeArray<int>(new[] { 1 }, Allocator.TempJob))
            {
                physicsWorld.CollisionWorld.Broadphase.ScheduleBuildJobs(ref physicsWorld, buildStaticTree, default).Complete();
                physicsWorld.Dispose();
            }
        }

        #region Utility

        static private PhysicsWorld CreatePhysicsWorld(int staticBodies, int dynamicBodies)
        {
            return new PhysicsWorld(staticBodyCount: staticBodies, dynamicBodyCount: dynamicBodies, jointCount: 0);
        }

        // Adds a static box to the world
        private unsafe void AddStaticBoxToWorld(ref PhysicsWorld physicsWorld, int index, BoxGeometry geometry)
        {
            Assert.IsTrue(index < physicsWorld.StaticBodyCount, "Static body index is out of range in AddStaticBoxToWorld");
            
            var bodies = physicsWorld.StaticBodies;
            var physicsBody = bodies[index];

            var colliderBlob = PhysicsBoxCollider.Create(geometry);
            physicsBody.SetCollider(colliderBlob);
            m_ColliderBlobs.Add(colliderBlob);

            bodies[index] = physicsBody;
        }

        // Adds a dynamic box to the world
        private unsafe void AddDynamicBoxToWorld(ref PhysicsWorld physicsWorld, int index, BoxGeometry geometry)
        {
            Assert.IsTrue(index < physicsWorld.DynamicBodyCount, "Dynamic body index is out of range in AddDynamicBoxToWorld");

            var bodies = physicsWorld.DynamicBodies;
            var physicsBody = bodies[index];

            var colliderBlob = PhysicsBoxCollider.Create(geometry);
            physicsBody.SetCollider(colliderBlob);
            m_ColliderBlobs.Add(colliderBlob);

            bodies[index] = physicsBody;
        }

        #endregion
    }
}
