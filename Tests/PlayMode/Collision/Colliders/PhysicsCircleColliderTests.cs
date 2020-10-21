using System;

using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsCircleColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        private struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                PhysicsCircleCollider.Create(new CircleGeometry { Center = float2.zero, Radius = 0.5f }).Dispose();
        }

        [Test]
        public void PhysicsCircleCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        [Test]
        public void TestPhysicsCircleColliderCreate()
        {
            var geometry = new CircleGeometry
            {
                Center = new float2(-10.10f, 10.12f),
                Radius = 3.0f
            };

            const uint UserData = 0xDEADBEEF;

            using (var colliderBlob = PhysicsCircleCollider.Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default, UserData))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCircleCollider>();

                Assert.AreEqual(ColliderType.Circle, collider.ColliderType);
                Assert.AreEqual(CollisionType.Convex, collider.CollisionType);
                Assert.AreEqual(UserData, collider.UserData);
                Assert.AreEqual(CollisionFilter.Default, collider.Filter);
                Assert.AreEqual(PhysicsMaterial.Default, collider.Material);

                Assert.AreEqual(geometry.Center, collider.Center);
                Assert.AreEqual(geometry.Center, collider.Geometry.Center);
                Assert.AreEqual(geometry.Radius, collider.Radius);
                Assert.AreEqual(geometry.Radius, collider.Geometry.Radius);
            }
        }

        [Test]
        public void TestPhysicsCircleColliderCreateInvalid()
        {
            var geometry = new CircleGeometry
            {
                Center = new float2(1.0f, 0.0f),
                Radius = 3.0f
            };

            // Invalid center, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.PositiveInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Invalid center, negative infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.NegativeInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Invalid center, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.NaN);
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Negative radius
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = -1.0f;
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Invalid radius, positive inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.PositiveInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Invalid radius, negative inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.NegativeInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }

            // Invalid radius, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.NaN;
                Assert.Throws<ArgumentException>(() => PhysicsCircleCollider.Create(invalidGeometry));
            }
        }

        #endregion

        #region IConvexCollider

        [Test]
        public void TestPhysicsCircleColliderCalculateAabbLocalTranslation()
        {
            {
                var geometry = new CircleGeometry
                {
                    Center = new float2(-0.59f, 0.36f),
                    Radius = 5.0f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new float2(-5.59f, -4.64f),
                    Max = new float2(4.41f, 5.36f)
                };

                using (var physicCircleCollider = PhysicsCircleCollider.Create(geometry))
                {
                    var aabb = physicCircleCollider.Value.CalculateAabb();

                    Assert.AreEqual(expectedAabb.Min, aabb.Min);
                    Assert.AreEqual(expectedAabb.Max, aabb.Max);
                }
            }
        }

        [Test]
        public void TestPhysicsCircleColliderMassProperties()
        {
            var geometry = new CircleGeometry
            {
                Center = new float2(1.5f, -6.2f),
                Radius = 23.4f
            };

            using (var colliderBlob = PhysicsCircleCollider.Create(geometry))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCircleCollider>();
                
                var massProperties = collider.MassProperties;
                
                // The following assumptions are made for the MassProperties:
                var radiusSqr = geometry.Radius * geometry.Radius;
                var area = math.PI * radiusSqr;
                var mass = area;
                var localCenterOfMass = geometry.Center;
                var inertia = mass * ((radiusSqr * 0.5f) + math.dot(localCenterOfMass, localCenterOfMass));
                var angularExpansionFactor = 0.0f;

                Assert.AreEqual(localCenterOfMass, massProperties.MassDistribution.LocalCenterOfMass);
                Assert.AreEqual(1.0f / inertia, massProperties.MassDistribution.InverseInertia);
                Assert.AreEqual(area, massProperties.Area);
                Assert.AreEqual(angularExpansionFactor, massProperties.AngularExpansionFactor);
            }
        }

        #endregion
    }
}
