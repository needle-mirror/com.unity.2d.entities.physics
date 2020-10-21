using System;

using Unity.Burst;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsBoxColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                PhysicsBoxCollider.Create(new BoxGeometry { Size = new float2(1f), Center = float2.zero, Angle = 0f }).Dispose();
        }

        [Test]
        public void PhysicsBoxCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        [Test]
        public void TestPhysicsBoxColliderCreate()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(0.01f, 120.40f),
                Center = new float2(-10.10f, 10.12f),
                Angle = 0f,
                BevelRadius = 0f
            };

            const uint UserData = 0xDEADBEEF;

            using (var colliderBlob = PhysicsBoxCollider.Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default, UserData))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsBoxCollider>();

                Assert.AreEqual(ColliderType.Box, collider.ColliderType);
                Assert.AreEqual(CollisionType.Convex, collider.CollisionType);
                Assert.AreEqual(UserData, collider.UserData);
                Assert.AreEqual(CollisionFilter.Default, collider.Filter);
                Assert.AreEqual(PhysicsMaterial.Default, collider.Material);

                Assert.AreEqual(geometry.Size, collider.Size);
                Assert.AreEqual(geometry.Size, collider.Geometry.Size);
                Assert.AreEqual(geometry.Center, collider.Center);
                Assert.AreEqual(geometry.Center, collider.Geometry.Center);
                Assert.AreEqual(geometry.Angle, collider.Angle);
                Assert.AreEqual(geometry.Angle, collider.Geometry.Angle);
                Assert.AreEqual(geometry.BevelRadius, collider.BevelRadius);
                Assert.AreEqual(geometry.BevelRadius, collider.Geometry.BevelRadius);
            }
        }

        [Test]
        public void TestPhysicsBoxColliderCreateInvalid()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(1.0f, 2.0f),
                Center = new float2(1.0f, 0.0f),
                Angle = 0f,
                BevelRadius = 0.45f
            };

            // Invalid center, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.PositiveInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid center, negative infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.NegativeInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid center, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Center = new float2(float.NaN);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Negative size
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float2(-1.0f);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid size, positive inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float2(float.PositiveInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid size, negative inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float2(float.NegativeInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid size, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Size = new float2(float.NaN);
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Negative bevel radius
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = -0.0001f;
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, +inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.PositiveInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, -inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.NegativeInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.BevelRadius = float.NaN;
                Assert.Throws<ArgumentException>(() => PhysicsBoxCollider.Create(invalidGeometry));
            }
        }

        #endregion

        #region IConvexCollider

        [Test]
        public void TestPhysicsBoxColliderCalculateAabbLocalTranslation()
        {
            {
                var geometry = new BoxGeometry
                {
                    Size = new float2(2.32f, 10.87f),
                    Center = new float2(-0.59f, 0.36f),
                    Angle = 0f,
                    BevelRadius = 0.25f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new float2(-1.75f, -5.075f),
                    Max = new float2(0.57f, 5.795f)
                };

                using (var colliderBlob = PhysicsBoxCollider.Create(geometry))
                {
                    var aabb = colliderBlob.Value.CalculateAabb();

                    Assert.AreEqual(expectedAabb.Min, aabb.Min);
                    Assert.AreEqual(expectedAabb.Max, aabb.Max);
                }
            }
        }

        [Test]
        public void TestPhysicsBoxColliderMassProperties()
        {
            var geometry = new BoxGeometry
            {
                Center = new float2(1.5f, -6.2f),
                Size = new float2(5.0f, 250.0f),
                Angle = 1.8f,
                BevelRadius = 0f
            };

            using (var colliderBlob = PhysicsBoxCollider.Create(geometry))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsBoxCollider>();

                var massProperties = collider.MassProperties;
                var convexHullMassProperties = collider.m_ConvexHull.GetMassProperties();

                Assert.AreEqual(convexHullMassProperties.MassDistribution.LocalCenterOfMass, massProperties.MassDistribution.LocalCenterOfMass);
                Assert.AreEqual(convexHullMassProperties.MassDistribution.InverseInertia, massProperties.MassDistribution.InverseInertia);
                Assert.AreEqual(convexHullMassProperties.Area, massProperties.Area);
                Assert.AreEqual(convexHullMassProperties.AngularExpansionFactor, massProperties.AngularExpansionFactor);
            }
        }

        #endregion
    }
}
