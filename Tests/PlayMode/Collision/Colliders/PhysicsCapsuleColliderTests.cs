using System;

using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsCapsuleColliderTests
    {
        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        struct CreateFromBurstJob : IJob
        {
            public void Execute() =>
                PhysicsCapsuleCollider.Create(new CapsuleGeometry { Vertex0 = new float2(0f, -1f), Vertex1 = new float2(0f, 1f), Radius = 0.5f }).Dispose();
        }

        [Test]
        public void PhysicsCapsuleCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob().Run();

        [Test]
        public void TestPhysicsCapsuleColliderCreate()
        {
            var geometry = new CapsuleGeometry
            {
                Vertex0 = new float2(0f, -2f),
                Vertex1 = new float2(0f, 3f),
                Radius = 1.5f
            };

            using (var colliderBlob = PhysicsCapsuleCollider.Create(geometry))
            {                
                var collider = colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                Assert.AreEqual(ColliderType.Capsule, collider.ColliderType);
                Assert.AreEqual(CollisionType.Convex, collider.CollisionType);

                Assert.AreEqual(geometry.Vertex0, collider.Vertex0);
                Assert.AreEqual(geometry.Vertex0, collider.Geometry.Vertex0);
                Assert.AreEqual(geometry.Vertex1, collider.Vertex1);
                Assert.AreEqual(geometry.Vertex1, collider.Geometry.Vertex1);
                Assert.AreEqual(geometry.Radius, collider.Radius);
                Assert.AreEqual(geometry.Radius, collider.Geometry.Radius);
            }
        }

        [Test]
        public void TestPhysicsCapsuleColliderCreateInvalid()
        {
            var geometry = new CapsuleGeometry
            {
                Vertex0 = new float2(0f, -2f),
                Vertex1 = new float2(0f, 3f),
                Radius = 1.5f
            };

            // Invalid vertex0, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex0 = new float2(float.PositiveInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid vertex0, negative infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex0 = new float2(float.NegativeInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid vertex0, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex0 = new float2(float.NaN);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid vertex1, positive infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex1 = new float2(float.PositiveInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid vertex1, negative infinity
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex1 = new float2(float.NegativeInfinity);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid vertex1, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Vertex1 = new float2(float.NaN);
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Negative radius
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = -1.0f;
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid radius, positive inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.PositiveInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid radius, negative inf
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.NegativeInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }

            // Invalid radius, nan
            {
                var invalidGeometry = geometry;
                invalidGeometry.Radius = float.NaN;
                Assert.Throws<ArgumentException>(() => PhysicsCapsuleCollider.Create(invalidGeometry));
            }
        }

        #endregion

        #region IConvexCollider

        [Test]
        public void TestPhysicsCapsuleColliderCalculateAabbLocalTranslation()
        {
            {
                var geometry = new CapsuleGeometry
                {
                    Vertex0 = new float2(-23f, -2f),
                    Vertex1 = new float2(2.1f, 3f),
                    Radius = 1.5f
                };

                Aabb expectedAabb = new Aabb
                {
                    Min = new float2(-24.5f, -3.5f),
                    Max = new  float2(3.6f, 4.5f)
                };

                using (var colliderBlob = PhysicsCapsuleCollider.Create(geometry))
                {
                    var aabb = colliderBlob.Value.CalculateAabb();

                    Assert.AreEqual(expectedAabb.Min, aabb.Min);
                    Assert.AreEqual(expectedAabb.Max, aabb.Max);
                }
            }
        }

        [Test]
        public void TestPhysicsCapsuleColliderMassProperties()
        {
            var geometry = new CapsuleGeometry
            {
                Vertex0 = new float2(0f, -2f),
                Vertex1 = new float2(0f, 3f),
                Radius = 1.5f
            };

            const uint UserData = 0xDEADBEEF;

            using (var colliderBlob = PhysicsCapsuleCollider.Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default, UserData))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                Assert.AreEqual(ColliderType.Capsule, collider.ColliderType);
                Assert.AreEqual(CollisionType.Convex, collider.CollisionType);
                Assert.AreEqual(UserData, collider.UserData);
                Assert.AreEqual(CollisionFilter.Default, collider.Filter);
                Assert.AreEqual(PhysicsMaterial.Default, collider.Material);

                // The following assumptions are made for the MassProperties:
                var radiusSqr = geometry.Radius * geometry.Radius;
                var bodyLength = math.distance(geometry.Vertex0, geometry.Vertex1);
                var bodyArea = bodyLength * geometry.Radius * 2.0f;
                var bodyMass = bodyArea;
                var bodyInertia = bodyMass * (bodyLength * bodyLength + radiusSqr) / 12.0f;

                var capsArea = math.PI * radiusSqr;
                var capsMass = capsArea;
                var capsInertia = capsMass * (0.5f * radiusSqr + bodyLength * bodyLength * 0.25f);

                var mass = bodyMass + capsArea;
                var localCenterOfMass = 0.5f * (geometry.Vertex0 + geometry.Vertex1);
                var area = bodyArea + capsArea;
                var inertia = bodyInertia + capsInertia + mass * math.dot(localCenterOfMass, localCenterOfMass);
                var angularExpansionFactor = math.length(geometry.Vertex1 - geometry.Vertex0) * 0.5f;

                var massProperties = collider.MassProperties;
                Assert.AreEqual(localCenterOfMass, massProperties.MassDistribution.LocalCenterOfMass);
                Assert.AreEqual(1.0f / inertia, massProperties.MassDistribution.InverseInertia);
                Assert.AreEqual(area, massProperties.Area);
                Assert.AreEqual(angularExpansionFactor, massProperties.AngularExpansionFactor);
            }
        }

        #endregion
    }
}
