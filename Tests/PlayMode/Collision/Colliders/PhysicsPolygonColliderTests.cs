using System;

using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsPolygonColliderTests
    {
        NativeArray<float2> m_Vertices;
        PolygonGeometry m_PolygonGeometry;

        [SetUp]
        public void Setup()
        {
            m_Vertices = new NativeArray<float2>(
                new []
                {
                    new float2(-1f, -2f),
                    new float2(2f, -3f),
                    new float2(4f, 5f),
                    new float2(-6f, 7f)
                },
                Allocator.Persistent
                );

            m_PolygonGeometry = new PolygonGeometry
            {
                Vertices = m_Vertices,
                BevelRadius = 0.0f
            };
        }

        [TearDown]
        public void TearDown()
        {
            m_Vertices.Dispose();
            m_PolygonGeometry = default;
        }

        #region Construction

        [BurstCompile(CompileSynchronously = true)]
        private struct CreateFromBurstJob : IJob
        {
            public PolygonGeometry Geometry;

            public void Execute() =>
                PhysicsPolygonCollider.Create(Geometry).Dispose();
        }

        [Test]
        public void PhysicsPolygonCollider_Create_WhenCalledFromBurstJob_DoesNotThrow() => new CreateFromBurstJob { Geometry = m_PolygonGeometry }.Run();

        [Test]
        public void TestPhysicsPolygonColliderCreate()
        {
            const uint UserData = 0xDEADBEEF;

            using (var colliderBlob = PhysicsPolygonCollider.Create(m_PolygonGeometry, CollisionFilter.Default, PhysicsMaterial.Default, UserData))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsPolygonCollider>();

                Assert.AreEqual(ColliderType.Polygon, collider.ColliderType);
                Assert.AreEqual(CollisionType.Convex, collider.CollisionType);
                Assert.AreEqual(UserData, collider.UserData);
                Assert.AreEqual(CollisionFilter.Default, collider.Filter);
                Assert.AreEqual(PhysicsMaterial.Default, collider.Material);

                Assert.AreEqual(m_PolygonGeometry.BevelRadius, collider.BevelRadius);
                Assert.AreEqual(m_PolygonGeometry.Vertices.Length, collider.VertexCount);

                var vertexCount = collider.VertexCount;
                var giftWrappedIndices = new int[4] { 2, 3, 0, 1 };
                for(var i = 0; i < vertexCount; ++i)
                {
                    Assert.AreEqual(m_PolygonGeometry.Vertices[i], collider.Vertices[giftWrappedIndices[i]]);
                }
            }
        }

        [Test]
        public void TestPhysicsPolygonColliderCreateInvalid()
        {
            var vertexCount = m_Vertices.Length;
            for(var i = 0; i < vertexCount; ++i)
            {
                // Invalid vertex, positive infinity
                {
                    var invalidVertices = new NativeArray<float2>(m_Vertices, Allocator.Temp);
                    {
                        invalidVertices[i] = new float2(float.PositiveInfinity);

                        var invalidGeometry = new PolygonGeometry { Vertices = invalidVertices, BevelRadius = 0f };
                        Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
                    }
                    invalidVertices.Dispose();
                }

                // Invalid vertex, negative infinity
                {
                    var invalidVertices = new NativeArray<float2>(m_Vertices, Allocator.Temp);
                    {
                        invalidVertices[i] = new float2(float.NegativeInfinity);

                        var invalidGeometry = new PolygonGeometry { Vertices = invalidVertices, BevelRadius = 0f };
                        Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
                    }
                    invalidVertices.Dispose();
                }

                // Invalid vertex, nan
                {
                    var invalidVertices = new NativeArray<float2>(m_Vertices, Allocator.Temp);
                    {
                        invalidVertices[i] = new float2(float.NaN);

                        var invalidGeometry = new PolygonGeometry { Vertices = invalidVertices, BevelRadius = 0f };
                        Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
                    }
                    invalidVertices.Dispose();
                }
            }

            // Negative bevel radius
            {
                var invalidGeometry = m_PolygonGeometry;
                invalidGeometry.BevelRadius = -0.0001f;
                Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, +inf
            {
                var invalidGeometry = m_PolygonGeometry;
                invalidGeometry.BevelRadius = float.PositiveInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, -inf
            {
                var invalidGeometry = m_PolygonGeometry;
                invalidGeometry.BevelRadius = float.NegativeInfinity;
                Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
            }

            // Invalid bevel radius, nan
            {
                var invalidGeometry = m_PolygonGeometry;
                invalidGeometry.BevelRadius = float.NaN;
                Assert.Throws<ArgumentException>(() => PhysicsPolygonCollider.Create(invalidGeometry));
            }
        }

        #endregion

        #region IConvexCollider

        [Test]
        public void TestPhysicsPolygonColliderCalculateAabbLocalTranslation()
        {
            {
                Aabb expectedAabb = new Aabb
                {
                    Min = new  float2(-6f, -3f),
                    Max = new  float2(4f, 7f)
                };

                using (var colliderBlob = PhysicsPolygonCollider.Create(m_PolygonGeometry))
                {
                    var aabb = colliderBlob.Value.CalculateAabb();

                    Assert.AreEqual(expectedAabb.Min, aabb.Min);
                    Assert.AreEqual(expectedAabb.Max, aabb.Max);
                }
            }
        }

        [Test]
        public void TestPhysicsPolygonColliderMassProperties()
        {
            using (var colliderBlob = PhysicsPolygonCollider.Create(m_PolygonGeometry))
            {
                ref var collider = ref colliderBlob.GetColliderRef<PhysicsPolygonCollider>();

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
