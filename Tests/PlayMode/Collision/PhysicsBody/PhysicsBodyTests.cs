using Unity.Collections;
using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class PhysicsBodyTest
    {
        [Test]
        public void PhysicsBodyCalculateAabb_BoxColliderTest()
        {
            var geometry = new BoxGeometry
            {
                Size = 1.0f,
                BevelRadius = 0.2f
            };

            var collider1 = PhysicsBoxCollider.Create(geometry);
            var collider2 = PhysicsBoxCollider.Create(geometry);

            var physicsBody = new PhysicsBody(collider1);

            var aabb = physicsBody.CalculateAabb();

            Assert.IsTrue(aabb.Equals(collider2.Value.CalculateAabb()));

            collider1.Dispose();
            collider2.Dispose();
        }

        [Test]
        public void PhysicsBodyCalculateAabb_CapsuleColliderTest()
        {
            var geometry = new CapsuleGeometry
            {
                Vertex0 = new float2(0f, -2f),
                Vertex1 = new float2(0f, 3f),
                Radius = 1.5f
            };

            var collider1 = PhysicsCapsuleCollider.Create(geometry);
            var collider2 = PhysicsCapsuleCollider.Create(geometry);

            var physicsBody = new PhysicsBody(collider1);

            var aabb = physicsBody.CalculateAabb();

            Assert.IsTrue(aabb.Equals(collider2.Value.CalculateAabb()));

            collider1.Dispose();
            collider2.Dispose();
        }

        [Test]
        public void PhysicsBodyCalculateAabb_CircleColliderTest()
        {
            var geometry = new CircleGeometry
            {
                Center = new float2(-10.10f, 10.12f),
                Radius = 3.0f
            };

            var collider1 = PhysicsCircleCollider.Create(geometry);
            var collider2 = PhysicsCircleCollider.Create(geometry);

            var physicsBody = new PhysicsBody(collider1);

            var aabb = physicsBody.CalculateAabb();

            Assert.IsTrue(aabb.Equals(collider2.Value.CalculateAabb()));

            collider1.Dispose();
            collider2.Dispose();
        }

        [Test]
        public void PhysicsBodyCalculateAabb_PolygonColliderTest()
        {
            var vertices = new NativeArray<float2>(
                new float2[]
                {
                    new float2(-1f, -2f),
                    new float2(2f, -3f),
                    new float2(4f, 5f),
                    new float2(-6f, 7f)
                },
                Allocator.Temp
                );

            var geometry = new PolygonGeometry
            {
                Vertices = vertices,
                BevelRadius = 3.0f
            };

            var collider1 = PhysicsPolygonCollider.Create(geometry);
            var collider2 = PhysicsPolygonCollider.Create(geometry);

            var physicsBody = new PhysicsBody(collider1);

            var aabb = physicsBody.CalculateAabb();

            Assert.IsTrue(aabb.Equals(collider2.Value.CalculateAabb()));

            vertices.Dispose();
            collider1.Dispose();
            collider2.Dispose();
        }

        [Test]
        public unsafe void PhysicsBodyOverlapColliderTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(2f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var queryInput = new OverlapColliderInput() { Filter = CollisionFilter.Default };
                var closestHit = new OverlapColliderHit();
                var allHits = new NativeList<OverlapColliderHit>(Allocator.Temp);

                var circleGeometry = new CircleGeometry { Radius = 0.5f };
                using (var circleBlob = PhysicsCircleCollider.Create(circleGeometry))
                {
                    queryInput.Collider = circleBlob;

                    // OK case.
                    var transformOK = new PhysicsTransform(new float2(1f));
                    queryInput.Transform = transformOK;
                    Assert.IsTrue(physicsBody.OverlapCollider(queryInput));
                    Assert.IsTrue(physicsBody.OverlapCollider(queryInput, out closestHit));
                    Assert.IsTrue(physicsBody.OverlapCollider(queryInput, ref allHits));

                    // Fail Case.
                    var transformFail = new PhysicsTransform(new float2(-10f));
                    queryInput.Transform = transformFail;
                    Assert.IsFalse(physicsBody.OverlapCollider(queryInput));
                    Assert.IsFalse(physicsBody.OverlapCollider(queryInput, out closestHit));
                    Assert.IsFalse(physicsBody.OverlapCollider(queryInput, ref allHits));
                }

                allHits.Dispose();
            }
        }

        [Test]
        public unsafe void PhysicsBodyOverlapPointTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(2f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var queryInput = new OverlapPointInput() { Filter = CollisionFilter.Default };
                var closestHit = new OverlapPointHit();
                var allHits = new NativeList<OverlapPointHit>(Allocator.Temp);

                // OK case.
                var positionOK = new float2(1f);
                queryInput.Position = positionOK;
                Assert.IsTrue(physicsBody.OverlapPoint(queryInput));
                Assert.IsTrue(physicsBody.OverlapPoint(queryInput, out closestHit));
                Assert.IsTrue(physicsBody.OverlapPoint(queryInput, ref allHits));

                // Fail Case.
                var positionFail = new float2(-10f);
                queryInput.Position = positionFail;
                Assert.IsFalse(physicsBody.OverlapPoint(queryInput));
                Assert.IsFalse(physicsBody.OverlapPoint(queryInput, out closestHit));
                Assert.IsFalse(physicsBody.OverlapPoint(queryInput, ref allHits));

                allHits.Dispose();
            }
        }

        [Test]
        public unsafe void PhysicsBodyCastRayTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(1f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var queryInput  = new RaycastInput() { Filter = CollisionFilter.Default };
                var closestHit = new RaycastHit();
                var allHits = new NativeList<RaycastHit>(Allocator.Temp);

                // OK case.
                var startOK = new float2(-10f, -10f);
                var endOK = new float2(10f, 10f);
                queryInput.Start = startOK;
                queryInput.End = endOK;
                Assert.IsTrue(physicsBody.CastRay(queryInput));
                Assert.IsTrue(physicsBody.CastRay(queryInput, out closestHit));
                Assert.IsTrue(physicsBody.CastRay(queryInput, ref allHits));

                // Fail Case.
                var startFail = new float2(-10f, -10f);
                var endFail = new float2(-20f, -20f);
                queryInput.Start = startFail;
                queryInput.End = endFail;
                Assert.IsFalse(physicsBody.CastRay(queryInput));
                Assert.IsFalse(physicsBody.CastRay(queryInput, out closestHit));
                Assert.IsFalse(physicsBody.CastRay(queryInput, ref allHits));

                allHits.Dispose();
            }
        }

        [Test]
        public unsafe void PhysicsBodyCastColliderTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(1f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var queryInput = new ColliderCastInput() { Rotation = float2x2.identity };
                var closestHit = new ColliderCastHit();
                var allHits = new NativeList<ColliderCastHit>(Allocator.Temp);

                var circleGeometry = new CircleGeometry { Radius = 0.5f };

                using (var circleBlob = PhysicsCircleCollider.Create(circleGeometry))
                {
                    queryInput.Collider = circleBlob;

                    // OK case.
                    var startOK = new float2(-10f, -10f);
                    var endOK = new float2(10f, 10f);
                    queryInput.Start = startOK;
                    queryInput.End = endOK;                   
                    Assert.IsTrue(physicsBody.CastCollider(queryInput));
                    Assert.IsTrue(physicsBody.CastCollider(queryInput, out closestHit));
                    Assert.IsTrue(physicsBody.CastCollider(queryInput, ref allHits));

                    // Fail Case.
                    var startFail = new float2(-10f, -10f);
                    var endFail = new float2(10f, -10f);
                    queryInput.Start = startFail;
                    queryInput.End = endFail;
                    Assert.IsFalse(physicsBody.CastCollider(queryInput));
                    Assert.IsFalse(physicsBody.CastCollider(queryInput, out closestHit));
                    Assert.IsFalse(physicsBody.CastCollider(queryInput, ref allHits));
                }

                allHits.Dispose();
            }
        }

        [Test]
        public unsafe void PhysicsBodyCalculateDistancePointTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(1f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var queryInput = new PointDistanceInput
                {
                    Position = new float2(-10f),
                    Filter = CollisionFilter.Default
                };

                var closestHit = new DistanceHit();
                var allHits = new NativeList<DistanceHit>(Allocator.Temp);

                // OK case : with enough max distance
                queryInput.MaxDistance = 10000.0f;
                Assert.IsTrue(physicsBody.CalculateDistance(queryInput));
                Assert.IsTrue(physicsBody.CalculateDistance(queryInput, out closestHit));
                Assert.IsTrue(physicsBody.CalculateDistance(queryInput, ref allHits));

                // Fail case : not enough max distance
                queryInput.MaxDistance = 1;
                Assert.IsFalse(physicsBody.CalculateDistance(queryInput));
                Assert.IsFalse(physicsBody.CalculateDistance(queryInput, out closestHit));
                Assert.IsFalse(physicsBody.CalculateDistance(queryInput, ref allHits));

                allHits.Dispose();
            }
        }

        [Test]
        public unsafe void PhysicsBodyCalculateDistanceTest()
        {
            var geometry = new BoxGeometry
            {
                Size = new float2(1f),
            };

            using (var collider = PhysicsBoxCollider.Create(geometry))
            {
                var physicsBody = new PhysicsBody(collider);

                var circleGeometry = new CircleGeometry { Radius = 1f };

                using (var circleBlob = PhysicsCircleCollider.Create(circleGeometry))
                {
                    var queryInput = new ColliderDistanceInput
                    {
                        Collider = circleBlob,
                        Transform = new PhysicsTransform(new float2(-10f))
                    };

                    var closestHit = new DistanceHit();
                    var allHits = new NativeList<DistanceHit>(Allocator.Temp);

                    // OK case : with enough max distance
                    queryInput.MaxDistance = 10000.0f;
                    Assert.IsTrue(physicsBody.CalculateDistance(queryInput));
                    Assert.IsTrue(physicsBody.CalculateDistance(queryInput, out closestHit));
                    Assert.IsTrue(physicsBody.CalculateDistance(queryInput, ref allHits));

                    // Fail case : not enough max distance
                    queryInput.MaxDistance = 1f;
                    Assert.IsFalse(physicsBody.CalculateDistance(queryInput));
                    Assert.IsFalse(physicsBody.CalculateDistance(queryInput, out closestHit));
                    Assert.IsFalse(physicsBody.CalculateDistance(queryInput, ref allHits));

                    allHits.Dispose();
                }
            }
        }
    }
}
