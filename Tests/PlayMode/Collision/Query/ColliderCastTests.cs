using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class ColliderCastTests : BaseQueryFixture
    {
        const float ColliderEpsilon = QueryEpsilon * 0.1f;
        const float BoxSize = 5f;
        const float BoxRotation = 45f;

        protected TestBox NonRotatedBox { get; private set; }
        protected TestBox RotatedBox { get; private set; }

        [SetUp]
        public override void Setup()
        {
            base.Setup();

            NonRotatedBox = new TestBox(
                new BoxGeometry { Size = new float2(5f, 10f) },
                new float3(-100f, 0f, 0f),
                quaternion.identity
                );

            RotatedBox = new TestBox(
                new BoxGeometry { Size = new float2(20f, 30f) },
                new float3(100f, 0f, 0f),
                quaternion.RotateZ(math.radians(90f))
                );
        }

        [TearDown]
        public override void TearDown()
        {
            NonRotatedBox.Dispose();
            RotatedBox.Dispose();

            base.TearDown();
        }

        [Test]
        public void RotatedColliderCast_Against_NonRotatedBox_Test(
            [Values(0f, 10f, -20f, 30f, 40f)] float startX)
        {
            // Simulate the physics.
            SimulatePhysics();

            var aabb = NonRotatedBox.Aabb;
            Assert.IsTrue(startX > aabb.Max.x);

            var start = new float2(startX, 0f);
            var expectedPosition = new float2(aabb.Max.x - ColliderEpsilon, 0f);

            // Set-up the query.
            var geometry = new BoxGeometry { Size = new float2(BoxSize) };
            using (var colliderBlob = PhysicsBoxCollider.Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default))
            {
                var input = new ColliderCastInput
                {
                    Start = start,
                    End = expectedPosition,

                    Rotation = float2x2.Rotate(math.radians(BoxRotation)),
                    Collider = colliderBlob
                };

                // Perform the query.
                var results = PhysicsWorld.CastCollider(input, out ColliderCastHit hit);
                Assert.IsTrue(results);
                Assert.IsTrue(hit.PhysicsBodyIndex != PhysicsBody.Constants.InvalidBodyIndex, "PhysicsBody Index is Invalid.");
                Assert.AreEqual(hit.Entity, NonRotatedBox.Entity, "Entity is invalid.");
                PhysicsAssert.AreEqual(expectedPosition, hit.Position, QueryEpsilon, "Hit position is incorrect.");
            }
        }

        [Test]
        public void RotatedColliderCast_Against_RotatedBox_Test(
            [Values(0f, 10f, -20f, 30f, 40f)] float startX)
        {
            // Simulate the physics.
            SimulatePhysics();

            var aabb = RotatedBox.Aabb;
            Assert.IsTrue(startX < aabb.Min.x);

            var start = new float2(startX, 0f);
            var expectedPosition = new float2(aabb.Min.x + ColliderEpsilon, 0f);

            // Set-up the query.
            var geometry = new BoxGeometry { Size = new float2(BoxSize) };
            using (var colliderBlob = PhysicsBoxCollider.Create(geometry, CollisionFilter.Default, PhysicsMaterial.Default))
            {
                var input = new ColliderCastInput
                {
                    Start = start,
                    End = expectedPosition,

                    Rotation = float2x2.Rotate(math.radians(BoxRotation)),
                    Collider = colliderBlob
                };

                // Perform the query.
                var results = PhysicsWorld.CastCollider(input, out ColliderCastHit hit);
                Assert.IsTrue(results);
                Assert.IsTrue(hit.PhysicsBodyIndex != PhysicsBody.Constants.InvalidBodyIndex, "PhysicsBody Index is Invalid.");
                Assert.AreEqual(hit.Entity, RotatedBox.Entity, "Entity is invalid.");
                PhysicsAssert.AreEqual(expectedPosition, hit.Position, QueryEpsilon, "Hit position is incorrect.");
            }
        }
    }
}
