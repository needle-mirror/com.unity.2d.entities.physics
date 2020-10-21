using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class RaycastTests : BaseQueryFixture
    {
        const float RayEpsilon = QueryEpsilon * 0.1f;

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
        public void Raycast_Against_NonRotatedBox_Test(
            [Values(0f, 10f, -20f, 30f, 40f)] float startX,
            [Values(0f, -10f, 30f, -50f, 70f)] float startY)
        {
            // Simulate the physics.
            SimulatePhysics();

            var aabb = NonRotatedBox.Aabb;
            Assert.IsTrue(startX > aabb.Max.x);

            var start = new float2(startX, startY);
            var expectedPosition = new float2(aabb.Max.x - RayEpsilon, 0f);

            // Set-up the query.
            var input = new RaycastInput
            {
                Start = start,
                End = expectedPosition,

                Filter = CollisionFilter.Default
            };

            // Perform the query.
            var results = PhysicsWorld.CastRay(input, out RaycastHit hit);
            Assert.IsTrue(results);
            Assert.IsTrue(hit.PhysicsBodyIndex != PhysicsBody.Constants.InvalidBodyIndex, "PhysicsBody Index is Invalid.");
            Assert.AreEqual(hit.Entity, NonRotatedBox.Entity, "Entity is invalid.");
            PhysicsAssert.AreEqual(expectedPosition, hit.Position, QueryEpsilon, "Hit position is incorrect.");
        }

        [Test]
        public void Raycast_Against_RotatedBox_Test(
            [Values(0f, 10f, -20f, 30f, 40f)] float startX,
            [Values(0f, -10f, 30f, -50f, 70f)] float startY)
        {
            // Simulate the physics.
            SimulatePhysics();

            var aabb = RotatedBox.Aabb;
            Assert.IsTrue(startX < aabb.Min.x);

            var start = new float2(startX, startY);
            var expectedPosition = new float2(aabb.Min.x + RayEpsilon, 0f);

            // Set-up the query.
            var input = new RaycastInput
            {
                Start = start,
                End = expectedPosition,

                Filter = CollisionFilter.Default
            };

            // Perform the query.
            var results = PhysicsWorld.CastRay(input, out RaycastHit hit);
            Assert.IsTrue(results);
            Assert.IsTrue(hit.PhysicsBodyIndex != PhysicsBody.Constants.InvalidBodyIndex, "PhysicsBody Index is Invalid.");
            Assert.AreEqual(hit.Entity, RotatedBox.Entity, "Entity is invalid.");
            PhysicsAssert.AreEqual(expectedPosition, hit.Position, QueryEpsilon, "Hit position is incorrect.");
        }
    }
}
