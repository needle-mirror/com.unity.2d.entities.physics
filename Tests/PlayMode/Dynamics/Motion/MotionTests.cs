using Unity.Mathematics;

using NUnit.Framework;

namespace Unity.U2D.Entities.Physics.Tests
{
    class MotionTests
    {
        [Test]
        public void MassPropertiesDefaultTest()
        {
            var defaultMassProperties = MassProperties.Default;

            Assert.AreEqual(float2.zero, defaultMassProperties.MassDistribution.LocalCenterOfMass);
            Assert.AreEqual(1.0f, defaultMassProperties.MassDistribution.InverseInertia);
            Assert.AreEqual(1.0f, defaultMassProperties.Area);
            Assert.AreEqual(0.0f, defaultMassProperties.AngularExpansionFactor);
        }

        [Test]
        public void MotionVelocityApplyLinearImpulseTest()
        {
            // Test values.
            var initialLinearVelocity = new float2(3.0f, 4.0f);
            var inverseMass = 0.5f;
            var impulse = new float2(2.0f, 3.0f);

            // Expectations.
            var expectedLinearVelocity = initialLinearVelocity + impulse * inverseMass;

            var motionVelocity = new PhysicsBody.MotionVelocity()
            {
                LinearVelocity = initialLinearVelocity,
                InverseMass = inverseMass
            };

            motionVelocity.ApplyLinearImpulse(impulse);

            Assert.AreEqual(expectedLinearVelocity, motionVelocity.LinearVelocity);
        }

        [Test]
        public void MotionVelocityApplyAngularImpulseTest()
        {
            // Test values.
            var initialAngularVelocity = 4.0f;
            var inverseInertia = 2.0f;
            var impulse = 5.0f;

            // Expectations.
            var expectedAngularVelocity = initialAngularVelocity + impulse * inverseInertia;

            var motionVelocity = new PhysicsBody.MotionVelocity()
            {
                AngularVelocity = initialAngularVelocity,
                InverseInertia = inverseInertia
            };
            motionVelocity.ApplyAngularImpulse(impulse);

            Assert.AreEqual(expectedAngularVelocity, motionVelocity.AngularVelocity);
        }

        [Test]
        public void MotionVelocityCalculateExpansionTest()
        {
            // Test values.
            var initialLinearVelocity = new float2(3.0f, 4.0f);
            var initialAngularVelocity = 4.0f;
            var inverseMass = 0.5f;
            var inverseInertia = 2.0f;
            var angularExpansionFactor = 1.2f;
            var timeStep = 1.0f / 60.0f;

            // Expectations.
            var expectedLinear = initialLinearVelocity * timeStep;
            var expectedUniform = math.min(math.length(initialAngularVelocity) * timeStep, (float)math.PI / 2.0f) * angularExpansionFactor;

            var motionVelocity = new PhysicsBody.MotionVelocity()
            {
                LinearVelocity = initialLinearVelocity,
                AngularVelocity = initialAngularVelocity,
                InverseMass = inverseMass,
                InverseInertia = inverseInertia,
                AngularExpansionFactor = angularExpansionFactor
            };

            var motionExpansion = motionVelocity.CalculateExpansion(timeStep);

            Assert.AreEqual(expectedLinear, motionExpansion.Linear);
            Assert.AreEqual(expectedUniform, motionExpansion.Uniform);
        }

        [Test]
        public void MotionExpansionMaxDistanceTest()
        {
            // Test values.
            var linear =  new float2(2.0f, 3.0f);
            var uniform = 5.0f;

            // Expectations.
            var expectedMaxDistance = math.length(linear) + uniform;

            var motionExpansion = new MotionExpansion()
            {
                Linear = linear,
                Uniform = uniform
            };

            Assert.AreEqual(expectedMaxDistance, motionExpansion.MaxDistance);
        }

        [Test]
        public void MotionExpansionSweepAabbTest()
        {
            // Test values.
            var linear =  new float2(2.0f, 3.0f);
            var uniform = 5.0f;
            var aabb = new Aabb() { Min = new float2(-5.0f, -10.0f), Max = new float2(40.0f, 30.0f) };

            // Expectations.
            var expectedAabb = new Aabb
            {
                Min = math.min(aabb.Min, aabb.Min + linear) - uniform,
                Max = math.max(aabb.Max, aabb.Max + linear) + uniform
            };

            var motionExpansion = new MotionExpansion()
            {
                Linear = linear,
                Uniform = uniform
            };
                       
            var resultAabb = motionExpansion.ExpandAabb(aabb);

            Assert.AreEqual(expectedAabb.Min, resultAabb.Min);
            Assert.AreEqual(expectedAabb.Max, resultAabb.Max);
        }
    }
}
