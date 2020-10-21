using UnityEngine;
using NUnit.Framework;

using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Entities.Physics;

[TestFixture]
class ClassicRigidbodyConversionTests : AuthoringTestFixture
{
    [Test]
    public void DynamicRigidbody_ConversionTest()
    {
        var expectedMass = 7f;
        var expectedLinearVelocity = new float2(5f, -3f);
        var expectedAngularVelocity = 4f;
        var expectedLinearDamping = 0.6f;
        var expectedAngularDamping = 0.3f;
        var expectedGravityScale = 1.2f;

        // Set-up the hierarchy.
        {
            Root = new GameObject();
            var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
            rigidbody.bodyType = RigidbodyType2D.Dynamic;
            rigidbody.mass = expectedMass;
            rigidbody.velocity = expectedLinearVelocity;
            rigidbody.angularVelocity = expectedAngularVelocity;
            rigidbody.drag = expectedLinearDamping;
            rigidbody.angularDrag = expectedAngularDamping;
            rigidbody.gravityScale = expectedGravityScale;
        }

        // Run the GameObject Conversion.
        RunConversion(Root);

        // Verify the Conversion.
        using (
            var query = EntityManager.CreateEntityQuery(
            typeof(PhysicsMass), 
            typeof(PhysicsVelocity), 
            typeof(PhysicsDamping),
            typeof(PhysicsGravity) 
            ))
        {        
            Assert.AreEqual(1, query.CalculateEntityCount(), "Was expecting a single entity!");

            // Fetch the entity.
            using (var entities = query.ToEntityArray(Allocator.TempJob))
            {
                var entity = entities[0];

                // Sanity check.
                Assert.IsFalse(HasComponent<PhysicsColliderBlob>(entity));

                // Mass.
                {
                    Assert.IsTrue(HasComponent<PhysicsMass>(entity));
                    var mass = GetComponentData<PhysicsMass>(entity);

                    Assert.AreEqual(expectedMass, mass.GetMass(), Epsilon);
                    PhysicsAssert.AreEqual(new float2(0f), mass.LocalCenterOfMass, Epsilon);
                    Assert.AreEqual(1f, mass.GetInertia(), Epsilon);
                }

                // Velocity.
                {
                    Assert.IsTrue(HasComponent<PhysicsVelocity>(entity));
                    var velocity = GetComponentData<PhysicsVelocity>(entity);

                    PhysicsAssert.AreEqual(expectedLinearVelocity, velocity.Linear, Epsilon);
                    Assert.AreEqual(expectedAngularVelocity, velocity.Angular, Epsilon);
                }

                // Damping.
                {
                    Assert.IsTrue(HasComponent<PhysicsDamping>(entity));
                    var damping = GetComponentData<PhysicsDamping>(entity);

                    Assert.AreEqual(expectedLinearDamping, damping.Linear, Epsilon);
                    Assert.AreEqual(expectedAngularDamping, damping.Angular, Epsilon);
                }

                // Gravity scale.
                {
                    Assert.IsTrue(HasComponent<PhysicsGravity>(entity));
                    var gravity = GetComponentData<PhysicsGravity>(entity);

                    Assert.AreEqual(expectedGravityScale, gravity.Scale, Epsilon);
                }
            }
        }
    }

    [Test]
    public void KinematicRigidbody_ConversionTest()
    {
        var expectedLinearVelocity = new float2(5f, -3f);
        var expectedAngularVelocity = 4f;

        // Set-up the hierarchy.
        {
            Root = new GameObject();
            var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
            rigidbody.bodyType = RigidbodyType2D.Kinematic;
            rigidbody.mass = 7f;
            rigidbody.velocity = expectedLinearVelocity;
            rigidbody.angularVelocity = expectedAngularVelocity;
        }

        // Run the GameObject Conversion.
        RunConversion(Root);

        // Verify the Conversion.
        using (
            var query = EntityManager.CreateEntityQuery(
                new EntityQueryDesc
                {
                    All = new ComponentType[]
                    {
                        ComponentType.ReadOnly<PhysicsMass>(),
                        ComponentType.ReadOnly<PhysicsVelocity>()
                    },
                    None = new ComponentType[]
                    {
                        ComponentType.ReadOnly<PhysicsDamping>(),
                        ComponentType.ReadOnly<PhysicsGravity>(),
                    }
                }))
        {        
            Assert.AreEqual(1, query.CalculateEntityCount(), "Was expecting a single entity!");

            // Fetch the entity.
            using (var entities = query.ToEntityArray(Allocator.TempJob))
            {
                var entity = entities[0];

                // Sanity check.
                Assert.IsFalse(HasComponent<PhysicsColliderBlob>(entity));
                Assert.IsFalse(HasComponent<PhysicsDamping>(entity));
                Assert.IsFalse(HasComponent<PhysicsGravity>(entity));

                // Mass.
                {
                    Assert.IsTrue(HasComponent<PhysicsMass>(entity));
                    var mass = GetComponentData<PhysicsMass>(entity);

                    Assert.AreEqual(0f, mass.GetMass(), Epsilon);
                    PhysicsAssert.AreEqual(new float2(0f), mass.LocalCenterOfMass, Epsilon);
                    Assert.AreEqual(0f, mass.GetInertia(), Epsilon);
                }

                // Velocity.
                {
                    Assert.IsTrue(HasComponent<PhysicsVelocity>(entity));
                    var velocity = GetComponentData<PhysicsVelocity>(entity);

                    PhysicsAssert.AreEqual(expectedLinearVelocity, velocity.Linear, Epsilon);
                    Assert.AreEqual(expectedAngularVelocity, velocity.Angular, Epsilon);
                }
            }
        }
    }

    [Test]
    public void StaticRigidbody_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            Root = new GameObject();
            var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
            rigidbody.bodyType = RigidbodyType2D.Static;
        }

        // Run the GameObject Conversion.
        RunConversion(Root);

        // Verify the Conversion.
        using (
            var query = EntityManager.CreateEntityQuery(
                new EntityQueryDesc
                {
                    Any = new ComponentType[]
                    {
                        ComponentType.ReadOnly<PhysicsMass>(),
                        ComponentType.ReadOnly<PhysicsVelocity>(),
                        ComponentType.ReadOnly<PhysicsDamping>(),
                        ComponentType.ReadOnly<PhysicsGravity>()
                    }
                }))
        {        
            Assert.AreEqual(0, query.CalculateEntityCount(), "Was expecting a single entity!");
        }
    }
}
