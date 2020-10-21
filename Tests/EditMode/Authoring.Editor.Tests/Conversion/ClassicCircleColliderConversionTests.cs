using UnityEngine;
using NUnit.Framework;

using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Entities.Physics;

[TestFixture]
class ClassicCircleColliderConversionTests : BaseClassicColliderConversionFixture
{
    [Test]
    public void CircleCollider_BadTransformScale_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            // Create Root with Bad Transform Scale.
            Root = new GameObject();
            Root.transform.localScale = Vector3.zero;

            CreateClassicComponent<CircleCollider2D>(Root);
        }

        // Run the Conversion.
        Assert.DoesNotThrow(() => { RunConversion(Root); });

        // Verify that no conversion took place.
        using (var query = EntityManager.CreateEntityQuery(typeof(PhysicsColliderBlob)))
        {
            Assert.AreEqual(0, query.CalculateEntityCount(), "Was NOT expecting a PhysicsColliderBlob!");
        }
    }

    [Test]
    public void CircleCollider_Root_ConversionTest()
    {
        var expectedOffset = new float2(3f, -4f);
        var expectedRadius = 1.6f;

        // Set-up the hierarchy.
        {
            Root = new GameObject();

            var collider = CreateClassicComponent<CircleCollider2D>(Root);
            collider.offset = expectedOffset;
            collider.radius = expectedRadius;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Circle, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCircleCollider>();

                PhysicsAssert.AreEqual(expectedOffset, collider.Center, Epsilon);
                Assert.AreEqual(expectedRadius, collider.Radius, Epsilon);
            });
    }

    [Test]
    public void CircleCollider_Child_ConversionTest(
        [Values(0f, -2f, 10f, -323f, 648f)] float translateX,
        [Values(0f, 4f, -38f, 483f, -843f)] float translateY,
        [Values(0, 45f, -30f, 140f, -280f)] float rotateZ)
    {
        var wantedOffset = new float2(6f, -2f);

        // Note: We calculate the collider offset here in body-space so we don't need to rotate it.
        var expectedOffset = PhysicsMath.mul(new PhysicsTransform(new float2(translateX, translateY), 0f), wantedOffset);
        var expectedRadius = 2.5f;

        // Set-up the hierarchy.
        {
            CreateHierarchy<Rigidbody2D, CircleCollider2D>();

            // Transform the hierarchy.
            Root.transform.rotation = Quaternion.Euler(0f, 0f, rotateZ);
            Child.transform.localPosition = new float3(new float2(translateX, translateY), 0f);

            var collider = Child.GetComponent<CircleCollider2D>();
            collider.offset = wantedOffset;
            collider.radius = expectedRadius;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Circle, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCircleCollider>();

                PhysicsAssert.AreEqual(expectedOffset, collider.Center, Epsilon);
                Assert.AreEqual(expectedRadius, collider.Radius, Epsilon);
            });
    }
}
