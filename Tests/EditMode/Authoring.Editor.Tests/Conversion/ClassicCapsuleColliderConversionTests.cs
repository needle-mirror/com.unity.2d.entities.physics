using UnityEngine;
using NUnit.Framework;

using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Entities.Physics;

[TestFixture]
class ClassicCapsuleColliderConversionTests : BaseClassicColliderConversionFixture
{
    [Test]
    public void CapsuleCollider_BadTransformScale_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            // Create Root with Bad Transform Scale.
            Root = new GameObject();
            Root.transform.localScale = Vector3.zero;

            CreateClassicComponent<CapsuleCollider2D>(Root);
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
    public void CapsuleCollider_Vertical_Root_ConversionTest()
    {
        var expectedOffset = new float2(3f, -4f);
        var expectedSize = new float2(5f, 10f);

        // Set-up the hierarchy.
        {
            Root = new GameObject();

            var collider = CreateClassicComponent<CapsuleCollider2D>(Root);
            collider.direction = CapsuleDirection2D.Vertical;
            collider.offset = expectedOffset;
            collider.size = expectedSize;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Capsule, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                var actualOffset = (collider.Vertex0 + collider.Vertex1) * 0.5f;
                var actualSize = new float2(collider.Radius * 2f, math.distance(collider.Vertex0, collider.Vertex1) + collider.Radius * 2f);

                PhysicsAssert.AreEqual(expectedOffset, actualOffset, Epsilon);
                PhysicsAssert.AreEqual(expectedSize, actualSize, Epsilon);
            });
    }

    [Test]
    public void CapsuleCollider_Horizontal_Root_ConversionTest()
    {
        var expectedOffset = new float2(3f, -4f);
        var expectedSize = new float2(10f, 5f);

        // Set-up the hierarchy.
        {
            Root = new GameObject();

            var collider = CreateClassicComponent<CapsuleCollider2D>(Root);
            collider.direction = CapsuleDirection2D.Horizontal;
            collider.offset = expectedOffset;
            collider.size = expectedSize;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Capsule, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                var actualOffset = (collider.Vertex0 + collider.Vertex1) * 0.5f;
                var actualSize = new float2(math.distance(collider.Vertex0, collider.Vertex1) + collider.Radius * 2f, collider.Radius * 2f);

                PhysicsAssert.AreEqual(expectedOffset, actualOffset, Epsilon);
                PhysicsAssert.AreEqual(expectedSize, actualSize, Epsilon);
            });
    }

    [Test]
    public void CapsuleCollider_Vertical_Child_ConversionTest(
        [Values(0f, -2f, 10f, -323f, 648f)] float translateX,
        [Values(0f, 4f, -38f, 483f, -843f)] float translateY,
        [Values(0, 45f, -30f, 140f, -280f)] float rotateZ)
    {
        var wantedOffset = new float2(6f, -2f);

        // Note: We calculate the collider offset here in body-space so we don't need to rotate it.
        var expectedOffset = PhysicsMath.mul(new PhysicsTransform(new float2(translateX, translateY), 0f), wantedOffset);
        var expectedSize = new float2(5f, 10f);

        // Set-up the hierarchy.
        {
            CreateHierarchy<Rigidbody2D, CapsuleCollider2D>();

            // Transform the hierarchy.
            Root.transform.rotation = Quaternion.Euler(0f, 0f, rotateZ);
            Child.transform.localPosition = new float3(new float2(translateX, translateY), 0f);

            var collider = Child.GetComponent<CapsuleCollider2D>();
            collider.direction = CapsuleDirection2D.Vertical;
            collider.offset = wantedOffset;
            collider.size = expectedSize;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Capsule, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                var actualOffset = (collider.Vertex0 + collider.Vertex1) * 0.5f;
                var actualSize = new float2(collider.Radius * 2f, math.distance(collider.Vertex0, collider.Vertex1) + collider.Radius * 2f);

                PhysicsAssert.AreEqual(expectedOffset, actualOffset, Epsilon);
                PhysicsAssert.AreEqual(expectedSize, actualSize, Epsilon);
            });
    }

    [Test]
    public void CapsuleCollider_Horizontal_Child_ConversionTest(
        [Values(0f, -2f, 10f, -323f, 648f)] float translateX,
        [Values(0f, 4f, -38f, 483f, -843f)] float translateY,
        [Values(0, 45f, -30f, 140f, -280f)] float rotateZ)
    {
        var wantedOffset = new float2(6f, -2f);

        // Note: We calculate the collider offset here in body-space so we don't need to rotate it.
        var expectedOffset = PhysicsMath.mul(new PhysicsTransform(new float2(translateX, translateY), 0f), wantedOffset);
        var expectedSize = new float2(10f, 5f);

        // Set-up the hierarchy.
        {
            CreateHierarchy<Rigidbody2D, CapsuleCollider2D>();

            // Transform the hierarchy.
            Root.transform.rotation = Quaternion.Euler(0f, 0f, rotateZ);
            Child.transform.localPosition = new float3(new float2(translateX, translateY), 0f);

            var collider = Child.GetComponent<CapsuleCollider2D>();
            collider.direction = CapsuleDirection2D.Horizontal;
            collider.offset = wantedOffset;
            collider.size = expectedSize;
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Capsule, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsCapsuleCollider>();

                var actualOffset = (collider.Vertex0 + collider.Vertex1) * 0.5f;
                var actualSize = new float2(math.distance(collider.Vertex0, collider.Vertex1) + collider.Radius * 2f, collider.Radius * 2f);

                PhysicsAssert.AreEqual(expectedOffset, actualOffset, Epsilon);
                PhysicsAssert.AreEqual(expectedSize, actualSize, Epsilon);
            });
    }

}
