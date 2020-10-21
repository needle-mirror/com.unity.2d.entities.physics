using UnityEngine;
using NUnit.Framework;

using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Entities.Physics;

[TestFixture]
class ClassicPolygonColliderConversionTests : BaseClassicColliderConversionFixture
{
    private readonly Vector2[] m_Points = new Vector2[]
        {
            new Vector2(-1f, -2f),
            new Vector2(2f, -3f),
            new Vector2(4f, 5f),
            new Vector2(-6f, 7f)
        };

    private readonly int[] m_GiftWrappedIndices = new int[] { 2, 3, 0, 1 };

    [Test]
    public void PolygonCollider_BadTransformScale_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            // Create Root with Bad Transform Scale.
            Root = new GameObject();
            Root.transform.localScale = Vector3.zero;

            CreateClassicComponent<PolygonCollider2D>(Root);
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
    public void PolygonCollider_TestDataSanity_Test()
    {
        Assert.IsTrue(m_Points.Length >= 3);
        Assert.AreEqual(m_Points.Length, m_GiftWrappedIndices.Length);
    }
    
    [Test]
    public void PolygonCollider_Root_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            Root = new GameObject();

            var collider = CreateClassicComponent<PolygonCollider2D>(Root);
            collider.SetPath(0, m_Points);
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Polygon, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsPolygonCollider>();

                Assert.AreEqual(m_Points.Length, collider.VertexCount);
                var vertices = collider.Vertices;
                for(var i = 0; i < m_Points.Length; ++i)
                {
                    PhysicsAssert.AreEqual(m_Points[i], vertices[m_GiftWrappedIndices[i]], Epsilon);
                }
            });
    }

    [Test]
    public void PolygonCollider_Child_ConversionTest(
        [Values(0f, -2f, 10f, -323f, 648f)] float translateX,
        [Values(0f, 4f, -38f, 483f, -843f)] float translateY,
        [Values(0f, 45f, -30f, 140f, -280f)] float rotateZ)
    {
        // Note: We calculate the collider offset here in body-space so we don't need to rotate it.
        var expectedOffset = new Vector2(translateX, translateY);

        // Set-up the hierarchy.
        {
            CreateHierarchy<Rigidbody2D, PolygonCollider2D>();

            // Transform the hierarchy.
            Root.transform.rotation = Quaternion.Euler(0f, 0f, rotateZ);
            Child.transform.localPosition = new float3(new float2(translateX, translateY), 0f);

            var collider = Child.GetComponent<PolygonCollider2D>();
            collider.SetPath(0, m_Points);
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Convex, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Polygon, colliderBlob.Value.ColliderType);

                ref var collider = ref colliderBlob.GetColliderRef<PhysicsPolygonCollider>();

                Assert.AreEqual(m_Points.Length, collider.VertexCount);
                var vertices = collider.Vertices;
                for(var i = 0; i < m_Points.Length; ++i)
                {
                    PhysicsAssert.AreEqual(m_Points[i] + expectedOffset, vertices[m_GiftWrappedIndices[i]], Epsilon);
                }
            });
    }
}
