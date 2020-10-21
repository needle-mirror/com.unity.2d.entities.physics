using UnityEngine;
using NUnit.Framework;

using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.U2D.Entities.Physics;
using Unity.U2D.Entities.Physics.Authoring;

[TestFixture]
class ClassicCompoundColliderConversionTests : BaseClassicColliderConversionFixture
{
    private BoxGeometry m_BoxGeometry;
    private CapsuleGeometry m_CapsuleGeometry;
    private CircleGeometry m_CircleGeometry;
    private PolygonGeometry m_PolygonGeometry;
    private NativeArray<float2> m_Vertices;
    private NativeArray<int> m_GiftWrappedIndices;

    [SetUp]
    protected override void Setup()
    {
        base.Setup();

        m_BoxGeometry = new BoxGeometry
        {
            Size = new float2(0.01f, 120.40f),
            Center = new float2(-10.10f, 10.12f),
        };

        m_CapsuleGeometry = new CapsuleGeometry
        {
            Vertex0 = new float2(0f, 3f),
            Vertex1 = new float2(0f, -2f),
            Radius = 1.5f
        };

        m_CircleGeometry = new CircleGeometry
        {
            Center = new float2(-10.10f, 10.12f),
            Radius = 3.0f
        };

        m_Vertices = new NativeArray<float2>(
            new float2[]
            {
                new float2(-1f, -2f),
                new float2(2f, -3f),
                new float2(4f, 5f),
                new float2(-6f, 7f)
            },
            Allocator.Persistent
            );

        m_GiftWrappedIndices = new NativeArray<int>(
            new int[]
            {
                2, 3, 0, 1
            },
            Allocator.Persistent
            );

        Assert.IsTrue(m_Vertices.Length >= 3, "Test array must contain at least 3 points.");
        Assert.AreEqual(m_Vertices.Length, m_GiftWrappedIndices.Length, "Test array lengths do not match.");

        m_PolygonGeometry = new PolygonGeometry
        {
            Vertices = m_Vertices,
            BevelRadius = 0.0f
        };
    }

    [TearDown]
    protected override void TearDown()
    {
        m_GiftWrappedIndices.Dispose();
        m_Vertices.Dispose();
        base.TearDown();
    }

    private Vector2[] ToVector2Array()
    {
        Assert.IsTrue(m_Vertices.Length >= 3);

        var output = new Vector2[m_Vertices.Length];
        for(var i = 0; i < m_Vertices.Length; ++i)
        {
            var point = m_Vertices[i];
            output[i] = new Vector2(point.x, point.y);
        }
        return output;
    }

    [Test]
    public void CompoundCollider_BadTransformScale_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            // Create Root with Bad Transform Scale.
            Root = new GameObject();
            Root.transform.localScale = Vector3.zero;

            // Rigidbody.
            {
                var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
                rigidbody.bodyType = RigidbodyType2D.Static;
            }

            // Box.
            {
                var boxCollider = CreateClassicComponent<BoxCollider2D>(Root);
                boxCollider.offset = m_BoxGeometry.Center;
                boxCollider.size = m_BoxGeometry.Size;
            }

            // Capsule.
            {
                var capsuleCollider = CreateClassicComponent<CapsuleCollider2D>(Root);
                capsuleCollider.offset = (m_CapsuleGeometry.Vertex0 + m_CapsuleGeometry.Vertex1) * 0.5f;
                capsuleCollider.size = new Vector2(m_CapsuleGeometry.Radius * 2f, math.distance(m_CapsuleGeometry.Vertex0, m_CapsuleGeometry.Vertex1) + m_CapsuleGeometry.Radius * 2f);
            }

            // Circle.
            {
                var circleCollider = CreateClassicComponent<CircleCollider2D>(Root);
                circleCollider.offset = m_CircleGeometry.Center;
                circleCollider.radius = m_CircleGeometry.Radius;
            }

            // Polygon.
            {
                var polygonCollider = CreateClassicComponent<PolygonCollider2D>(Root);
                polygonCollider.SetPath(0, ToVector2Array());
            }
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
    public void CompoundCollider_ChildrenSameLevel_ConversionTest()
    {
        // Set-up the hierarchy.
        {
            Root = new GameObject();

            // Rigidbody.
            {
                var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
                rigidbody.bodyType = RigidbodyType2D.Static;
            }

            // Box.
            {
                var boxCollider = CreateClassicComponent<BoxCollider2D>(Root);
                boxCollider.offset = m_BoxGeometry.Center;
                boxCollider.size = m_BoxGeometry.Size;
            }

            // Capsule.
            {
                var capsuleCollider = CreateClassicComponent<CapsuleCollider2D>(Root);
                capsuleCollider.offset = (m_CapsuleGeometry.Vertex0 + m_CapsuleGeometry.Vertex1) * 0.5f;
                capsuleCollider.size = new Vector2(m_CapsuleGeometry.Radius * 2f, math.distance(m_CapsuleGeometry.Vertex0, m_CapsuleGeometry.Vertex1) + m_CapsuleGeometry.Radius * 2f);
            }

            // Circle.
            {
                var circleCollider = CreateClassicComponent<CircleCollider2D>(Root);
                circleCollider.offset = m_CircleGeometry.Center;
                circleCollider.radius = m_CircleGeometry.Radius;
            }

            // Polygon.
            {
                var polygonCollider = CreateClassicComponent<PolygonCollider2D>(Root);
                polygonCollider.SetPath(0, ToVector2Array());
            }
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Composite, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Compound, colliderBlob.Value.ColliderType);

                ref var compoundCollider = ref colliderBlob.GetColliderRef<PhysicsCompoundCollider>();

                Assert.AreEqual(4, compoundCollider.NumChildren);

                ValidateCompoundChildren(ref compoundCollider, Matrix4x4.identity);
            });
    }

    [Test]
    public void CompoundCollider_ChildrenNestedLevel_ConversionTest(
        [Values(0f, -2f, 10f, -323f, 648f)] float translateX,
        [Values(0f, 4f, -38f, 483f, -843f)] float translateY)
    {
        Matrix4x4 invColliderTransform;

        // Set-up the hierarchy.
        {
            Root = new GameObject();

            var translation = new float2(translateX, translateY);

            // Rigidbody.
            {
                var rigidbody = CreateClassicComponent<Rigidbody2D>(Root);
                rigidbody.bodyType = RigidbodyType2D.Static;
            }

            // Box.
            {
                var boxCollider = CreateClassicComponent<BoxCollider2D>(CreateChild(translation));
                boxCollider.offset = m_BoxGeometry.Center;
                boxCollider.size = m_BoxGeometry.Size;
            }

            // Capsule.
            {
                var capsuleCollider = CreateClassicComponent<CapsuleCollider2D>(CreateChild(translation));
                capsuleCollider.offset = (m_CapsuleGeometry.Vertex0 + m_CapsuleGeometry.Vertex1) * 0.5f;
                capsuleCollider.size = new Vector2(m_CapsuleGeometry.Radius * 2f, math.distance(m_CapsuleGeometry.Vertex0, m_CapsuleGeometry.Vertex1) + m_CapsuleGeometry.Radius * 2f);
            }

            // Circle.
            {
                var circleCollider = CreateClassicComponent<CircleCollider2D>(CreateChild(translation));
                circleCollider.offset = m_CircleGeometry.Center;
                circleCollider.radius = m_CircleGeometry.Radius;
            }

            // Polygon.
            {
                var polygonCollider = CreateClassicComponent<PolygonCollider2D>(CreateChild(translation));
                polygonCollider.SetPath(0, ToVector2Array());

                // NOTE: All these colliders use the same transform so grab it here.
                invColliderTransform = ConversionUtilities.GetColliderLocalToWorld(polygonCollider).inverse;
            }
        }

        // Run the conversion test and validate.
        RunConversionTest(
            (BlobAssetReference<Unity.U2D.Entities.Physics.Collider> colliderBlob) =>
            {
                // Sanity.
                Assert.AreEqual(CollisionType.Composite, colliderBlob.Value.CollisionType);
                Assert.AreEqual(ColliderType.Compound, colliderBlob.Value.ColliderType);

                ref var compoundCollider = ref colliderBlob.GetColliderRef<PhysicsCompoundCollider>();

                Assert.AreEqual(4, compoundCollider.NumChildren);

                ValidateCompoundChildren(ref compoundCollider, invColliderTransform);
            });
    }

    private float2 AdjustSubColliderPoint(Matrix4x4 inverseColliderTransform, float2 point)
    {
        var point3 = inverseColliderTransform.MultiplyPoint(new float3(point, 0f));
        return new float2(point3.x, point3.y);
    }

    private unsafe void ValidateCompoundChildren(ref PhysicsCompoundCollider compoundCollider, Matrix4x4 invColliderTransform)
    {
        // Validate each child.
        for(var childIndex = 0; childIndex < compoundCollider.NumChildren; ++childIndex)
        {
            ref var childCollider = ref compoundCollider.Children[childIndex];

            switch (childCollider.Collider->ColliderType)
            {
                case ColliderType.Box:
                    {
                        var actualGeometry = ((PhysicsBoxCollider*)childCollider.Collider)->Geometry;

                        PhysicsAssert.AreEqual(m_BoxGeometry.Size, actualGeometry.Size, Epsilon, "Invalid Compound BoxGeometry.Size");
                        PhysicsAssert.AreEqual(m_BoxGeometry.Center, AdjustSubColliderPoint(invColliderTransform, actualGeometry.Center), Epsilon, "Invalid Compound BoxGeometry.Center");
                        break;
                    }

                case ColliderType.Capsule:
                    {
                        var actualGeometry = ((PhysicsCapsuleCollider*)childCollider.Collider)->Geometry;

                        PhysicsAssert.AreEqual(m_CapsuleGeometry.Vertex0, AdjustSubColliderPoint(invColliderTransform, actualGeometry.Vertex0), Epsilon, "Invalid Compound CapsuleGeometry.Vertex0");
                        PhysicsAssert.AreEqual(m_CapsuleGeometry.Vertex1, AdjustSubColliderPoint(invColliderTransform, actualGeometry.Vertex1), Epsilon, "Invalid Compound CapsuleGeometry.Vertex1");
                        Assert.AreEqual(m_CapsuleGeometry.Radius, actualGeometry.Radius, Epsilon, "Invalid Compound CapsuleGeometry.Radius");
                        break;
                    }

                case ColliderType.Circle:
                    {
                        var actualGeometry = ((PhysicsCircleCollider*)childCollider.Collider)->Geometry;

                        PhysicsAssert.AreEqual(m_CircleGeometry.Center, AdjustSubColliderPoint(invColliderTransform, actualGeometry.Center), Epsilon, "Invalid Compound CircleGeometry.Center");
                        Assert.AreEqual(m_CircleGeometry.Radius, actualGeometry.Radius, Epsilon, "Invalid Compound CircleGeometry.Radius");
                        break;
                    }

                case ColliderType.Polygon:
                    {
                        var colliderPtr = (PhysicsPolygonCollider*)childCollider.Collider;

                        var expectedVertices = m_PolygonGeometry.Vertices;
                        var actualVertexCount = colliderPtr->VertexCount;
                        Assert.AreEqual(expectedVertices.Length, actualVertexCount, "Invalid Compound PolygonGeometry.VertexCount");

                        var actualVertices = colliderPtr->Vertices;
                        for(var i = 0; i < actualVertexCount; ++i)
                        {
                            PhysicsAssert.AreEqual(expectedVertices[i], AdjustSubColliderPoint(invColliderTransform, actualVertices[m_GiftWrappedIndices[i]]), Epsilon, "Invalid Compound PolygonGeometry.Vertex");
                        }
                        break;
                    }

                case ColliderType.Invalid:
                case ColliderType.Compound:
                default:
                    Assert.Fail("Invalid compound child collider type found.");
                    break;
            }
        }
    }
}
