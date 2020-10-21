using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class PolygonColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;
        private List<UnityEngine.Vector2> m_PolygonVertices;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
            m_PolygonVertices = new List<UnityEngine.Vector2>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.PolygonCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        // Only single paths with no more than the maximum allowed vertex allowed.
                        // NOTE: Until we implement a convex polygon decomposition, only the convex hull of these points will be used.
                        var colliderPointCount = collider.GetTotalPointCount();
                        if (collider.pathCount != 1 || colliderPointCount > PhysicsPolygonCollider.Constants.MaxVertexCount)
                            return;

                        var lossyScale =new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);

                        UnityEngine.Vector3 offset =  collider.offset;
                        collider.GetPath(0, m_PolygonVertices);

                        var vertices = new NativeArray<float2>(colliderPointCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
                        for (var i = 0; i < colliderPointCount; ++i)
                        {
                            var vertex = localToWorld.MultiplyPoint(offset + (UnityEngine.Vector3)m_PolygonVertices[i]);
                            vertices[i] = new float2(vertex.x, vertex.y);
                        }
                   
                        var geometry = new PolygonGeometry
                        {
                            Vertices = vertices,
                            BevelRadius = PhysicsSettings.Constants.MinimumConvexRadius,
                        };

                        var colliderBlob = PhysicsPolygonCollider.Create(
                                geometry,
                                ConversionUtilities.GetCollisionFilterFromCollider(collider),
                                ConversionUtilities.GetPhysicsMaterialFromCollider(collider)
                                );

                        // We finished with the points.
                        vertices.Dispose();

                        // Submit the collider for conversion.
                        m_ColliderConversionSystem.SubmitCollider(collider, ref colliderBlob);
                    }
                    catch(ArgumentException exception)
                    {
                        UnityEngine.Debug.LogWarning($"{collider.name}: {exception.Message}", collider);
                    }
                }
            });

            m_PolygonVertices.Clear();
        }
    }
}
