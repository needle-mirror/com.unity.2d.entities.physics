using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class CapsuleColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.CapsuleCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        UnityEngine.Vector3 vertex0;
                        UnityEngine.Vector3 vertex1;
                        float radius;

                        var halfSize = new float2(collider.size) * 0.5f;

                        if (collider.direction == UnityEngine.CapsuleDirection2D.Vertical)
                        {
                            radius = halfSize.x;
                            vertex0 = new UnityEngine.Vector3(0.0f, halfSize.y - radius, 0.0f);
                            vertex1 = new UnityEngine.Vector3(0.0f, -halfSize.y + radius, 0.0f);
                        }
                        else
                        {
                            radius = halfSize.y;
                            vertex0 = new UnityEngine.Vector3(halfSize.x - radius, 0.0f, 0.0f);
                            vertex1 = new UnityEngine.Vector3(-halfSize.x + radius, 0.0f, 0.0f);
                        }

                        // Add offset to capsule.
                        var colliderOffset = (UnityEngine.Vector3)collider.offset;
                        vertex0 += colliderOffset;
                        vertex1 += colliderOffset;

                        var lossyScale = new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);

                        var geometry = new CapsuleGeometry
                        {
                            Vertex0 = new float3(localToWorld.MultiplyPoint(vertex0)).xy,
                            Vertex1 = new float3(localToWorld.MultiplyPoint(vertex1)).xy,
                            Radius = math.max(PhysicsSettings.Constants.MinimumConvexRadius, math.cmax(lossyScale) * radius),
                        };

                        var colliderBlob = PhysicsCapsuleCollider.Create(
                                geometry,
                                ConversionUtilities.GetCollisionFilterFromCollider(collider),
                                ConversionUtilities.GetPhysicsMaterialFromCollider(collider)
                                );

                        // Submit the collider for conversion.
                        m_ColliderConversionSystem.SubmitCollider(collider, ref colliderBlob);
                    }
                    catch(ArgumentException exception)
                    {
                        UnityEngine.Debug.LogWarning($"{collider.name}: {exception.Message}", collider);
                    }
                }
            });
        }
    }
}
