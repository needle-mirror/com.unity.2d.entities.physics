using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class CircleColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.CircleCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        var lossyScale =new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);

                        var geometry = new CircleGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.offset)).xy,
                            Radius = math.clamp(collider.radius *  math.cmax(lossyScale), ConversionUtilities.MinRangeClamp, ConversionUtilities.MaxRangeClamp),
                        };

                        var colliderBlob = PhysicsCircleCollider.Create(
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
