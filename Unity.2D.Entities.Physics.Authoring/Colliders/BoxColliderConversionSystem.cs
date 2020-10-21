using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [UpdateBefore(typeof(RigidbodyConversionSystem))]
    [ConverterVersion("2d", 1)]
    internal sealed class BoxColliderConversionSystem : GameObjectConversionSystem
    {
        private ColliderConversionSystem m_ColliderConversionSystem;

        protected override void OnCreate()
        {
            base.OnCreate();
            m_ColliderConversionSystem = World.GetOrCreateSystem<ColliderConversionSystem>();
        }

        protected override void OnUpdate()
        {
            Entities.ForEach((UnityEngine.BoxCollider2D collider) =>
            {
                // Convert the collider if it's valid.
                if (ConversionUtilities.CanConvertCollider(collider))
                {
                    try
                    {
                        var lossyScale = new float3(collider.transform.lossyScale).xy;
                        if (math.any(!math.isfinite(lossyScale)) || math.any(lossyScale <= 0.0f))
                            throw new ArgumentException("Transform XY scale cannot be zero or Infinite/NaN.", "Transform XY scale.");

                        var localToWorld = ConversionUtilities.GetColliderLocalToWorld(collider);
                        var size = collider.size;
                    
                        var geometry = new BoxGeometry
                        {
                            Center = new float3(localToWorld.MultiplyPoint(collider.offset)).xy,
                            Size = new float2(size.x * lossyScale.x, size.y * lossyScale.y),
                            Angle = PhysicsMath.ZRotationFromQuaternion(localToWorld.rotation),
                            BevelRadius = math.max(collider.edgeRadius, PhysicsSettings.Constants.MinimumConvexRadius),
                        };

                        geometry.Validate();

                        var colliderBlob = PhysicsBoxCollider.Create(
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
