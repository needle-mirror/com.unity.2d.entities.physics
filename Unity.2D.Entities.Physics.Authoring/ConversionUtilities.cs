using UnityEngine;
using Unity.Entities;

namespace Unity.U2D.Entities.Physics.Authoring
{
	internal static class ConversionUtilities
	{
        public const float MinRangeClamp = 0.0001f;
        public const float MaxRangeClamp = 1000000.0f;

        // Create an equivalent of a classic 2D physics material.
        public static PhysicsMaterial GetPhysicsMaterialFromCollider(Collider2D collider)
        {
            // Fetch the collider material.
            var classicMaterial = collider.sharedMaterial;
            if (classicMaterial == null)
            {
                // Fetch the attached body material.
                var attachedRigidbody = collider.attachedRigidbody;
                if (attachedRigidbody != null)
                {
                    classicMaterial = attachedRigidbody.sharedMaterial;
                }
            }

            var material = PhysicsMaterial.Default;
            
            // Set trigger flag if required.
            if (collider.isTrigger)
                material.Flags |= PhysicsMaterial.MaterialFlags.IsTrigger;      

            // Convert any classic material.
            if (classicMaterial != null)
            {
                material.Friction = classicMaterial.friction;
                material.Restitution = classicMaterial.bounciness;
            }

            return material;
        }

        // Create an equivalent of a classic 2D physics collision layer mask.
        public static CollisionFilter GetCollisionFilterFromCollider(Collider2D collider)
        {
            var layer = collider.gameObject.layer;
            var collisionMask = Physics2D.GetLayerCollisionMask(layer);

            return new CollisionFilter
            {
                BelongsTo = (uint)(1 << layer),
                CollidesWith = (uint)collisionMask
            };
        }

        // Get collider local to world as defined relative to any attached Rigidbody2D.
        public static Matrix4x4 GetColliderLocalToWorld(Collider2D collider)
        {
            // If no attached rigidbody or we're attached to a rigidbody but it's on the same GameObject
            // then we simply use identity as the relative transform.
            var attachedRigidbody = collider.attachedRigidbody;
            if (attachedRigidbody == null || attachedRigidbody.gameObject == collider.gameObject)
            {
                return Matrix4x4.identity;
            }

            // Calculate relative to the attached rigidbody.
            var rigidbodyTransform = attachedRigidbody.transform;
            var bodyInverseRotation = Quaternion.Inverse(PhysicsMath.ZQuaternionFromQuaternion(rigidbodyTransform.rotation));
            var bodyInversePosition = bodyInverseRotation * -rigidbodyTransform.position;
            var localToWorld = collider.transform.localToWorldMatrix;
            return Matrix4x4.TRS(bodyInversePosition, bodyInverseRotation, Vector3.one) * localToWorld;
        }

        // Add or Set a component.
        public static void AddOrSetComponent<T>(this EntityManager manager, Entity entity, T value)
            where T : struct, IComponentData
        {
            if (!manager.HasComponent<T>(entity))
            {
                manager.AddComponentData(entity, value);
                return;
            }

            if (!TypeManager.IsZeroSized(TypeManager.GetTypeIndex<T>()))
            {
                manager.SetComponentData(entity, value);
            }
        }

        public static bool CanConvertCollider(Collider2D collider)
        {
            // Cannot convert if we're attached to a Rigidbody2D that isn't Simulated.
            var attachedRigidbody = collider.attachedRigidbody;
            if (attachedRigidbody != null && !attachedRigidbody.simulated)
                return false;

            // Cannot convert if the collider isn't enabled or is using a CompositeCollider2D.
            if (!collider.enabled || collider.usedByComposite)
                return false;

            // If this is a prefab then convert it.
            if (IsPrefab(collider.gameObject))
                return true;

            // Now we've discounted it being a prefab, ignore if it's not active or enabled.
            return collider.isActiveAndEnabled;
        }

        // Check if a GameObject is a prefab.
        public static bool IsPrefab(GameObject gameObject)
        {
            return !gameObject.scene.IsValid();
        }
	}
}
