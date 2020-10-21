using Unity.Collections;
using Unity.Entities;

namespace Unity.U2D.Entities.Physics.Authoring
{
    [AlwaysUpdateSystem]
    [UpdateInGroup(typeof(GameObjectBeforeConversionGroup))]
    [ConverterVersion("2d", 1)]
    internal sealed class ColliderConversionSystem : GameObjectConversionSystem
    {
        private NativeMultiHashMap<Entity, BlobAssetReference<Collider>> RigidbodyToColliderMapping;

        protected override void OnCreate()
        {
            base.OnCreate();
            RigidbodyToColliderMapping = new NativeMultiHashMap<Entity, BlobAssetReference<Collider>>(256, Allocator.Persistent);
        }

        protected override void OnDestroy()
        {
            RigidbodyToColliderMapping.Dispose();
            base.OnDestroy();
        }

        protected override void OnUpdate()
        {
            RigidbodyToColliderMapping.Clear();
        }

        internal void SubmitCollider(
            UnityEngine.Collider2D collider,
            ref BlobAssetReference<Collider> colliderBlob)
        {
            // Fetch any rigidbody we're attached to.
            var rigidbody = collider.attachedRigidbody;

            // Are we attached to a rigidbody?
            if (rigidbody != null)
            {
                // Yes, so add as a mapping to the rigidbody entity for now in-case we're a compound collider.
                RigidbodyToColliderMapping.Add(GetPrimaryEntity(rigidbody), colliderBlob);

                // Declare a dependency between the rigidbody and the collider components.
                DeclareDependency(rigidbody, collider);
                return;
            }

            // No attached Rigidbody2D so add the collider blob onto this Entity.
            // NOTE: This is the implicit static collider case.
            DstEntityManager.AddComponentData(GetPrimaryEntity(collider), new PhysicsColliderBlob { Collider = colliderBlob });
        }

        internal void CreateCollider(Entity rigidbodyEntity)
        {
            var colliderCount = RigidbodyToColliderMapping.CountValuesForKey(rigidbodyEntity);
            if (colliderCount == 0)
                return;

            // Single collider doesn't require a compound collider.
            if (colliderCount == 1)
            {
                var foundColliderBlob = RigidbodyToColliderMapping.TryGetFirstValue(rigidbodyEntity, out BlobAssetReference<Collider> colliderBlob, out NativeMultiHashMapIterator<Entity> ignore);
                SafetyChecks.IsTrue(foundColliderBlob);

                // Add the single collider to the rigidbody entity.
                DstEntityManager.AddComponentData(rigidbodyEntity, new PhysicsColliderBlob { Collider = colliderBlob });

                return;
            }

            // Multiple colliders required a compound collider.
            var childrenArray = new NativeArray<PhysicsCompoundCollider.ColliderBlobInstance>(colliderCount, Allocator.Temp, NativeArrayOptions.UninitializedMemory);
            var childIndex = 0;
            foreach(var colliderBlob in RigidbodyToColliderMapping.GetValuesForKey(rigidbodyEntity))
            {
                childrenArray[childIndex++] = new PhysicsCompoundCollider.ColliderBlobInstance
                {
                    Collider = colliderBlob,
                    // NOTE: Right now the relative pose of the collider with respect to the rigidbody is baked into the shape.
                    // Later, we'll want to remove that and only have its offset (if any) baked into it and use this transform instead.
                    CompoundFromChild = PhysicsTransform.Identity
                };
            }

            // Create the compound collider.
            DstEntityManager.AddComponentData(rigidbodyEntity, new PhysicsColliderBlob { Collider = PhysicsCompoundCollider.Create(childrenArray)});

            // We've finished with the children blobs and array.
            for(var i = 0; i < colliderCount; ++i)
            {
                childrenArray[i].Collider.Dispose();
            }           
            childrenArray.Dispose();
        }
    }
}
