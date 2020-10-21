using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    // Describes the Collider (Blob) attached to a PhysicsBody.
    // Without this the PhysicsBody cannot collide with anything nor will it be returned by queries.
    public struct PhysicsColliderBlob : IComponentData
    {
        // This collider blob is allowed to be effectively null.
        public BlobAssetReference<Collider> Collider;
    }

    // This component indicates that this entity owns the referenced collider blob.
    // This will cause the referenced blob to be destroyed when the Entity is destroyed.
    public struct PhysicsColliderBlobOwner : ISystemStateComponentData
    {
        public BlobAssetReference<Collider> Collider;
    }
}
