using System;
using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    public struct PhysicsJointData
    {
    }
    
    // A runtime joint instance, attached to specific rigid bodies
    public struct Joint
    {
        public BlobAssetReference<PhysicsJointData> JointData;
        public PhysicsBody.IndexPair BodyPair;
        
        // If non-zero, allows these bodies to collide
        public int EnableCollision;

        // The entity that contained the component which created this joint
        // Note, this isn't necessarily an entity with a rigid body, as a pair
        // of bodies can have an arbitrary number of constraints, but only one
        // instance of a particular component type per entity
        public Entity Entity;
    }
}
