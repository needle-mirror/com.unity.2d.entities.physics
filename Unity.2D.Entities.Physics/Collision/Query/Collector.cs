using Unity.Collections;
using Unity.Entities;

namespace Unity.U2D.Entities.Physics
{
    public interface IQueryResult
    {
        int PhysicsBodyIndex { get; }
        ColliderKey ColliderKey { get; }
        Entity Entity { get; }
        float Fraction { get; }

        bool IsValid { get; }
    }

    struct QueryContext
    {
        int m_IsInitialized;

        public int PhysicsBodyIndex;
        public Entity Entity;
        public PhysicsTransform LocalToWorldTransform;

        public ColliderKey ColliderKey;
        public uint NumColliderKeyBits;

        public QueryContext(int physicsBodyIndex, Entity entity, PhysicsTransform localToWorldTransform)
        {
            PhysicsBodyIndex = physicsBodyIndex;
            Entity = entity;
            LocalToWorldTransform = localToWorldTransform;

            ColliderKey = ColliderKey.Empty;
            NumColliderKeyBits = 0;

            m_IsInitialized = 1;
        }

        static QueryContext Default => new QueryContext
        {
            PhysicsBodyIndex = PhysicsBody.Constants.InvalidBodyIndex,
            Entity = Entity.Null,
            LocalToWorldTransform = PhysicsTransform.Identity,

            ColliderKey = ColliderKey.Empty,
            NumColliderKeyBits = 0,

            m_IsInitialized = 1
        };

        internal void EnsureIsInitialized()
        {
            if (m_IsInitialized == 0)
                this = Default;
        }

        public ColliderKey SetSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var parentColliderKey = ColliderKey;
            parentColliderKey.PopSubKey(NumColliderKeyBits, out var parentKey);

            var colliderKey = new ColliderKey(childSubKeyNumOfBits, childSubKey);
            colliderKey.PushSubKey(NumColliderKeyBits, parentKey);
            return colliderKey;
        }

        public ColliderKey PushSubKey(uint childSubKeyNumOfBits, uint childSubKey)
        {
            var colliderKey = SetSubKey(childSubKeyNumOfBits, childSubKey);
            NumColliderKeyBits += childSubKeyNumOfBits;
            return colliderKey;
        }
    }

    // Interface for collecting hits during a collision query
    public interface ICollector<T> where T : struct, IQueryResult
    {
        // Whether to exit the query. Called after each accepted hit.
        bool EarlyOutOnHit { get; }

        // The maximum fraction of the query within which to check for hits
        // For casts, this is a fraction along the ray
        // For distance queries, this is a distance from the query object
        float MaxFraction { get; }

        // The number of hits that have been collected
        int NumHits { get; }

        // Called when the query hits something
        // Return true to accept the hit, or false to ignore it
        bool AddHit(T hit);
    }

    // A collector which exits the query as soon as any hit is detected.
    public struct AnyHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnHit => true;
        public float MaxFraction { get; }
        public int NumHits => 0;

        public AnyHitCollector(float maxFraction)
        {
            MaxFraction = maxFraction;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            SafetyChecks.IsTrue(hit.Fraction < MaxFraction);
            return true;
        }

        #endregion
    }

    // A collector which stores only the closest hit.
    public struct ClosestHitCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnHit => false;
        public float MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        T m_ClosestHit;
        public T ClosestHit => m_ClosestHit;

        public ClosestHitCollector(float maxFraction)
        {
            MaxFraction = maxFraction;
            m_ClosestHit = default(T);
            NumHits = 0;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            SafetyChecks.IsTrue(hit.Fraction <= MaxFraction);
            MaxFraction = hit.Fraction;
            m_ClosestHit = hit;
            NumHits = 1;
            return true;
        }

        #endregion
    }

    // A collector which stores every hit.
    public struct AllHitsCollector<T> : ICollector<T> where T : struct, IQueryResult
    {
        public bool EarlyOutOnHit => false;
        public float MaxFraction { get; }
        public int NumHits => AllHits.Length;

        public NativeList<T> AllHits;

        public AllHitsCollector(float maxFraction, ref NativeList<T> allHits)
        {
            MaxFraction = maxFraction;
            AllHits = allHits;
        }

        #region ICollector

        public bool AddHit(T hit)
        {
            SafetyChecks.IsTrue(hit.Fraction < MaxFraction);
            AllHits.Add(hit);
            return true;
        }

        #endregion
    }
}
