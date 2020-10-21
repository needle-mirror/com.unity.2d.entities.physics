using Unity.Collections;

namespace Unity.U2D.Entities.Physics
{
    public interface IQueryable
    {
        #region Bounding Box

        // Calculate an axis aligned bounding box around the object, in local space.
        Aabb CalculateAabb();

        // Calculate an axis aligned bounding box around the object, in the given space.
        Aabb CalculateAabb(PhysicsTransform transform);

        #endregion

        #region Overlap Point

        // Check a point against the object.
        // Returns true if it hits.
        bool OverlapPoint(OverlapPointInput input);

        // Check a point against the object.
        // Return true if it hits, with details of the overlap hit in "hit".
        bool OverlapPoint(OverlapPointInput input, out OverlapPointHit hit);

        // Check a point against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool OverlapPoint(OverlapPointInput input, ref NativeList<OverlapPointHit> allHits);

        // Check a point against the object.
        // Return true if it hits, with details stored in the collector implementation.
        bool OverlapPoint<T>(OverlapPointInput input, ref T collector) where T : struct, ICollector<OverlapPointHit>;

        #endregion

        #region Overlap Collider.

        // Check a collider against the object.
        // Returns true if it hits.
        bool OverlapCollider(OverlapColliderInput input);

        // Check a collider against the object.
        // Return true if it hits, with details of the overlap hit in "hit".
        bool OverlapCollider(OverlapColliderInput input, out OverlapColliderHit hit);

        // Check a collider against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool OverlapCollider(OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits);

        // Check a collider against the object.
        // Return true if it hits, with details stored in the collector implementation.
        bool OverlapCollider<T>(OverlapColliderInput input, ref T collector) where T : struct, ICollector<OverlapColliderHit>;

        #endregion

        #region Cast Ray

        // Cast a ray against the object.
        // Return true if it hits.
        bool CastRay(RaycastInput input);

        // Cast a ray against the object.
        // Return true if it hits, with details of the closest hit in "closestHit".
        bool CastRay(RaycastInput input, out RaycastHit closestHit);

        // Cast a ray against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool CastRay(RaycastInput input, ref NativeList<RaycastHit> allHits);

        // Generic ray cast.
        // Return true if it hits, with details stored in the collector implementation.
        bool CastRay<T>(RaycastInput input, ref T collector) where T : struct, ICollector<RaycastHit>;

        #endregion
        
        #region Cast Collider

        // Cast a collider against the object.
        // Return true if it hits.
        bool CastCollider(ColliderCastInput input);

        // Cast a collider against the object.
        // Return true if it hits, with details of the closest hit in "closestHit".
        bool CastCollider(ColliderCastInput input, out ColliderCastHit closestHit);

        // Cast a collider against the object.
        // Return true if it hits, with details of every hit in "allHits".
        bool CastCollider(ColliderCastInput input, ref NativeList<ColliderCastHit> allHits);

        // Generic collider cast.
        // Return true if it hits, with details stored in the collector implementation.
        bool CastCollider<T>(ColliderCastInput input, ref T collector) where T : struct, ICollector<ColliderCastHit>;

        #endregion
        
        #region Point Distance

        // Calculate the distance from a point to the object.
        // Return true if there are any hits.
        bool CalculateDistance(PointDistanceInput input);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details of the closest hit in "closestHit".
        bool CalculateDistance(PointDistanceInput input, out DistanceHit closestHit);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details of every hit in "allHits".
        bool CalculateDistance(PointDistanceInput input, ref NativeList<DistanceHit> allHits);

        // Calculate the distance from a point to the object.
        // Return true if there are any hits, with details stored in the collector implementation.
        bool CalculateDistance<T>(PointDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>;

        #endregion
        
        #region Collider Distance

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits.
        bool CalculateDistance(ColliderDistanceInput input);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details of the closest hit in "closestHit".
        bool CalculateDistance(ColliderDistanceInput input, out DistanceHit closestHit);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details of every hit in "allHits".
        bool CalculateDistance(ColliderDistanceInput input, ref NativeList<DistanceHit> allHits);

        // Calculate the distance from a collider to the object.
        // Return true if there are any hits, with details stored in the collector implementation.
        bool CalculateDistance<T>(ColliderDistanceInput input, ref T collector) where T : struct, ICollector<DistanceHit>;
        
        #endregion
    }

    public struct IgnoreHit
    {
        public IgnoreHit(int ignoreBodyIndex)
        {
            IgnoreBodyIndex = ignoreBodyIndex;
            IgnoreBody = true;
        }

        public int IgnoreBodyIndex { get; private set; }
        public bool IgnoreBody { get; private set; }
    }

    public interface IQueryInput
    {
        IgnoreHit Ignore { get; set; }
    }

    // Wrappers around generic IQueryable queries
    static class QueryWrappers
    {
        #region Overlap Point

        public static bool OverlapPoint<T>(ref T target, OverlapPointInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<OverlapPointHit>(1.0f);
            return target.OverlapPoint(input, ref collector);
        }

        public static bool OverlapPoint<T>(ref T target, OverlapPointInput input, out OverlapPointHit closestHit) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<OverlapPointHit>(1.0f);
            if (target.OverlapPoint(input, ref collector))
            {
                closestHit = collector.ClosestHit;
                return true;
            }

            closestHit = new OverlapPointHit();
            return false;
        }

        public static bool OverlapPoint<T>(ref T target, OverlapPointInput input, ref NativeList<OverlapPointHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<OverlapPointHit>(1.0f, ref allHits);
            return target.OverlapPoint(input, ref collector);
        }
  
        #endregion

        #region Overlap Collider

        public static bool OverlapCollider<T>(ref T target, OverlapColliderInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<OverlapColliderHit>(1.0f);
            return target.OverlapCollider(input, ref collector);
        }

        public static bool OverlapCollider<T>(ref T target, OverlapColliderInput input, out OverlapColliderHit closestHit) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<OverlapColliderHit>(1.0f);
            if (target.OverlapCollider(input, ref collector))
            {
                closestHit = collector.ClosestHit;
                return true;
            }

            closestHit = new OverlapColliderHit();
            return false;
        }

        public static bool OverlapCollider<T>(ref T target, OverlapColliderInput input, ref NativeList<OverlapColliderHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<OverlapColliderHit>(1.0f, ref allHits);
            return target.OverlapCollider(input, ref collector);
        }
  
        #endregion

        #region Ray Casts

        public static bool RayCast<T>(ref T target, RaycastInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<RaycastHit>(1.0f);
            return target.CastRay(input, ref collector);
        }

        public static bool RayCast<T>(ref T target, RaycastInput input, out RaycastHit closestHit) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<RaycastHit>(1.0f);
            if (target.CastRay(input, ref collector))
            {
                closestHit = collector.ClosestHit;
                return true;
            }

            closestHit = new RaycastHit();
            return false;
        }

        public static bool RayCast<T>(ref T target, RaycastInput input, ref NativeList<RaycastHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<RaycastHit>(1.0f, ref allHits);
            return target.CastRay(input, ref collector);
        }

        #endregion

        #region Collider Casts

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<ColliderCastHit>(1.0f);
            return target.CastCollider(input, ref collector);
        }

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input, out ColliderCastHit result) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<ColliderCastHit>(1.0f);
            if (target.CastCollider(input, ref collector))
            {
                result = collector.ClosestHit;
                return true;
            }

            result = new ColliderCastHit();
            return false;
        }

        public static bool ColliderCast<T>(ref T target, ColliderCastInput input, ref NativeList<ColliderCastHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<ColliderCastHit>(1.0f, ref allHits);
            return target.CastCollider(input, ref collector);
        }

        #endregion

        #region Point distance queries

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<DistanceHit>(input.MaxDistance);
            return target.CalculateDistance(input, ref collector);
        }

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input, out DistanceHit result) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<DistanceHit>(input.MaxDistance);
            if (target.CalculateDistance(input, ref collector))
            {
                result = collector.ClosestHit;
                return true;
            }

            result = new DistanceHit();
            return false;
        }

        public static bool CalculateDistance<T>(ref T target, PointDistanceInput input, ref NativeList<DistanceHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<DistanceHit>(input.MaxDistance, ref allHits);
            return target.CalculateDistance(input, ref collector);
        }

        #endregion

        #region Collider distance queries

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input) where T : struct, IQueryable
        {
            var collector = new AnyHitCollector<DistanceHit>(input.MaxDistance);
            return target.CalculateDistance(input, ref collector);
        }

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input, out DistanceHit result) where T : struct, IQueryable
        {
            var collector = new ClosestHitCollector<DistanceHit>(input.MaxDistance);
            if (target.CalculateDistance(input, ref collector))
            {
                result = collector.ClosestHit;
                return true;
            }

            result = new DistanceHit();
            return false;
        }

        public static bool CalculateDistance<T>(ref T target, ColliderDistanceInput input, ref NativeList<DistanceHit> allHits) where T : struct, IQueryable
        {
            var collector = new AllHitsCollector<DistanceHit>(input.MaxDistance, ref allHits);
            return target.CalculateDistance(input, ref collector);
        }

        #endregion
    }
}
