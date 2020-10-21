using System;

namespace Unity.U2D.Entities.Physics
{
    class PreserveAttribute : Attribute { }

    [Obsolete("Do not access this type. It is only included to hint AOT compilation", true)]
    static unsafe class AOTHint
    {
        [Preserve]
        static void HintAllImplementations()
        {
            RaycastLeafProcessor_RaycastHitCollectors<Broadphase.BvhLeafProcessor>();
            ColliderCastLeafProcessor_ColliderCasttHitCollectors<Broadphase.BvhLeafProcessor>();
            PointOverlapLeafProcessor_OverlapPointHitCollectors<Broadphase.BvhLeafProcessor>();
            ColliderOverlapLeafProcessor_OverlapColliderHitCollectors<Broadphase.BvhLeafProcessor>();
            ColliderDistanceLeafProcessor_DistanceCollectors<Broadphase.BvhLeafProcessor>();
            PointDistanceLeafProcessor_DistanceCollectors<Broadphase.BvhLeafProcessor>();

            AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<Broadphase.BvhLeafProcessor, Broadphase.PhysicsBodyOverlapsCollector>();
        }

        static void RaycastLeafProcessor_RaycastHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<RaycastHit>();
            p.RayLeaf(default, default, ref all);
            var any = new AnyHitCollector<RaycastHit>();
            p.RayLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<RaycastHit>();
            p.RayLeaf(default, default, ref closest);
        }

        static void ColliderCastLeafProcessor_ColliderCasttHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IColliderCastLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref all);
            var any = new AnyHitCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<ColliderCastHit>();
            p.ColliderCastLeaf(default, default, ref closest);
        }

        static void PointOverlapLeafProcessor_OverlapPointHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IPointOverlapLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<OverlapPointHit>();
            p.PointLeaf(default, default, ref all);
            var any = new AnyHitCollector<OverlapPointHit>();
            p.PointLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<OverlapPointHit>();
            p.PointLeaf(default, default, ref closest);
        }


        static void ColliderOverlapLeafProcessor_OverlapColliderHitCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IColliderOverlapLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<OverlapColliderHit>();
            p.ColliderLeaf(default, default, ref all);
            var any = new AnyHitCollector<OverlapColliderHit>();
            p.ColliderLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<OverlapColliderHit>();
            p.ColliderLeaf(default, default, ref closest);
        }

        static void ColliderDistanceLeafProcessor_DistanceCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IColliderDistanceLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref all);
            var any = new AnyHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref closest);
        }

        static void PointDistanceLeafProcessor_DistanceCollectors<TProcessor>()
            where TProcessor : struct, BoundingVolumeHierarchy.IPointDistanceLeafProcessor
        {
            var p = new TProcessor();
            var all = new AllHitsCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref all);
            var any = new AnyHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref any);
            var closest = new ClosestHitCollector<DistanceHit>();
            p.DistanceLeaf(default, default, ref closest);
        }

        static void AabbOverlapLeafProcessor_BoundingVolumeHierarchy_OverlapQueries_OverlapCollectors<TProcessor,
            TCollector>()
            where TProcessor : struct, BoundingVolumeHierarchy.IAabbOverlapLeafProcessor
            where TCollector : struct, IOverlapCollector
        {
            var collector = new TCollector();
            var p = new TProcessor();
            p.AabbLeaf(default, default, ref collector);
            var bvh = new BoundingVolumeHierarchy();
            bvh.AabbOverlap(default, ref p, ref collector, default);
            OverlapQueries.AabbCollider(default, null, ref collector);
        }
    }
}