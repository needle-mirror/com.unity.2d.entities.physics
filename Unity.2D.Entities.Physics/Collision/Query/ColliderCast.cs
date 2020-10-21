// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Miscellaneous parts of the source code below are an adaption of the Box2D library.

using System;

using Unity.Entities;
using Unity.Mathematics;

namespace Unity.U2D.Entities.Physics
{
    #region Query Input & Output

    // The input to the Collider Cast queries.
    public struct ColliderCastInput
    {
        public IgnoreHit Ignore { get; set; }
        public BlobAssetReference<Collider> Collider;
        public float2x2 Rotation;
        public Ray Ray => m_Ray;
        Ray m_Ray;

        internal QueryContext QueryContext;

        public float2 Start
        {
            get => Ray.Origin;
            set
            {
                var end = m_Ray.Origin + m_Ray.Displacement;
                m_Ray.Origin = value;
                m_Ray.Displacement = end - value;
            }
        }
        public float2 End
        {
            get => m_Ray.Origin + m_Ray.Displacement;
            set => m_Ray.Displacement = value - m_Ray.Origin;
        }

        public static ColliderCastInput Default =>
            new ColliderCastInput
            {
                Rotation = float2x2.identity
            };
    }

    #endregion

    // A hit from a Collider Cast Query.
    public struct ColliderCastHit : IQueryResult
    {
        #region IQueryResult

        public int PhysicsBodyIndex { get; internal set; }
        public ColliderKey ColliderKey { get; internal set; }
        public Entity Entity { get; internal set; }
        public float Fraction { get; internal set; }

        public bool IsValid => Entity != Entity.Null;

        #endregion

        public float2 Position { get; internal set; }
        public float2 SurfaceNormal { get; internal set; }
    }

    // Collider Cast query implementations.
    static class ColliderCastQueries
    {
        static unsafe bool CastCollider(
            Ray ray, ref float2x2 rotation,
            ref DistanceProxy proxySource, ref DistanceProxy proxyTarget,
            out ColliderCastHit hit)
        {
            hit = default;

            var transformSource = new PhysicsTransform
            {
                Translation = ray.Origin,
                Rotation = rotation
            };

            // Check we're not initially overlapped.
            if ((proxySource.VertexCount < 3 || proxyTarget.VertexCount < 3) &&
                OverlapQueries.OverlapConvexConvex(ref transformSource, ref proxySource, ref proxyTarget))
                return false;

            // B = Source
            // A = Target

            var radiusSource = proxySource.ConvexRadius;
            var radiusTarget = proxyTarget.ConvexRadius;
            var totalRadius = radiusSource + radiusTarget;

            var invRotation = math.inverse(rotation);

            var sweepDirection = ray.Displacement;
            var normal = float2.zero;
            var lambda = 0.0f;

            // Initialize the simplex.
            var simplex = new Simplex();
            simplex.Count = 0;
            var vertices = &simplex.Vertex1;

            // Get a support point in the inverse direction.
            var indexTarget = proxyTarget.GetSupport(-sweepDirection);
            var supportTarget = proxyTarget.Vertices[indexTarget];
            var indexSource = proxySource.GetSupport(PhysicsMath.mul(invRotation, sweepDirection));
            var supportSource = PhysicsMath.mul(transformSource, proxySource.Vertices[indexSource]);
            var v = supportTarget - supportSource;

            // Sigma is the target distance between polygons
            var sigma = math.max(PhysicsSettings.Constants.MinimumConvexRadius, totalRadius - PhysicsSettings.Constants.MinimumConvexRadius);
            const float tolerance = PhysicsSettings.Constants.LinearSlop * 0.5f;
    
            var iteration = 0;
            while (
                iteration++ < PhysicsSettings.Constants.MaxGJKInterations &&
                math.abs(math.length(v) - sigma) > tolerance
                )
            {
                if (simplex.Count >= 3)
                    SafetyChecks.ThrowInvalidOperationException("ColliderCast Simplex must have less than 3 vertex.");

		        // Support in direction -supportV (Target - Source)
                indexTarget = proxyTarget.GetSupport(-v);
                supportTarget = proxyTarget.Vertices[indexTarget];
                indexSource = proxySource.GetSupport(PhysicsMath.mul(invRotation, v));
                supportSource = PhysicsMath.mul(transformSource, proxySource.Vertices[indexSource]);
                var p = supportTarget - supportSource;

                v = math.normalizesafe(v);

                // Intersect ray with plane.
                var vp = math.dot(v, p);
                var vr = math.dot(v, sweepDirection);
                if (vp - sigma > lambda * vr)
                {
                    if (vr <= 0.0f)
                        return false;

                    lambda = (vp - sigma) / vr;
                    if (lambda > 1.0f)
                        return false;

                    normal = -v;
                    simplex.Count = 0;
                }

                // Reverse simplex since it works with B - A.
                // Shift by lambda * r because we want the closest point to the current clip point.
                // Note that the support point p is not shifted because we want the plane equation
                // to be formed in un-shifted space.
		        var vertex = vertices + simplex.Count;
		        vertex->IndexA = indexSource;
		        vertex->SupportA = supportSource + lambda * sweepDirection;
		        vertex->IndexB = indexTarget;
		        vertex->SupportB = supportTarget;
		        vertex->W = vertex->SupportB - vertex->SupportA;
		        vertex->A = 1.0f;
		        simplex.Count += 1;

                switch (simplex.Count)
                {
                    case 1:
                        break;

                    case 2:
                        simplex.Solve2();
                        break;

                    case 3:
                        simplex.Solve3();
                        break;

                    default:
                        SafetyChecks.ThrowInvalidOperationException("Simplex has invalid count.");
                        return default;
                }

                // If we have 3 points, then the origin is in the corresponding triangle.
                if (simplex.Count == 3)
                {
                    // Overlap.
                    return false;
                }

                // Get search direction.
                v = simplex.GetClosestPoint();
            }

            // Ensure we don't process an empty simplex.
            if (simplex.Count == 0)
                return false;

            // Prepare result.
            var pointSource = float2.zero;
            var pointTarget = float2.zero;
            simplex.GetWitnessPoints(ref pointSource, ref pointTarget);

            normal = math.normalizesafe(-v);

            hit = new ColliderCastHit
            {
                Position = pointTarget + (normal * radiusTarget),
                SurfaceNormal = normal,
                Fraction = lambda
            };

            return true;
        }

        static bool CastConvex(ref ColliderCastInput input, ref DistanceProxy proxyTarget, out ColliderCastHit hit)
        {
            DistanceProxy proxySource;

            var inputColliderBlob = input.Collider;
            switch (inputColliderBlob.Value.ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                    {
                        ref var convexHull = ref inputColliderBlob.GetColliderRef<ConvexCollider>().m_ConvexHull;
                        proxySource = new DistanceProxy(ref convexHull);
                        break;
                    }

                case ColliderType.Compound:
                default:
                    SafetyChecks.ThrowNotImplementedException();
                    hit = default;
                    return default;
            }

            return CastCollider(input.Ray, ref input.Rotation, ref proxySource, ref proxyTarget, out hit);
        }

        internal static unsafe bool CastCollider<T>(ColliderCastInput input, Collider* collider, ref T collector) where T : struct, ICollector<ColliderCastHit>
        {
            if (!input.Collider.IsCreated ||
                !CollisionFilter.IsCollisionEnabled(input.Collider.Value.Filter, collider->Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            // Clip the ray end-point to the max-fraction.
            input.End = input.Ray.GetLerpPosition(collector.MaxFraction);

            bool hadHit;
            ColliderCastHit hit;
            switch (collider->ColliderType)
            {
                case ColliderType.Box:
                case ColliderType.Polygon:
                case ColliderType.Capsule:
                case ColliderType.Circle:
                    {
                        var convexCollider = (ConvexCollider*)collider;
                        var proxyTarget = new DistanceProxy(ref convexCollider->m_ConvexHull);
                        hadHit = CastConvex(ref input, ref proxyTarget, out hit);
                        break;
                    }

                case ColliderType.Compound:
                    {
                        return CastConvexCompound(input, (PhysicsCompoundCollider*)collider, ref collector);
                    }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            // Ensure we scale the hit fraction back to its original scale.
            hit.Fraction *= collector.MaxFraction;

            if (hadHit && hit.Fraction < collector.MaxFraction)
            {
                hit.PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex;
                hit.ColliderKey = input.QueryContext.ColliderKey;
                hit.Entity = input.QueryContext.Entity;

                hit.Position = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, hit.Position);
                hit.SurfaceNormal = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform.Rotation, hit.SurfaceNormal);

                return collector.AddHit(hit);
            }

            return false;
        }

        #region Convex Compound

        static unsafe bool CastConvexCompound<T>(ColliderCastInput input, PhysicsCompoundCollider* compoundCollider, ref T collector)
            where T : struct, ICollector<ColliderCastHit>
        {
            var leafProcessor = new ConvexCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.ColliderCast(input, ref leafProcessor, ref collector);
        }

        unsafe struct ConvexCompoundLeafProcessor : BoundingVolumeHierarchy.IColliderCastLeafProcessor
        {
            readonly PhysicsCompoundCollider* m_CompoundCollider;

            public ConvexCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool ColliderCastLeaf<T>(ColliderCastInput input, int leafData, ref T collector)
                where T : struct, ICollector<ColliderCastHit>
            {
                ref var child = ref m_CompoundCollider->Children[leafData];

                if (!input.Collider.IsCreated ||
                    !CollisionFilter.IsCollisionEnabled(input.Collider.Value.Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the cast into child space
                var inputLs = input;

                var compoundFromChild = child.CompoundFromChild;
                var childFromCompound = PhysicsMath.inverse(child.CompoundFromChild);

                inputLs.Start = PhysicsMath.mul(childFromCompound, input.Start);
                inputLs.End = PhysicsMath.mul(childFromCompound, input.End);
                inputLs.Rotation = math.mul(childFromCompound.Rotation, input.Rotation);

                inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
                inputLs.QueryContext.LocalToWorldTransform = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, compoundFromChild);

                return child.Collider->CastCollider(inputLs, ref collector);
            }
        }

        #endregion
    }
}