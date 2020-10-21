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

    /// <summary>
    /// This struct captures the information needed for ray casting.
    /// It is technically not a Ray as it includes a length.
    /// This is to avoid performance issues with infinite length Rays.
    /// </summary>
    public struct Ray
    {
        /// <summary>
        /// The Origin point of the Ray in World Space.
        /// </summary>
        /// <value> Point vector coordinate. </value>
        public float2 Origin;

        /// <summary>
        /// This represents the line from the Ray's Origin to a second point on the Ray. The second point will be the Ray End if nothing is hit.
        /// </summary>
        /// <value> Line vector. </value>
        public float2 Displacement
        {
            get => m_Displacement;
            set
            {
                m_Displacement = value;
                ReciprocalDisplacement = math.select(math.rcp(m_Displacement), math.sqrt(float.MaxValue), m_Displacement == float2.zero);
            }
        }
        float2 m_Displacement;

        /// <summary>
        /// Get a linearly interpolated position.
        /// </summary>
        /// <param name="fraction">The fraction along the ray where 0 = Origin and 1 = Origin+Displacement</param>
        /// <remarks>The fraction can be < 0 or > 1 </remarks>
        /// <returns>The position along the ray.</returns>
        public float2 GetLerpPosition(float fraction) => Origin + (Displacement * fraction);

        // Performance optimization used in the BoundingVolumeHierarchy casting functions
        internal float2 ReciprocalDisplacement { get; private set; }
    }

    /// <summary>
    /// The input to RayCastQueries consists of the Start and End positions of a line segment as well as a CollisionFilter to cull potential hits.
    /// </summary>
    public struct RaycastInput : IQueryInput
    {
        /// <summary>
        /// The Start position of a Ray.
        /// </summary>
        public float2 Start
        {
            get => m_Ray.Origin;
            set
            {
                var end = m_Ray.Origin + m_Ray.Displacement;
                m_Ray.Origin = value;
                m_Ray.Displacement = end - value;
            }
        }
        /// <summary>
        /// The End position of a Ray.
        /// </summary>
        public float2 End
        {
            get => m_Ray.Origin + m_Ray.Displacement;
            set => m_Ray.Displacement = value - m_Ray.Origin;
        }
        /// <summary>
        /// The CollisionFilter is used to determine what objects the Ray is and isn't going to hit.
        /// </summary>
        public CollisionFilter Filter;

        /// <summary>
        /// Optional control of a specific PhysicsBody to ignore.
        /// </summary>
        public IgnoreHit Ignore { get; set; }

        public Ray Ray => m_Ray;
        Ray m_Ray;

        internal QueryContext QueryContext;
        
        public static RaycastInput Default =>
            new RaycastInput
            {
                Filter = CollisionFilter.Default
            };
    }

    // A hit from a ray cast query
    public struct RaycastHit : IQueryResult
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

    #endregion

    // Raycast query implementations
    static class RaycastQueries
    {
        #region Intersection Tests

        // Circle intersection from Box2D.
        static bool RayCircle(
            Ray ray,
            float2 circleCenter, float circleRadius,
            ref float fraction, out float2 normal)
        {
            // Assign an degenerate normal by default so we don't have to
            // when intersection is false.
            normal = float2.zero;

            var localRayOrigin = ray.Origin - circleCenter;
            var b = math.dot(localRayOrigin, localRayOrigin) - (circleRadius * circleRadius);

            // Solve quadratic equation.
            var c = math.dot(localRayOrigin, ray.Displacement);
            var directionDot = math.dot(ray.Displacement, ray.Displacement);
            var sigma = c * c - directionDot * b;

            // Check for negative discriminant and short segment.
            if (sigma < 0.0f || directionDot < float.Epsilon)
            {
                return false;
            }

            // Find the point of intersection of the line with the circle.
            var a = -(c + math.sqrt(sigma));

            // Is the intersection point on the segment?
            if (0.0f <= a && a <= fraction * directionDot)
            {
                a /= directionDot;
                fraction = a;
                normal = math.normalize(localRayOrigin + a * ray.Displacement);
                return true;
            }

            return false;
        }

        // Convex intersection from Box2D.
        static unsafe bool RayConvex(Ray ray, ref ConvexHull hull,
            ref float fraction, out float2 normal)
        {
            // Assign an degenerate normal by default so we don't have to
            // when intersection is false.
            normal = float2.zero;

            var lowerFraction = 0.0f;
            var upperFraction = fraction;

            var index = -1;

            var vertices = hull.Vertices.GetUnsafePtr();
            var normals = hull.Normals.GetUnsafePtr();
            var length = hull.Length;

            for (var i = 0; i < length; ++i)
            {
                var numerator = math.dot(normals[i], vertices[i] - ray.Origin);
                var denominator = math.dot(normals[i], ray.Displacement);

                if (math.abs(denominator) < float.Epsilon)
                {
                    if (numerator < 0f)
                        return false;
                }
                else
                {
                    if (denominator < 0f && numerator < lowerFraction * denominator)
                    {
                        lowerFraction = numerator / denominator;
                        index = i;
                    }
                    else if (denominator > 0.0f && numerator < upperFraction * denominator)
                    {
                        upperFraction = numerator / denominator;
                    }
                }

                if (upperFraction < lowerFraction)
                    return false;
            }

            SafetyChecks.IsTrue(0f <= lowerFraction && lowerFraction <= fraction);

            if (index >= 0)
            {
                fraction = lowerFraction;
                normal = normals[index];
                return true;
            }

            return false;
        }

        // Capsule intersection from Box2D.
        static bool RayCapsule(
            Ray ray,
            float2 vertex0, float2 vertex1, float radius,
            ref float fraction, out float2 normal)
        {
            // Assign an degenerate normal by default so we don't have to
            // when intersection is false.
            normal = float2.zero;

            // No intersection if ray starts inside the capsule.
            if (OverlapQueries.PointCapsule(ray.Origin, vertex0, vertex1, radius))
                return false;

            var rayStart = ray.Origin;
            var rayEnd = rayStart + ray.Displacement;
            var radiusSqr = radius * radius;

            // Figure out signed distance from p1 to infinite capsule line
            var d = vertex1 - vertex0;
            var ld = PhysicsMath.cross(d, vertex0 - rayStart);

            // Only bother if we don't start inside the infinite "cylinder"
            if (!((ld * ld) <= radiusSqr * math.lengthsq(d)))
            {
                var perp = PhysicsMath.cross(d, radius / math.length(d));

                if (ld < 0f)
                {
                    if (QueryUtility.RaycastSegment(ref ray, vertex1 - perp, vertex0 - perp, ref fraction, out normal))
                        return true;
                }
                else
                {
                    if (QueryUtility.RaycastSegment(ref ray, vertex0 + perp, vertex1 + perp, ref fraction, out normal))
                        return true;
                }
            }

            // Check end-caps.
            var rayDirection = rayEnd - rayStart;
            var rayDirectionLengthSqr = math.lengthsq(rayDirection);

            // Ignore small ray fraction.
            if (rayDirectionLengthSqr < float.Epsilon)
                return false;

            // Check the circle caps, starting with closer
            var startVertex = 0;
            if (math.lengthsq(vertex1 - rayStart) < math.lengthsq(vertex0 - rayStart))
                startVertex = 1;

            for (var i = 0; i < 2; ++i)
            {
                var center = startVertex == 0 ? vertex0 : vertex1;

                var s = rayStart - center;
                var b = math.max(0f, math.dot(s, s) - radiusSqr);

                // Solve quadratic equation.
                var c = math.dot(s, rayDirection);
                var sigma = c * c - rayDirectionLengthSqr * b;

                // Check for negative discriminant.
                if (!(sigma < 0f))
                {
                    // Find the point of intersection of the line with the circle.
                    var a = -(c + math.sqrt(sigma));

                    // Is the intersection point on the segment?
                    if (0f <= a && a <= fraction * rayDirectionLengthSqr)
                    {
                        a /= rayDirectionLengthSqr;
                        fraction = a;
                        normal = math.normalize(s + a * rayDirection);
                        return true;
                    }
                }

                startVertex = 1 - startVertex;
            }

            return false;
        }

        #endregion

        #region Ray vs Convex

        internal static unsafe bool RayCollider<T>(RaycastInput input, Collider* collider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, collider->Filter))
            {
                return false;
            }

            // Ensure the query context is initialized.
            input.QueryContext.EnsureIsInitialized();

            var fraction = collector.MaxFraction;
            float2 normal;
            bool hadHit;
            switch (collider->ColliderType)
            {
                case ColliderType.Box:
                    {
                        var box = (PhysicsBoxCollider*)collider;
                        hadHit = RayConvex(input.Ray, ref box->m_ConvexHull, ref fraction, out normal);
                        break;
                    }

                case ColliderType.Polygon:
                    {
                        var box = (PhysicsPolygonCollider*)collider;
                        hadHit = RayConvex(input.Ray, ref box->m_ConvexHull, ref fraction, out normal);
                        break;
                    }

                case ColliderType.Capsule:
                    {
                        var capsule = (PhysicsCapsuleCollider*)collider;
                        hadHit = RayCapsule(input.Ray, capsule->Vertex0, capsule->Vertex1, capsule->Radius, ref fraction, out normal);
                        break;
                    }

                case ColliderType.Circle:
                    {
                        var circle = (PhysicsCircleCollider*)collider;
                        hadHit = RayCircle(input.Ray, circle->Center, circle->Radius, ref fraction, out normal);
                        break;
                    }

                case ColliderType.Compound:
                    {
                        return RayCompound(input, (PhysicsCompoundCollider*)collider, ref collector);
                    }

                default:
                    SafetyChecks.ThrowNotImplementedException();
                    return default;
            }

            if (hadHit && fraction < collector.MaxFraction)
            {
                var hit = new RaycastHit
                {
                    Fraction = fraction,
                    Position = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, input.Ray.GetLerpPosition(fraction)),
                    SurfaceNormal = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform.Rotation, normal),

                    PhysicsBodyIndex = input.QueryContext.PhysicsBodyIndex,
                    ColliderKey = input.QueryContext.ColliderKey,
                    Entity = input.QueryContext.Entity
                };

                return collector.AddHit(hit);
            }
            return false;
        }

        #region Ray Compound

        unsafe struct RayCompoundLeafProcessor : BoundingVolumeHierarchy.IRaycastLeafProcessor
        {
            readonly PhysicsCompoundCollider* m_CompoundCollider;

            public RayCompoundLeafProcessor(PhysicsCompoundCollider* compoundCollider)
            {
                m_CompoundCollider = compoundCollider;
            }

            public bool RayLeaf<T>(RaycastInput input, int leafData, ref T collector) where T : struct, ICollector<RaycastHit>
            {
                ref var child = ref m_CompoundCollider->Children[leafData];

                if (!CollisionFilter.IsCollisionEnabled(input.Filter, child.Collider->Filter))
                {
                    return false;
                }

                // Transform the ray into child space.
                var inputLs = input;
                {
                    var compoundFromChild = child.CompoundFromChild;
                    var childFromCompound = PhysicsMath.inverse(compoundFromChild);

                    inputLs.Start = PhysicsMath.mul(childFromCompound, input.Start);
                    inputLs.End = PhysicsMath.mul(childFromCompound, input.End);

                    inputLs.QueryContext.ColliderKey = input.QueryContext.PushSubKey(m_CompoundCollider->NumColliderKeyBits, (uint)leafData);
                    inputLs.QueryContext.NumColliderKeyBits = input.QueryContext.NumColliderKeyBits;
                    inputLs.QueryContext.LocalToWorldTransform = PhysicsMath.mul(input.QueryContext.LocalToWorldTransform, compoundFromChild);
                }

                return child.Collider->CastRay(inputLs, ref collector);
            }
        }

        static unsafe bool RayCompound<T>(RaycastInput input, PhysicsCompoundCollider* compoundCollider, ref T collector) where T : struct, ICollector<RaycastHit>
        {
            if (!CollisionFilter.IsCollisionEnabled(input.Filter, compoundCollider->Filter))
            {
                return false;
            }

            var leafProcessor = new RayCompoundLeafProcessor(compoundCollider);
            return compoundCollider->BoundingVolumeHierarchy.Raycast(input, ref leafProcessor, ref collector);
        }

        #endregion

        #endregion
    }
}
