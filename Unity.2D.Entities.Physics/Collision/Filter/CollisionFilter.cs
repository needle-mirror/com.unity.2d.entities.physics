using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Unity.U2D.Entities.Physics
{
    // Describes which other objects an object can collide with.
    [DebuggerDisplay("Group: {GroupIndex} BelongsTo: {BelongsTo} CollidesWith: {CollidesWith}")]
    public struct CollisionFilter
    {
        // A bit mask describing which layers this object belongs to.
        public uint BelongsTo;

        // A bit mask describing which layers this object can collide with.
        public uint CollidesWith;

        // An optional override for the bit mask checks.
        // If the value in both objects is equal and positive, the objects always collide.
        // If the value in both objects is equal and negative, the objects never collide.
        public int GroupIndex;

        // Return true if the filter cannot collide with anything,
        // which likely means it was default constructed but not initialized.
        public bool IsEmpty => BelongsTo == 0 || CollidesWith == 0;

        // A collision filter which wants to collide with everything.
        public static readonly CollisionFilter Default = new CollisionFilter
        {
            BelongsTo = 0xffffffff,
            CollidesWith = 0xffffffff,
            GroupIndex = 0
        };

        // A collision filter which never collides with against anything (including Default).
        public static readonly CollisionFilter Zero = new CollisionFilter
        {
            BelongsTo = 0,
            CollidesWith = 0,
            GroupIndex = 0
        };

        // Return true if the given pair of filters want to collide with each other.
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsCollisionEnabled(CollisionFilter filterA, CollisionFilter filterB)
        {
            if (filterA.GroupIndex > 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return true;
            }
            if (filterA.GroupIndex < 0 && filterA.GroupIndex == filterB.GroupIndex)
            {
                return false;
            }
            return
                (filterA.BelongsTo & filterB.CollidesWith) != 0 &&
                (filterB.BelongsTo & filterA.CollidesWith) != 0;
        }

        // Return a union of two filters.
        public static CollisionFilter CreateUnion(CollisionFilter filterA, CollisionFilter filterB)
        {
            return new CollisionFilter
            {
                BelongsTo = filterA.BelongsTo | filterB.BelongsTo,
                CollidesWith = filterA.CollidesWith | filterB.CollidesWith,
                GroupIndex = (filterA.GroupIndex == filterB.GroupIndex) ? filterA.GroupIndex : 0
            };
        }

        // Create a mask given a selection of layers.
        public static uint CreateMask(params int[] collisionLayers)
        {
            uint mask = 0;
            for(var i = 0; i < collisionLayers.Length; ++i)
            {
                var layer = collisionLayers[i];
                if (layer >= 0 && layer <= 32)
                {
                    mask |= (uint)1 << layer;
                    continue;
                }

                SafetyChecks.ThrowArgumentException("Collision mask layers must be in the range 0-31.");
                return default;
            }

            return mask;
        }
    }
}
