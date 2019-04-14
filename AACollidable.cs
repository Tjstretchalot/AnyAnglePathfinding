using Microsoft.Xna.Framework;
using SharpMath2;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AnyAnglePathfinding
{
    /// <summary>
    /// Describes a generic collidable object inside the map. This is not meant to correspond to
    /// the update / rendering logic that you would have, just as the container for collision
    /// information. It's reasonable to suspect that a single entity will have multiple collidables.
    /// </summary>
    public class AACollidable
    {
        /// <summary>
        /// The unique identifier for this collidable
        /// </summary>
        public int ID;

        /// <summary>
        /// A set of flags (up to 64) that this collidable has.
        /// </summary>
        public long Flags;

        /// <summary>
        /// The position of this collidable on the map
        /// </summary>
        public Vector2 Position;

        /// <summary>
        /// The bounds of this collidable
        /// </summary>
        public Polygon2 Bounds;
    }
}
