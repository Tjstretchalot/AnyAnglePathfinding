using Microsoft.Xna.Framework;
using SharpMath2;
using System;
using System.Collections.Generic;
using System.Text;

namespace ThetaStarSharp
{
    /// <summary>
    /// Describes a map on which any-angle raytraced pathfinding can occur.
    /// </summary>
    /// <typeparam name="T">The entity which is inside the map</typeparam>
    public interface TSMap<T> where T : TSCollidable
    {
        /// <summary>
        /// Determines if the polygon at the given position is entirely contained within the map.
        /// </summary>
        /// <param name="poly">The polygon</param>
        /// <param name="pos">The position</param>
        /// <returns>True if anything with the bounds poly can be located at pos</returns>
        bool Contains(Polygon2 poly, Vector2 pos);

        /// <summary>
        /// Checks if there are any entities intersecting any of the given polygons which do not have an ID
        /// in excludeIds and do not have any of the exclude flags.
        /// 
        /// Returns true if there are no intersecting entities, returns false if there are intersecting entities
        /// </summary>
        /// <param name="traces">The polygons to check if any entities intersect</param>
        /// <param name="from">The offset for the traces</param>
        /// <param name="excludeIds">Ids of collidables which will be excluded in this check</param>
        /// <param name="excludeFlags">Any collibable which has any of these flags is excluded from this check</param>
        /// <returns>True if there are entities in traces located at from, False otherwise</returns>
        bool Trace(List<Polygon2> traces, Vector2 from, HashSet<int> excludeIds, long excludeFlags);


        /// <summary>
        /// Determines if the given polygon moved from "from" to "to" will intersect anything
        /// </summary>
        /// <param name="poly">the polygon to trace from "from" to "to"</param>
        /// <param name="from">Where the poly wants to start its movement</param>
        /// <param name="to">Where the poly wants to end its movement</param>
        /// <param name="excludeIds">Collidables that poly wont collide with</param>
        /// <param name="excludeFlags">The flags</param>
        /// <returns>True if the movement will not intersect anything, False otherwise</returns>
        bool Trace(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags);

        /// <summary>
        /// Checks what entities if any intersect any of the specified traces 
        /// </summary>
        /// <param name="traces">The set of traces to check</param>
        /// <param name="from">Where the traces start at</param>
        /// <param name="excludeIds">The set of collidable ids which are excluded from this search</param>
        /// <param name="excludeFlags">Collidables with any of these flags are excluded</param>
        /// <returns>A list of all collidables inside any of the specified traces</returns>
        List<T> TraceExhaust(List<Polygon2> traces, Vector2 from, HashSet<int> excludeIds, long excludeFlags);


        /// <summary>
        /// Gets all the collidables that will prevent the specified poly from moving from "from" to "to", except
        /// those with an id in excludeIds or any of the specified exclude flags
        /// </summary>
        /// <param name="poly">The polygon to trace movement of</param>
        /// <param name="from">Where the polygon starts movement</param>
        /// <param name="to">Where the polygon ends movement</param>
        /// <param name="excludeIds">IDs of collidables to exclude from this search</param>
        /// <param name="excludeFlags">The list of flags that any entities who have it are excluded</param>
        /// <returns>A list of collidables that the given polygon will intersect during a move from "from" to "to"</returns>
        List<T> TraceExhaust(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags);
    }
}
