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
    /// Describes a rectangular map on which pathfinding can occur. The map contains the collidables
    /// that are placed on the map. It performs "traces" of these collidables across the map and can
    /// determine who, if anyone, a polygon will intersect along a particular path.
    /// 
    /// This is intended for use as a reference and testing implementation and is overly simplistic 
    /// and not conducive to 
    /// </summary>
    /// <typeparam name="T">The type of collidables that are on this map</typeparam>
    public class SimpleAAMap<T> : AAMap<T> where T : AACollidable
    {
        /// <summary>
        /// The width of the map
        /// </summary>
        public readonly int Width;

        /// <summary>
        /// The height of the map
        /// </summary>
        public readonly int Height;

        /// <summary>
        /// Which units are inside the map
        /// </summary>
        public List<T> Collidables;

        /// <summary>
        /// Incremented whenever a collidable is registered. Used to give the collidables
        /// a unique identifier.
        /// </summary>
        private int CollidableCounter;

        /// <summary>
        /// Initializes a new map with the given width and height and no collidables.
        /// </summary>
        /// <param name="width">The width of the map, used for contains</param>
        /// <param name="height">The height of the map, used for contains</param>
        public SimpleAAMap(int width, int height)
        {
            Collidables = new List<T>();
            CollidableCounter = 0;

            Width = width;
            Height = height;
        }

        /// <summary>
        /// Determines if a polygon located at the given spot would fit in the map
        /// </summary>
        /// <param name="poly">The polygon</param>
        /// <param name="pos">The position of the polygon</param>
        /// <returns>True if the specified polygon fits in the map, False otherwise</returns>
        public bool Contains(Polygon2 poly, Vector2 pos)
        {
            return pos.X >= 0 && pos.Y >= 0 && pos.X + poly.AABB.Width < Width && pos.Y + poly.AABB.Height < Height;
        }

        /// <summary>
        /// Finds the id of the first entity which intersects the given position if any entity fits that
        /// criteria, otherwise returns null
        /// </summary>
        /// <param name="pos">The position to find entities which intersect</param>
        /// <returns>The id of the first entity at the given position if there is one, null otherwise</returns>
        public int? GetIntersecting(Vector2 pos)
        {
            for(int i = 0, len = Collidables.Count; i < len; i++)
            {
                T collidable = Collidables[i];
                if (Polygon2.Contains(collidable.Bounds, collidable.Position, Rotation2.Zero, pos, false))
                    return collidable.ID;
            }
            return null;
        }

        /// <summary>
        /// Adds a new collidable to the map. Assigns it a unique id.
        /// </summary>
        /// <param name="collidable">The collidable to add</param>
        /// <returns>the id of the collidable, collidable.ID, after assigning it one</returns>
        public int Register(T collidable)
        {
            collidable.ID = CollidableCounter++;
            Collidables.Add(collidable);
            return collidable.ID;
        }

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
        public bool Trace(List<Polygon2> traces, Vector2 from, HashSet<int> excludeIds, long excludeFlags)
        {
            int tracesLen = traces.Count;

            for(int i = 0, len = Collidables.Count; i < len; i++)
            {
                T collidable = Collidables[i];
                if (excludeIds.Contains(collidable.ID))
                    continue;
                if ((collidable.Flags & excludeFlags) != 0)
                    continue;

                for(int j = 0; j < tracesLen; j++)
                {
                    if (Polygon2.Intersects(collidable.Bounds, traces[j], collidable.Position, from, true))
                        return false;
                }
            }
            return true;
        }
        
        /// <summary>
        /// Determines if the given polygon moved from "from" to "to" will intersect anything
        /// </summary>
        /// <param name="poly">the polygon to trace from "from" to "to"</param>
        /// <param name="from">Where the poly wants to start its movement</param>
        /// <param name="to">Where the poly wants to end its movement</param>
        /// <param name="excludeIds">Collidables that poly wont collide with</param>
        /// <param name="excludeFlags">The flags</param>
        /// <returns>True if the movement will not intersect anything, False otherwise</returns>
        public bool Trace(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags)
        {
            return Trace(Polygon2.CreateRaytraceAbles(poly, to - from), from, excludeIds, excludeFlags);
        }

        /// <summary>
        /// Checks what entities if any intersect any of the specified traces 
        /// </summary>
        /// <param name="traces">The set of traces to check</param>
        /// <param name="from">Where the traces start at</param>
        /// <param name="excludeIds">The set of collidable ids which are excluded from this search</param>
        /// <param name="excludeFlags">Collidables with any of these flags are excluded</param>
        /// <returns>A list of all collidables inside any of the specified traces</returns>
        public List<T> TraceExhaust(List<Polygon2> traces, Vector2 from, HashSet<int> excludeIds, long excludeFlags)
        {
            List<T> result = new List<T>();
            int tracesLen = traces.Count;

            for (int i = 0, len = Collidables.Count; i < len; i++)
            {
                T collidable = Collidables[i];
                if (excludeIds.Contains(collidable.ID))
                    continue;
                if ((collidable.Flags & excludeFlags) != 0)
                    continue;

                for (int j = 0; j < tracesLen; j++)
                {
                    if (Polygon2.Intersects(collidable.Bounds, traces[j], collidable.Position, from, true))
                    {
                        result.Add(collidable);
                        break;
                    }
                }
            }

            return result;
        }

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
        public List<T> TraceExhaust(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags)
        {
            return TraceExhaust(Polygon2.CreateRaytraceAbles(poly, to - from), from, excludeIds, excludeFlags);
        }
    }
}
