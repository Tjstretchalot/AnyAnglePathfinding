using Microsoft.Xna.Framework;
using Priority_Queue;
using SharpMath2;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ThetaStarSharpExample;

namespace ThetaStarSharp
{
    /// <summary>
    /// The central class for this package. A single pathfinder is used for a single path. Calculates
    /// a path from one location on a map with a greedy any-angle algorithm that looks fairly reasonable
    /// but is not optimal. It tries to go directly to the endpoint, but if it collides on something on that 
    /// way it would try to go around it.
    /// </summary>
    /// <typeparam name="T">The entity type</typeparam>
    public class TSPathfinder<T> where T : TSCollidable
    {
        /// <summary>
        /// These go onto a priority queue for nodes we decide to expand.
        /// </summary>
        private sealed class Unvisited : FastPriorityQueueNode
        {
            /// <summary>
            /// The node that you must go through to get here
            /// </summary>
            public Unvisited Parent;
            /// <summary>
            /// Where we are considering going
            /// </summary>
            public Vector2 Location;
            /// <summary>
            /// How far it requires to get from the start to here
            /// </summary>
            public float DistanceStartToHere;
            /// <summary>
            /// How much time it would take to get to the end from Location if 
            /// there was nothing in the way
            /// </summary>
            public float HeurDistanceHereToDest;
            
            /// <summary>
            /// Higher numbers are less important; a priority of 0 is most important. We prefer
            /// things that have already gone further to ones that are close to us
            /// </summary>
            public float CorrectPrio => DistanceStartToHere + HeurDistanceHereToDest * 1.5f;
        }

        private TSMap<T> Map;
        private Polygon2 Bounds;
        private Vector2 Start;
        private Vector2 End;
        private HashSet<int> ExcludeIDs;
        private long ExcludeFlags;

        /// <summary>
        /// Initializes a pathfinder that is attached to the given map that will move a given polygon from start to end, going through
        /// any collidables which have an id in excludeIds or have any of the exclude flags
        /// </summary>
        /// <param name="map">the map to move the polygon within</param>
        /// <param name="bounds">the bounds of the thing to move</param>
        /// <param name="start">where the path should start</param>
        /// <param name="end">where the path should end</param>
        /// <param name="excludeIds">the ids of collidables to ignore in the path calculation</param>
        /// <param name="excludeFlags">collidables with flags to exclude</param>
        public TSPathfinder(TSMap<T> map, Polygon2 bounds, Vector2 start, Vector2 end, HashSet<int> excludeIds, long excludeFlags)
        {
            Map = map;
            Bounds = bounds;
            Start = start;
            End = end;
            ExcludeIDs = excludeIds;
            ExcludeFlags = excludeFlags;
        }

        /// <summary>
        /// Calculates the path that the unit should follow to get to end from start. If no path was found, then
        /// this returns none
        /// </summary>
        /// <returns>the list of points to go to null if no path found</returns>
        public List<Vector2> CalculatePath()
        {
            List<T> collidables = Map.TraceExhaust(Bounds, Start, End, ExcludeIDs, ExcludeFlags);
            if (collidables.Count == 0)
                return ToList(End);

            if (!Map.Trace(ToList(Bounds), End, ExcludeIDs, ExcludeFlags))
                return null;

            HashSet<Tuple<int, int, int>> closed = new HashSet<Tuple<int, int, int>>();
            FastPriorityQueue<Unvisited> open = new FastPriorityQueue<Unvisited>(256);

            var uvStart = new Unvisited()
            {
                Parent = null,
                Location = Start,
                DistanceStartToHere = 0,
                HeurDistanceHereToDest = (End - Start).Length()
            };
            
            QueueCollidables(uvStart, collidables, open, closed);
            
            while (open.Count > 0)
            {
                Unvisited next = open.Dequeue();
                collidables = Map.TraceExhaust(Bounds, next.Location, End, ExcludeIDs, ExcludeFlags);
                if (collidables.Count == 0)
                    return Reverse(next);
                QueueCollidables(next, collidables, open, closed);
            }

            return null;
        }
        
        private void QueueCollidables(Unvisited from, List<T> cols, FastPriorityQueue<Unvisited> open, HashSet<Tuple<int, int, int>> closed)
        {
            float aabbW = Bounds.AABB.Width;
            float aabbH = Bounds.AABB.Height;

            HashSet<Tuple<int, int, int>> myClosed = null;
            for(int colsInd = 0, colsLen = cols.Count; colsInd < colsLen; colsInd++)
            {
                TSCollidable collidable = cols[colsInd];
                Vector2 cent = collidable.Bounds.Center;
                for(int vertsInd = 0, vertsLen = collidable.Bounds.Vertices.Length; vertsInd < vertsLen; vertsInd++)
                {
                    Vector2 vert = collidable.Bounds.Vertices[vertsInd];
                    for (int myVertInd = 0, myVertsLen = Bounds.Vertices.Length; myVertInd < myVertsLen; myVertInd++)
                    {
                        var tup = Tuple.Create(collidable.ID, vertsInd, myVertInd);
                        if (closed.Contains(tup))
                            continue;
                        if (myClosed != null && myClosed.Contains(tup))
                            continue;
                        Vector2 myVert = Bounds.Vertices[myVertInd];

                        Vector2 point = new Vector2(vert.X - myVert.X, vert.Y - myVert.Y);
                        point += collidable.Position;
                        
                        if (Math2.Approximately(point, from.Location))
                        {
                            closed.Add(tup);
                            continue;
                        }

                        if (!Map.Contains(Bounds, point))
                        {
                            closed.Add(tup);
                            continue;
                        }

                        var inters = Map.TraceExhaust(ToList(Bounds), point, ExcludeIDs, ExcludeFlags);
                        if (inters.Count != 0)
                        {
                            if (myClosed == null)
                                myClosed = new HashSet<Tuple<int, int, int>>();
                            myClosed.Add(tup);

                            for (int i = 0; i < inters.Count; i++)
                            {
                                if (inters[i] != collidable)
                                {
                                    cols.Add(inters[i]);
                                    colsLen++;
                                }
                            }
                            continue;
                        }

                        List<T> tmp = Map.TraceExhaust(Bounds, from.Location, point, ExcludeIDs, ExcludeFlags);
                        if (tmp.Count == 0)
                        {
                            closed.Add(tup);
                            var unv = new Unvisited()
                            {
                                Parent = from,
                                Location = point,
                                DistanceStartToHere = from.DistanceStartToHere + (point - from.Location).Length(),
                                HeurDistanceHereToDest = (End - point).Length()
                            };
                            open.Enqueue(unv, unv.CorrectPrio);
                        }
                        else
                        {
                            if (myClosed == null)
                                myClosed = new HashSet<Tuple<int, int, int>>();
                            myClosed.Add(tup);

                            for (int i = 0; i < tmp.Count; i++)
                            {
                                if (tmp[i] != collidable)
                                {
                                    cols.Add(tmp[i]);
                                    colsLen++;
                                }
                            }
                        }
                    }
                }
            }
        }

        private List<Vector2> Reverse(Unvisited last)
        {
            List<Vector2> inReverseOrder = new List<Vector2>();
            inReverseOrder.Add(End);
            while(last != null)
            {
                inReverseOrder.Add(last.Location);
                last = last.Parent;
            }

            inReverseOrder.Reverse();
            return inReverseOrder;
        }

        private static List<T> ToList<T>(params T[] v)
        {
            return v.ToList();
        }
    }
}
