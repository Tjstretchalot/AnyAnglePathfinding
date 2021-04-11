using Microsoft.Xna.Framework;
using Priority_Queue;
using SharpMath2;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AnyAnglePathfinding
{
    /// <summary>
    /// The central class for this package. A single pathfinder is used for a single path. Calculates
    /// a path from one location on a map with a greedy any-angle algorithm that looks fairly reasonable
    /// but is not optimal. It tries to go directly to the endpoint, but if it collides on something on that
    /// way it would try to go around it.
    /// </summary>
    /// <typeparam name="T">The entity type</typeparam>
    public class AAPathfinder<T> where T : AACollidable
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
            /// Where we are considering going exactly. This will either
            /// be our start location, or can be thought of as putting one
            /// of our vertexes at the same spot as a vertex on one of the collidables
            /// on the map.
            ///
            /// Note that in theory our "end" location would be another time we
            /// do not do this, but if we find a path to the end location we're
            /// done, so we don't need to really use that Unvisited node for anything
            /// at that point, although you could imagine constructing the Unvisited
            /// node for the end to reverse it.
            ///
            /// That is to say, if Parent is not null, then Location can
            /// be calculated from Collidable, CollidableVertexInd,
            /// and OurVertexInd using the following:
            ///
            /// Location = (
            ///   Collidable.Location +
            ///   Collidable.Bounds.Vertices[CollidableVertexInd] -
            ///   Bounds.Vertices[OurVertexInd]
            /// )
            /// </summary>
            public Vector2 Location;

            /// <summary>
            /// If Parent is not null, this is the collidable which produced
            /// the Location. See Location for more information.
            /// </summary>
            public T Collidable;

            /// <summary>
            /// If Parent is not null, this is the vertex index on the collidable
            /// that produced the location. See Location for more information.
            /// </summary>
            public int CollidableVertexInd;

            /// <summary>
            /// If Parent is not null, this is the vertex index on "our" (the thing
            /// which we are finding a path for) Bounds that produced the Location.
            /// See Location for more information.
            /// </summary>
            public int OurVertexInd;

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

        private AAMap<T> Map;
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
        public AAPathfinder(AAMap<T> map, Polygon2 bounds, Vector2 start, Vector2 end, HashSet<int> excludeIds, long excludeFlags)
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
            HashSet<int> colsIDs = null; // lazily initialized
            int ourNumVerts = Bounds.Vertices.Length;
            for(int colsInd = 0, colsLen = cols.Count; colsInd < colsLen; colsInd++)
            {
                T collidable = cols[colsInd];
                Vector2 cent = collidable.Bounds.Center;
                for(int vertsInd = 0, vertsLen = collidable.Bounds.Vertices.Length; vertsInd < vertsLen; vertsInd++)
                {
                    Vector2 vert = collidable.Bounds.Vertices[vertsInd];
                    int myVertInd = Polygon2.IndexOfFurthestPoint(Bounds.Vertices, cent - vert);

                    List<T> newCollidablesToQueue;
                    if (from.Parent == null || !ReferenceEquals(from.Collidable, collidable))
                    {
                        // This is not a slide operation: we are moving either from a different
                        // collidable onto this one, or we are moving from the start and gluing
                        // onto this collidable. This means we don't have much to worry about
                        // and can just try it.
                        newCollidablesToQueue = ConsiderTarget(from, open, closed, collidable, vertsInd, myVertInd);
                    }
                    else
                    {
                        // We want to slide along the same collidable. Before even trying anything,
                        // this is only likely going to work if the target vertices are adjacent
                        // (e.g., 1 and 2).

                        if (
                            Math.Abs(vertsInd - from.CollidableVertexInd) != 1 && // they aren't normally adjacent
                            (vertsInd != 0 || from.CollidableVertexInd != vertsLen - 1) && // not wrapping this way
                            (vertsInd != vertsLen - 1 || from.CollidableVertexInd != 0) // not wrapping this way
                        )
                        {
                            continue; // no point, this sliding manuever is not necessary and unlikely to work
                        }

                        // Now it's also probably not going to work unless our vertices are the same.
                        // However we can't just eliminate those, because we would actually never
                        // check them otherwise. If they are different we'll need to try the path
                        // from here such that next time the vertices are the same.

                        if (myVertInd != from.OurVertexInd)
                        {
                            // slide such that it's the same vertex on collidable, but change our vertex.
                            // However, we can't _just_ choose to go to our index, we have to make
                            // sure we're not skipping any vertices. In the extreme imagine a circle
                            // approximated by polygons; you can't go directly across the circle here

                            // there is a special case we want to optimize for, most common when
                            // pathfinding like shapes (a common problem). If the two vertices are
                            // already adjacent, and the line between them is parallel to the target
                            // edge, theres no benefit to expanding the point. Not expanding points is,
                            // obviously, a huge speed improvement.

                            // distanceDecreasingVertexInd = amount we need to decrease
                            // from.CollidableVertexInd to get to myVertInd (mod vertsLen).
                            int distanceDecreasingVertexInd, distanceIncreasingVertexInd;
                            if (from.OurVertexInd < myVertInd)
                            {
                                // would need to wrap it around
                                distanceDecreasingVertexInd = from.OurVertexInd + ourNumVerts - myVertInd;
                                distanceIncreasingVertexInd = myVertInd - from.OurVertexInd;
                            } else
                            {
                                distanceDecreasingVertexInd = from.OurVertexInd - myVertInd;
                                distanceIncreasingVertexInd = myVertInd + ourNumVerts - from.OurVertexInd;
                            }

                            int targetOurVertInd;
                            if (distanceIncreasingVertexInd < distanceDecreasingVertexInd)
                            {
                                targetOurVertInd = (from.OurVertexInd + 1) % ourNumVerts;
                            } else
                            {
                                targetOurVertInd = (from.OurVertexInd + ourNumVerts - 1) % ourNumVerts;
                            }

                            // Optimize: Parallel check.
                            Line2 theirLine;
                            if (from.CollidableVertexInd == vertsInd + 1 || vertsInd == vertsLen - 1 && from.CollidableVertexInd == 0)
                            {
                                theirLine = collidable.Bounds.Lines[from.CollidableVertexInd];
                            }
                            else
                            {
                                theirLine = collidable.Bounds.Lines[vertsInd];
                            }

                            bool slidingOurVertexIsUnnecessary = false;
                            if (distanceDecreasingVertexInd == 1)
                            {
                                Line2 ourLine = Bounds.Lines[from.OurVertexInd];

                                if (Line2.Parallel(ourLine, theirLine))
                                {
                                    slidingOurVertexIsUnnecessary = true;
                                }
                            }
                            else if (distanceIncreasingVertexInd == 1)
                            {
                                Line2 ourLine = Bounds.Lines[targetOurVertInd];

                                if (Line2.Parallel(ourLine, theirLine))
                                {
                                    slidingOurVertexIsUnnecessary = true;
                                }
                            }

                            if (slidingOurVertexIsUnnecessary)
                            {
                                // We treat this as if myVertInd == from.OurVertexInd
                                newCollidablesToQueue = ConsiderTarget(
                                    from, open, closed, collidable, vertsInd, myVertInd
                                );
                            }
                            else
                            {
                                newCollidablesToQueue = ConsiderTarget(
                                    from, open, closed, collidable, from.CollidableVertexInd, targetOurVertInd
                                );

                                // also try the other way, but using the "wrong" vertex. The fact we need
                                // to check both makes a lot of since pathfinding checks should be symmetric
                                var tmp = ConsiderTarget(
                                    from, open, closed, collidable, vertsInd, from.OurVertexInd
                                );

                                if (tmp != null)
                                {
                                    if (newCollidablesToQueue == null)
                                    {
                                        newCollidablesToQueue = tmp;
                                    }
                                    else
                                    {
                                        newCollidablesToQueue.AddRange(tmp);
                                    }
                                }
                            }
                        } else
                        {
                            // slide such that it changes the vertex on the collidable, but keeps
                            // ours the same.
                            newCollidablesToQueue = ConsiderTarget(
                                from, open, closed, collidable, vertsInd, myVertInd
                            );
                        }
                    }

                    if (newCollidablesToQueue != null)
                    {
                        if (colsIDs == null)
                        {
                            colsIDs = new HashSet<int>();
                            foreach (var col in cols)
                            {
                                colsIDs.Add(col.ID);
                            }
                        }

                        foreach (var col in newCollidablesToQueue)
                        {
                            if (colsIDs.Add(col.ID))
                            {
                                cols.Add(col);
                                colsLen++;
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// This handles the core step of trying to move from the Unvisited
        /// node to moving one of our vertices (specified by myVertexIndex)
        /// to the given vertex (collidableVertexIndex) on the target collidable.
        ///
        /// There are two relevant cases:
        /// 1. "from" is NOT on this collidable. In that case there are no further
        ///    restrictions.
        /// 2. "from" IS on this collidable. In that case:
        ///   - It should be on an adjacent vertex on the collidable (otherwise
        ///     it is likely to fail, to the point that it's not worth wasting
        ///     CPU cycles on)
        ///   - It should EITHER be the same myVertexIndex OR an adjacent one. If
        ///     it is an adjacent one, it then the line between our two vertexes
        ///     should be parallel to the line between the two vertexes on the
        ///     collidable.
        /// </summary>
        /// <param name="from">The node we are currently expanding.</param>
        /// <param name="open">
        /// The nodes we have scheduled to consider. We will add
        /// up one node (the one specified) to this list, if we determine that
        /// it can be pathed to
        /// </param>
        /// <param name="closed">
        /// The nodes that we have already considered or are confident there are
        /// no better paths to than the ones we have already considered. We might
        /// add the target point to the list if it's ineligible (e.g., not on the map)
        /// or we determine it's not possible to place our polygon there because
        /// it would intersect something when stationary.
        ///
        /// We also add to closed if we find a path to a point. This is the standard
        /// way that points get add to the closed list.
        /// </param>
        /// <param name="collidable">The object which we are trying to go around.</param>
        /// <param name="collidableVertexIndex">
        /// Which vertex we are targeting on the collidable, specified by index in
        /// collidable.Vertices.
        /// </param>
        /// <param name="myVertexIndex">
        /// Which vertex we are moving to match that vertex, specified by index in
        /// Bounds.Vertices
        /// </param>
        /// <returns>
        /// If we determined that this path could have been feasible except there were
        /// some other collidables (not collidable) in the way that we need to move around,
        /// then this is the list of collidables which we should try to go around. Otherwise,
        /// this is null. Never empty.
        /// </returns>
        private List<T> ConsiderTarget(
            Unvisited from, FastPriorityQueue<Unvisited> open, HashSet<Tuple<int, int, int>> closed,
            T collidable, int collidableVertexIndex, int myVertexIndex
        )
        {
            var tup = Tuple.Create(collidable.ID, collidableVertexIndex, myVertexIndex);
            if (closed.Contains(tup))
            {
                // We've either already proved this point is bad, or we've already
                // tried it (or are trying it right now)
                return null;
            }

            Vector2 myVert = Bounds.Vertices[myVertexIndex];
            Vector2 collidableVert = collidable.Bounds.Vertices[collidableVertexIndex];
            Vector2 point = new Vector2(
                collidable.Position.X + collidableVert.X - myVert.X,
                collidable.Position.Y + collidableVert.Y - myVert.Y
            );

            if (!Map.Contains(Bounds, point))
            {
                // This point would put us outside the map
                closed.Add(tup);
                return null;
            }

            List<T> intersectedWhenTryingToGoHere = Map.TraceExhaust(Bounds, from.Location, point, ExcludeIDs, ExcludeFlags);

            if (intersectedWhenTryingToGoHere.Count == 0)
            {
                // We found a path to the point!
                closed.Add(tup);
                var unv = new Unvisited()
                {
                    Parent = from,
                    Location = point,
                    Collidable = collidable,
                    CollidableVertexInd = collidableVertexIndex,
                    OurVertexInd = myVertexIndex,
                    DistanceStartToHere = from.DistanceStartToHere + (point - from.Location).Length(),
                    HeurDistanceHereToDest = (End - point).Length()
                };
                open.Enqueue(unv, unv.CorrectPrio);
                return null;
            }

            // Most likely intersectedWhenTryingToGoHere is a 1-length
            // list of just the target collidable. However, in general, it
            // could include new candidate collidables that we should try
            // and go around. Either case should be handled well by the callee.
            return intersectedWhenTryingToGoHere;
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

        private static List<E> ToList<E>(params E[] v)
        {
            return v.ToList();
        }
    }
}
