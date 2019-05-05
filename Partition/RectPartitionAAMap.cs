using Microsoft.Xna.Framework;
using SharpMath2;
using System;
using System.Collections.Generic;

namespace AnyAnglePathfinding.Partition
{

    /// <summary>
    /// An any-angle pathfinding map that automatically partitions the space into rectangles, attempting
    /// to get a target number of entities in each partition.
    /// </summary>
    public class RectPartitionAAMap<T> : AAMap<T> where T : AACollidable
    {
        // \frac{1}{a\cdot\left(z_1-x\right)^2+b\cdot\operatorname{abs}\left(z_1-x\right)+c}
        // a => PUNISH_QUAD, b => PUNISH_LINEAR, c => PUNISH_CONSTANT, x => OFFSET POINT, z_1 => ACTUAL CENTER
        // DERIVATIVE CALCULATOR PASTABLE: 1/(a*(z1-x)^2 + b * abs(z1-x) + c)
        // FIRST DERIVATIVE: -((b*(x-z_1))/abs(x-z_1)-2*a*(z_1-x))/(b*abs(x-z_1)+a*(z_1-x)^2+c)^2
        // The punish function goes from [0, 1] to (0, infty).
        //
        // If p(x) is the punishment for a given point, the line is chosen to minimize p(x).
        //
        // linear term must dominate derivative between points! Changing these arbitrarily can break the approximation

        /// <summary>
        /// The quadratic factor for punishment; punishment changes as a function of 1/(punish_quad*x*x)
        /// </summary>
        public const double PUNISH_QUAD = 16;

        /// <summary>
        /// The linear factor for punishment; punishment changes as a function of (1/(punish_linear*x))
        /// </summary>
        public const double PUNISH_LINEAR = 25;

        /// <summary>
        /// The constant factor for punishment; added to the denominator to avoid infinity. Smaller values correspond with sharper
        /// peaks around entities
        /// </summary>
        public const double PUNISH_CONST = 0.7;

        /// <summary>
        /// Describes the flags / boolean variables that a partition has
        /// </summary>
        [Flags]
        public enum PartitionFlags : int
        {
            /// <summary>
            /// If specified it means the partition is splitting up in the x-axis
            /// space rather than the y-axis space
            /// </summary>
            Horizontal = 1,

            /// <summary>
            /// If specified then the partition has no descendents on the left, meaning that
            /// the left index is pointing to an index in Maps. If not specified, then the left
            /// is pointing to an index in Partitions
            /// </summary>
            LeftLeaf = 1 << 1,

            /// <summary>
            /// If specified then the partition has no descendents on the right, meaning that the
            /// right index is pointing to an index in Maps. If not specified, then the right is
            /// pointing to an index in Partitions.
            /// </summary>
            RightLeaf = 1 << 2,

            /// <summary>
            /// If specified then this is a root partition and cannot be pruned
            /// </summary>
            Root = 1 << 3,

            /// <summary>
            /// If specified then Root must not be specified and means we are the left of Parent
            /// </summary>
            IsLeftOfParent = 1 << 4
        }

        /// <summary>
        /// Describes a a single partition in the map. 
        /// </summary>
        public struct Partition
        {
            /// <summary>
            /// Any flags that describe this partition. Makes it explicit that we will need 4 bytes
            /// to do this even if it were a bool
            /// </summary>
            public PartitionFlags Flags;
            /// <summary>
            /// Where the separating line was placed on the global map. For a horizontal partition this is the
            /// x-value of the separation. For a vertical partition this is the y-value of the separation.
            /// </summary>
            public float Split;
            /// <summary>
            /// Specifies the index to jump to to move left (for horizontal) or up (for vertical). If this
            /// is a leaf node this is an index in Maps, otherwise this is an index in Partitions
            /// </summary>
            public int Left;

            /// <summary>
            /// Specifies the index to jump to to move right (for horizontal) or down (for vertical). If this
            /// is a leaf node this is an index in Maps, otherwise this is an index in Partitions.
            /// </summary>
            public int Right;

            /// <summary>
            /// If not Flags.ROOT, this is the index in Partitions that contains our parent
            /// (ie. we are either their left or right)
            /// </summary>
            public int Parent;
        }

        /// <summary>
        /// The flags that we associate with a given map, to lock in the full 4 bytes of space
        /// </summary>
        [Flags]
        public enum PartitionMapFlags : int
        {
            /// <summary>
            /// If the partition map is on the left
            /// </summary>
            Left = 1
        }

        /// <summary>
        /// The wrapped simple AA map that we use for the double-linked structure
        /// </summary>
        public struct PartitionMap
        {
            /// <summary>
            /// The flags for this map
            /// </summary>
            public PartitionMapFlags Flags;

            /// <summary>
            /// The index in Partitions that this belongs to
            /// </summary>
            public int Partition;

            /// <summary>
            /// The rect that has the size of this map and is 0-offset
            /// </summary>
            public Rect2 Rect;

            /// <summary>
            /// The position of this map
            /// </summary>
            public Vector2 Position;

            /// <summary>
            /// The actual map
            /// </summary>
            public SimpleAAMap<T> Map;
        }

        /// <summary>
        /// The partitions that this map makes. We avoid using a list to make it easier to debug the effect
        /// of resizing on partitioning. This is exposed for ease of debugging
        /// </summary>
        public Partition[] Partitions;

        /// <summary>
        /// The number of partitions that we actually have. Partitions.Count may be larger.
        /// </summary>
        public int NumPartitions;

        /// <summary>
        /// The maps which are referenced from partitions but not reused. Any changes in indexes requires 
        /// updating partitions.
        /// </summary>
        public PartitionMap[] Maps;

        /// <summary>
        /// The number of maps that we actually have. Maps.Count may be larger.
        /// </summary>
        public int NumMaps;

        /// <summary>
        /// The width of the map
        /// </summary>
        public readonly int Width;

        /// <summary>
        /// The height of the map
        /// </summary>
        public readonly int Height;

        /// <summary>
        /// The minimum number of entities we try to place into a partition. Having fewer than this many
        /// entities will not cause the partition to be destroyed, use TriggerDestroyPartitionEntities for
        /// that.
        /// </summary>
        public readonly int MinPartitionEntities;

        /// <summary>
        /// Having this many or more entities in a single partition that will trigger creating a new partition. Should
        /// be at least MinPartitionEntities*2
        /// </summary>
        public readonly int TriggerCreatePartitionEntities;

        /// <summary>
        /// Having this many or fewer entities in a single partition will trigger collapsing the partition. Should be 
        /// no greater than MinPartitionEntities*2 - 1. The bigger the gap between triggering create and triggering
        /// destroy the better the performance.
        /// </summary>
        public readonly int TriggerDestroyPartitionEntities;

        /// <summary>
        /// The maximum number of entities that we will try to place into a single partition. A bigger gap between this and
        /// MinPartitionEntities will improve the likelihood that we find real partitions but worsen the constant time performance.
        /// </summary>
        public readonly int MaxPartitionEntities;

        /// <summary>
        /// This dictionary maps the ids of collidables to the actual collidable.
        /// </summary>
        public Dictionary<int, T> CollidablesLookup;

        /// <summary>
        /// A complete list of all the collidables
        /// </summary>
        public List<T> Collidables;

        /// <summary>
        /// An incrementing counter we use to ensure collidables have unique ids.
        /// </summary>
        private int CollidableCounter;

        /// <summary>
        /// Creates a new automatic rectangle-partitioning map with the given width and height. Entity bounding boxes should be within 
        /// the rectangle (0, 0) and (Width, Height). This is important for FindPartitions
        /// </summary>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="minPartitionEntities">The minimum number of entities we will use when scanning for a new partition line.</param>
        /// <param name="maxPartitionEntities">The maximum number of entities we will use when scanning for a new partition line.</param>
        /// <param name="triggerCreateEntities">The number of entities within a partition that will have us consider making the partition more granular</param>
        /// <param name="triggerDestroyEntities">The number of entities within a partition that will have us consider making the partition less granular</param>
        public RectPartitionAAMap(int width, int height, int minPartitionEntities = 4, int maxPartitionEntities = 20,
                                  int triggerCreateEntities = 15, int triggerDestroyEntities = 4)
        {
            this.Width = width;
            this.Height = height;


            this.MinPartitionEntities = minPartitionEntities;
            this.MaxPartitionEntities = maxPartitionEntities;
            this.TriggerCreatePartitionEntities = triggerCreateEntities;
            this.TriggerDestroyPartitionEntities = triggerDestroyEntities;

            this.Partitions = new Partition[8];
            this.Maps = new PartitionMap[8];

            this.Partitions[0] = new Partition()
            {
                Flags = PartitionFlags.LeftLeaf | PartitionFlags.RightLeaf | PartitionFlags.Root,
                Split = width / 2f,
                Left = 0,
                Right = 1,
                Parent = -1
            };
            this.NumPartitions = 1;

            this.Maps[0] = new PartitionMap()
            {
                Flags = PartitionMapFlags.Left,
                Partition = 0,
                Map = new SimpleAAMap<T>(-1, -1),
                Rect = new Rect2(Vector2.Zero, new Vector2(this.Partitions[0].Split, height)),
                Position = Vector2.Zero
            };

            this.Maps[1] = new PartitionMap()
            {
                Flags = 0,
                Partition = 0,
                Map = new SimpleAAMap<T>(-1, -1),
                Rect = this.Maps[0].Rect,
                Position = new Vector2(this.Partitions[0].Split, 0)
            };
            this.NumMaps = 2;

            this.Collidables = new List<T>();
            this.CollidablesLookup = new Dictionary<int, T>();
        }

        /// <summary>
        /// Finds the map that the given vector belongs to. Chooses the left when ambiguous
        /// </summary>
        /// <param name="pos">The position you are trying to find the partition for</param>
        /// <param name="mapIndex">the index of the map that the point belongs to</param>
        public void FindMap(Vector2 pos, out int mapIndex)
        {
            int partitionIndex = 0;

            while (true)
            {
                if ((this.Partitions[partitionIndex].Flags & PartitionFlags.Horizontal) != 0)
                {
                    // horizontal split
                    if (pos.Y <= this.Partitions[partitionIndex].Split)
                    {
                        // point is on the left side
                        if ((this.Partitions[partitionIndex].Flags & PartitionFlags.LeftLeaf) != 0)
                        {
                            // leaf
                            mapIndex = this.Partitions[partitionIndex].Left;
                            return;
                        }
                        else
                        {
                            // not leaf
                            partitionIndex = this.Partitions[partitionIndex].Left;
                        }
                    }
                    else
                    {
                        // point is on the right side
                        if ((this.Partitions[partitionIndex].Flags & PartitionFlags.RightLeaf) != 0)
                        {
                            // leaf
                            mapIndex = this.Partitions[partitionIndex].Right;
                            return;
                        }
                        else
                        {
                            // not leaf
                            partitionIndex = this.Partitions[partitionIndex].Right;
                        }
                    }
                }
                else
                {
                    // vertical split
                    if (pos.X <= this.Partitions[partitionIndex].Split)
                    {
                        // point is above
                        if ((this.Partitions[partitionIndex].Flags & PartitionFlags.LeftLeaf) != 0)
                        {
                            // leaf
                            mapIndex = this.Partitions[partitionIndex].Left;
                            return;
                        }
                        else
                        {
                            // not leaf
                            partitionIndex = this.Partitions[partitionIndex].Left;
                        }
                    }
                    else
                    {
                        // point is above
                        if ((this.Partitions[partitionIndex].Flags & PartitionFlags.RightLeaf) != 0)
                        {
                            // leaf
                            mapIndex = this.Partitions[partitionIndex].Right;
                            return;
                        }
                        else
                        {
                            // not leaf
                            partitionIndex = this.Partitions[partitionIndex].Right;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Finds all the maps that the given polygon intersects and stores them as the indices in Maps into
        /// maps
        /// </summary>
        /// <param name="poly">The polygon you want to know about</param>
        /// <param name="pos">The position of the polygon</param>
        /// <param name="maps">Where the result is stored</param>
        public void FindMaps(Polygon2 poly, Vector2 pos, List<int> maps)
        {
            // this is optimized for a small number of partitions. Would it be faster to use the tree
            // at the cost of iterative access? likely depends on complexity of polygon. remember we
            // can't as quickly prune as with points so would need a stack
            for (int i = 0, len = this.NumMaps; i < len; i++)
            {
                if (this.Maps[i].Map == null)
                {
                    return;
                }

                if (Shape2.Intersects(poly, this.Maps[i].Rect, pos, this.Maps[i].Position, true))
                {
                    maps.Add(i);

                    if (Rect2.Contains(this.Maps[i].Rect, poly, this.Maps[i].Position, pos, false))
                    {
                        return;
                    }
                }
            }
        }

        /// <summary>
        /// Finds all the maps that intersect any of the given polygons when positioned at pos and stores them as 
        /// indices in Maps into maps. This is equivalent to repeated calls to FindPartitions and removing duplicates
        /// except faster.
        /// </summary>
        /// <param name="traces">the traces you want to know about</param>
        /// <param name="pos">the position of the traces</param>
        /// <param name="maps">where the result is stored</param>
        public void FindMaps(List<Polygon2> traces, Vector2 pos, List<int> maps)
        {
            bool skipContains = false;
            for (int i = 0, len = this.NumMaps; i < len; i++)
            {
                if (this.Maps[i].Map == null)
                {
                    return;
                }

                bool intersect = false;
                for (int j = 0, tlen = traces.Count; j < tlen; j++)
                {
                    if (Shape2.Intersects(traces[j], this.Maps[i].Rect, pos, this.Maps[i].Position, true))
                    {
                        maps.Add(i);
                        intersect = true;
                        break;
                    }
                }

                if (intersect && !skipContains)
                {
                    bool contains = true;
                    for (int j = 0, tlen = traces.Count; j < tlen; j++)
                    {
                        if (!Rect2.Contains(this.Maps[i].Rect, traces[j], this.Maps[i].Position, pos, false))
                        {
                            contains = false;
                            break;
                        }
                    }
                    if (contains)
                    {
                        return;
                    }

                    skipContains = true;
                }
            }
        }

        /// <summary>
        /// Resizes the Partitions array to the specified capacity.
        /// </summary>
        /// <param name="newCapacity">The new capacity. Must be at least NumPartitions</param>
        public void ResizePartitions(int newCapacity)
        {
            if (newCapacity < this.NumPartitions)
            {
                throw new ArgumentException($"cannot resize to {newCapacity} when we have {this.NumPartitions}");
            }

            Partition[] newParts = new Partition[newCapacity];
            for (int i = 0; i < this.NumPartitions; i++)
            {
                newParts[i] = this.Partitions[i];
            }

            this.Partitions = newParts;
        }

        /// <summary>
        /// Reizes the Maps array to the specified capacity
        /// </summary>
        /// <param name="newCapacity">the new capacity</param>
        public void ResizeMaps(int newCapacity)
        {
            if (newCapacity < this.NumMaps)
            {
                throw new ArgumentException($"cannot resize to {newCapacity} when we have {this.NumMaps}");
            }

            PartitionMap[] newMaps = new PartitionMap[newCapacity];
            for (int i = 0; i < this.NumMaps; i++)
            {
                newMaps[i] = this.Maps[i];
            }

            this.Maps = newMaps;
        }

        /// <summary>
        /// Calculates the punishment that is assigned to a line splitting the specified map
        /// </summary>
        /// <param name="points">The points that are pushing the line away</param>
        /// <param name="split">the % of the way through the map that it is split</param>
        public double Punish(double[] points, double split)
        {
            double sum = 0;
            for (int i = 0, len = points.Length; i < len; i++)
            {
                double diff = split - points[i];
                diff = diff < 0 ? -diff : diff;
                sum += 1 / (PUNISH_QUAD * diff * diff + PUNISH_LINEAR * diff + PUNISH_CONST);
            }
            return sum;
        }

        /// <summary>
        /// Calculates the first derivative of the punishment that is assigned to a line splitting the
        /// specified map
        /// </summary>
        /// <param name="points">the points that are pushing the line away</param>
        /// <param name="split">the % of the way through the map that it is split</param>
        public double PunishDeriv(double[] points, double split)
        {
            double sum = 0;
            for (int i = 0, len = points.Length; i < len; i++)
            {
                double numTerm1 = PUNISH_LINEAR * Math.Sign(split - points[i]);
                double numTerm2 = 2 * PUNISH_QUAD * (points[i] - split);
                double num = numTerm1 - numTerm2;

                double sqrtDenom = PUNISH_LINEAR * Math.Abs(split - points[i]) + PUNISH_QUAD * (points[i] - split) * (points[i] - split) + PUNISH_CONST;
                double denom = sqrtDenom * sqrtDenom;
                sum -= num / denom; // note negative
            }
            return sum;
        }

        /// <summary>
        /// Calculates the second derivative of the punishment that is assigned to a line splitting the
        /// specified map
        /// </summary>
        /// <param name="points">the points that are pushing the line away</param>
        /// <param name="split">the % of the way through the map that it is split</param>
        /// <returns>the second derivative</returns>
        public double PunishDeriv2(double[] points, double split)
        {
            double sum = 0;
            const double a = PUNISH_QUAD;
            const double b = PUNISH_LINEAR;
            const double c = PUNISH_CONST;

            double abs(double val) => val < 0 ? -val : val;
            double sqr(double val) => val * val;
            double cub(double val) => val * val * val;

            double x = split;

            for (int i = 0, len = points.Length; i < len; i++)
            {
                double z_1 = points[i];
                sum += (2 * sqr((b * (x - z_1)) / abs(x - z_1) - 2 * a * (z_1 - x))) / cub(b * abs(x - z_1) + a * sqr(z_1 - x) + c) - (2 * a) / sqr(b * abs(x - z_1) + a * sqr(z_1 - x) + c);
            }

            return sum;
        }

        /// <summary>
        /// Fetches the points that will be fed to the punishment function for the given map. The
        /// points come from the centers of the collidables in that map.
        /// </summary>
        /// <param name="map">The map of interest</param>
        /// <param name="horizontal">True for the points setup for a horizontal split, False for the points setup for vertical split</param>
        /// <param name="mult">What you must multiply the offset from the map point to get the scaled point</param>
        /// <param name="add">What you must add to the mult*offset to get to the recentered point</param>
        /// <returns></returns>
        public double[] CalculatePoints(int map, bool horizontal, out double mult, out double add)
        {
            // We want a higher punishment for splitting across the thinner side. To accomplish this,
            // the thinner side is judged by the same criteria as the thicker side with points recentered.

            double width = this.Maps[map].Rect.Width;
            double height = this.Maps[map].Rect.Height;

            if (width > height)
            {
                mult = 1 / width;
                add = horizontal ? (1 - (height / width)) * 0.5 : 0;
            }
            else
            {
                mult = 1 / height;
                add = !horizontal ? (1 - (width / height)) * 0.5 : 0;
            }

            List<T> collidables = this.Maps[map].Map.Collidables;
            double[] pts = new double[collidables.Count];

            double val;
            for (int i = 0, len = pts.Length; i < len; i++)
            {
                if (!horizontal)
                {
                    val = (collidables[i].Position.X + collidables[i].Bounds.Center.X) - this.Maps[map].Position.X;
                }
                else
                {
                    val = (collidables[i].Position.Y + collidables[i].Bounds.Center.Y) - this.Maps[map].Position.Y;
                }

                pts[i] = val * mult + add;
            }

            return pts;
        }

        /// <summary>
        /// Gets the matlab pastable points and selection. Used for debugging
        /// </summary>
        /// <param name="points">the points</param>
        /// <param name="bestLoc">The location we decided to place the separating line</param>
        /// <returns>the corresponding punish equation</returns>
        public string GetPunishPointsPastable(double[] points, double bestLoc)
        {
            string sepd = string.Join(" ", points);
            List<string> lines = new List<string>
            {
                $"PTS = [{sepd}];",
                $"sel = {bestLoc};"
            };
            return string.Join("\r\n", lines);
        }

        /// <summary>
        /// Calculates the best skip point from the given set of possible points, ignoring the specified number of edges
        /// </summary>
        /// <param name="points">the points we can split between</param>
        /// <param name="edgesSkipped">the number of edge points ignored, must be at least 1</param>
        /// <param name="bestLoc">we will store the best split point here</param>
        /// <param name="bestPunish">we will store the punish at the best split point here</param>
        public void SelectSplitPoint(double[] points, int edgesSkipped, out double bestLoc, out double bestPunish)
        {
            bestLoc = 0;
            bestPunish = double.PositiveInfinity;
            for (int i = edgesSkipped, right = points.Length - edgesSkipped; i < right; i++)
            {
                double guessPoint = (points[i - 1] + points[i]) / 2.0;
                double derivAtGuess = PunishDeriv(points, guessPoint);
                double punishAtGuess = Punish(points, guessPoint);

                int newtonIter = 0;
                for (newtonIter = 0; newtonIter < 10; newtonIter++)
                {
                    if (derivAtGuess < -Math2.DEFAULT_EPSILON || derivAtGuess > Math2.DEFAULT_EPSILON)
                    {
                        double secDerivAtGuess = PunishDeriv2(points, guessPoint);
                        if (secDerivAtGuess > -Math2.DEFAULT_EPSILON && secDerivAtGuess < Math2.DEFAULT_EPSILON)
                        {
                            break;
                        }

                        double nextGuessPoint = guessPoint - derivAtGuess / secDerivAtGuess;
                        if (double.IsNaN(nextGuessPoint) || double.IsInfinity(nextGuessPoint) || nextGuessPoint < points[i - 1] || nextGuessPoint > points[i])
                        {
                            break;
                        }
                        guessPoint = nextGuessPoint;
                        punishAtGuess = Punish(points, guessPoint);
                        derivAtGuess = PunishDeriv(points, guessPoint);
                    }
                    else
                    {
                        break;
                    }
                }

                if (punishAtGuess < bestPunish)
                {
                    bestPunish = punishAtGuess;
                    bestLoc = guessPoint;
                }
            }
        }

        /// <summary>
        /// Actually splits the specified map into two maps at the given location
        /// </summary>
        /// <param name="mapInd">The index in Maps for the map to split</param>
        /// <param name="split">The location of the split in absolute coordinates</param>
        /// <param name="horizontal">True if this is a horizontal line split, meaning the split is a y-coordinate. False if a vertical line split, meaning the split is an x-coordinate</param>
        /// <param name="leftWidth">The new width of the left section; exactly the original width when doing a horizontal split</param>
        /// <param name="rightWidth">The new width of the right section; exactly the original width when doing a horizontal split</param>
        /// <param name="leftHeight">The new height of the left section; exactly the original height when doing a vertical split</param>
        /// <param name="rightHeight">The new height of the right section; exactly the original height when doing a vertical split</param>
        /// <param name="rightOffsetX">Where the right is offset horizontally now; exactly the original offset x for a horizontal split and split for a vertical split</param>
        /// <param name="rightOffsetY">Where the right is offset vertically now; exactly the original offset y for a vertical split and split for a horizontal split</param>
        public void DoSplit(int mapInd, float split, bool horizontal, float leftWidth, float rightWidth, float leftHeight, float rightHeight, float rightOffsetX, float rightOffsetY)
        {
            PartitionMap oldMap = this.Maps[mapInd];

            int leftMapInd = mapInd;
            int rightMapInd = this.NumMaps++;
            int partInd = this.NumPartitions++;


            this.Maps[leftMapInd] = new PartitionMap()
            {
                Flags = PartitionMapFlags.Left,
                Partition = partInd,
                Map = new SimpleAAMap<T>(-1, -1),
                Rect = new Rect2(0, 0, leftWidth, leftHeight),
                Position = new Vector2(this.Maps[mapInd].Position.X, this.Maps[mapInd].Position.Y)
            };

            this.Maps[rightMapInd] = new PartitionMap()
            {
                Flags = 0,
                Partition = partInd,
                Map = new SimpleAAMap<T>(-1, -1),
                Rect = new Rect2(0, 0, rightWidth, rightHeight),
                Position = new Vector2(this.Maps[mapInd].Position.X + rightOffsetX, this.Maps[mapInd].Position.Y + rightOffsetY)
            };

            this.Partitions[partInd] = new Partition()
            {
                Flags = PartitionFlags.LeftLeaf
                      | PartitionFlags.RightLeaf
                      | (horizontal ? PartitionFlags.Horizontal : 0)
                      | (((oldMap.Flags & PartitionMapFlags.Left) != 0) ? PartitionFlags.IsLeftOfParent : 0),
                Split = split,
                Left = leftMapInd,
                Right = rightMapInd,
                Parent = oldMap.Partition
            };

            Partition oldPart = this.Partitions[oldMap.Partition];

            PartitionFlags dropFlag = PartitionFlags.LeftLeaf;
            int newLeft = partInd;
            int newRight = oldPart.Right;
            if ((oldMap.Flags & PartitionMapFlags.Left) == 0)
            {
                dropFlag = PartitionFlags.RightLeaf;
                newLeft = oldPart.Left;
                newRight = partInd;
            }

            this.Partitions[oldMap.Partition] = new Partition()
            {
                Flags = (oldPart.Flags & (~dropFlag)),
                Split = oldPart.Split,
                Left = newLeft,
                Right = newRight,
                Parent = oldPart.Parent
            };

            for (int i = 0, len = oldMap.Map.Collidables.Count; i < len; i++)
            {
                T collidable = oldMap.Map.Collidables[i];

                if (Shape2.Intersects(collidable.Bounds, this.Maps[leftMapInd].Rect, collidable.Position, this.Maps[leftMapInd].Position, true))
                {
                    this.Maps[leftMapInd].Map.Collidables.Add(collidable);
                }

                if (Shape2.Intersects(collidable.Bounds, this.Maps[rightMapInd].Rect, collidable.Position, this.Maps[rightMapInd].Position, true))
                {
                    this.Maps[rightMapInd].Map.Collidables.Add(collidable);
                }
            }
        }

        /// <summary>
        /// Performs a vertical split where all entities left of x are separated
        /// from entities right of x within the specified map. Assumes theres enough
        /// space in Partitions and Maps.
        /// </summary>
        /// <param name="mapInd">The map</param>
        /// <param name="x">The x-value to split at</param>
        public void DoVerticalSplit(int mapInd, float x)
        {
            PartitionMap oldMap = this.Maps[mapInd];

            float leftWidth = x - oldMap.Position.X;
            float rightWidth = oldMap.Rect.Width - leftWidth;
            float height = oldMap.Rect.Height;

            DoSplit(mapInd, x, false, leftWidth, rightWidth, height, height, leftWidth, 0);
        }

        /// <summary>
        /// Performs a horizontal split of the map at the given y location
        /// </summary>
        /// <param name="mapInd"></param>
        /// <param name="y"></param>
        public void DoHorizontalSplit(int mapInd, float y)
        {
            PartitionMap oldMap = this.Maps[mapInd];

            float leftHeight = y - oldMap.Position.Y;
            float rightHeight = oldMap.Rect.Height - leftHeight;
            float width = oldMap.Rect.Width;

            DoSplit(mapInd, y, true, width, width, leftHeight, rightHeight, 0, leftHeight);
        }

        /// <summary>
        /// Splits the given map ind into two parts. Must have at least MinPartitionEntries entries
        /// </summary>
        /// <param name="mapInd">the index in maps for the map to split</param>
        public void Split(int mapInd)
        {
            if (this.NumPartitions == this.Partitions.Length)
            {
                ResizePartitions(2 * this.NumPartitions);
            }

            if (this.NumMaps == this.Maps.Length)
            {
                ResizeMaps(2 * this.NumMaps);
            }

            double[] points = CalculatePoints(mapInd, true, out double multH, out double addH);
            Array.Sort(points); // Ascending


            int viableEntries = points.Length - this.MinPartitionEntities * 2; // exclude all points which are more than MinPartitionEntries from the edges
            if (viableEntries > this.MaxPartitionEntities * 2) // exclude all points more than MaxPartitionEntries from the center
            {
                viableEntries = this.MaxPartitionEntities * 2;
            }

            int edgesSkipped = (points.Length - viableEntries) / 2;

            SelectSplitPoint(points, edgesSkipped, out double bestHorizLoc, out double bestHorizPunish);

            points = CalculatePoints(mapInd, false, out double multV, out double addV);
            Array.Sort(points); // Ascending

            SelectSplitPoint(points, edgesSkipped, out double bestVertLoc, out double bestVertPunish);

            if (bestVertPunish < bestHorizPunish)
            {
                double offset = (bestVertLoc - addV) / multV;

                float trueX = (float)(this.Maps[mapInd].Position.X + offset);
                DoVerticalSplit(mapInd, trueX);
            }
            else
            {
                double offset = (bestHorizLoc - addH) / multH;

                float trueY = (float)(this.Maps[mapInd].Position.Y + offset);
                DoHorizontalSplit(mapInd, trueY);
            }
        }

        /// <summary>
        /// Replaces the children on one side of the partition to a leaf node.
        /// </summary>
        /// <param name="partitionInd">the partition which is having one of its sides replaced with a leaf</param>
        /// <param name="left">true to turn the left tree into a left leaf, false to turn right tree into a right leaf</param>
        /// <param name="newX">the x-component that the child will have</param>
        /// <param name="newY">the y-component that the child will have</param>
        /// <param name="newWidth">the width that the new child will have</param>
        /// <param name="newHeight">the height that the new child will have</param>
        /// <param name="newMapInd">Stores the index for the map that the partition is now completely encompassed by</param>
        /// <param name="mapHoleRolling">like partitionHoleRolling but for the maps</param>
        /// <param name="partitionHoleRolling">for a given partition index 0 &lt;= i &lt; NumPartitions, j = i - partitionHoleRolling[i] gives the new partition index after this merge</param>
        private void MergeAllChildren(int partitionInd, bool left, float newX, float newY, float newWidth, float newHeight, out int newMapInd, out int[] mapHoleRolling, out int[] partitionHoleRolling)
        {
            int childPart = left ? this.Partitions[partitionInd].Left : this.Partitions[partitionInd].Right;


            HashSet<T> children = new HashSet<T>();
            void AddMap(int mapInd)
            {
                List<T> collidables = this.Maps[mapInd].Map.Collidables;
                for (int i = 0, len = collidables.Count; i < len; i++)
                {
                    children.Add(collidables[i]);
                }
            }

            List<int> partitionHoles = new List<int>();
            List<int> mapHoles = new List<int>();

            Stack<int> partStack = new Stack<int>();
            partStack.Push(childPart);
            partitionHoles.Add(childPart);

            while (partStack.Count > 0)
            {
                int partInd = partStack.Pop();

                if ((this.Partitions[partInd].Flags & PartitionFlags.LeftLeaf) == 0)
                {
                    partitionHoles.Add(this.Partitions[partInd].Left);
                    partStack.Push(this.Partitions[partInd].Left);
                }
                else
                {
                    mapHoles.Add(this.Partitions[partInd].Left);
                    AddMap(this.Partitions[partInd].Left);
                }

                if ((this.Partitions[partInd].Flags & PartitionFlags.RightLeaf) == 0)
                {
                    partitionHoles.Add(this.Partitions[partInd].Right);
                    partStack.Push(this.Partitions[partInd].Right);
                }
                else
                {
                    mapHoles.Add(this.Partitions[partInd].Right);
                    AddMap(this.Partitions[partInd].Right);
                }
            }

            partitionHoles.Sort();
            int leftShift = 0;
            int holeIndex = 0;
            partitionHoleRolling = new int[this.NumPartitions];
            for (int indInPartitions = partitionHoles[0], len = this.NumPartitions; indInPartitions < len; indInPartitions++)
            {
                if (partitionHoles[holeIndex] == indInPartitions)
                {
                    leftShift++;
                    if (holeIndex < partitionHoles.Count - 1)
                    {
                        holeIndex++;
                    }

                    partitionHoleRolling[indInPartitions] = leftShift;
                    // rolling hole value doesn't matter as theres no references here, but a rolling array is more interpretable
                    continue;
                }

                this.Partitions[indInPartitions - leftShift] = this.Partitions[indInPartitions];
                partitionHoleRolling[indInPartitions] = leftShift;
            }

            this.NumPartitions -= partitionHoles.Count;
            for (int indInPartitions = this.NumPartitions, end = this.NumPartitions + partitionHoles.Count; indInPartitions < end; indInPartitions++)
            {
                this.Partitions[indInPartitions] = default;
            }

            mapHoles.Sort();
            leftShift = 0;
            holeIndex = 0;
            mapHoleRolling = new int[this.NumMaps];
            for (int indInMaps = mapHoles[0], len = this.NumMaps; indInMaps < len; indInMaps++)
            {
                if (mapHoles[holeIndex] == indInMaps)
                {
                    leftShift++;
                    if (holeIndex < mapHoles.Count - 1)
                    {
                        holeIndex++;
                    }

                    mapHoleRolling[indInMaps] = leftShift;
                    // rolling hole value doesn't matter as theres no references here, but a rolling array is more interpretable
                    continue;
                }

                this.Maps[indInMaps - leftShift] = this.Maps[indInMaps];
                mapHoleRolling[indInMaps] = leftShift;
            }

            this.NumMaps -= mapHoles.Count;
            for (int indInMaps = this.NumMaps, end = this.NumMaps + mapHoles.Count; indInMaps < end; indInMaps++)
            {
                this.Maps[indInMaps] = default;
            }

            // updating all references inside partitions
            for (int indInPartitions = 0, len = this.NumPartitions; indInPartitions < len; indInPartitions++)
            {
                Partition part = this.Partitions[indInPartitions];
                if ((part.Flags & PartitionFlags.LeftLeaf) != 0)
                {
                    part.Left -= mapHoleRolling[part.Left];
                }
                else
                {
                    part.Left -= partitionHoleRolling[part.Left];
                }

                if ((part.Flags & PartitionFlags.RightLeaf) != 0)
                {
                    part.Right -= mapHoleRolling[part.Right];
                }
                else
                {
                    part.Right -= partitionHoleRolling[part.Right];
                }

                if ((part.Flags & PartitionFlags.Root) == 0)
                {
                    part.Parent -= partitionHoleRolling[part.Parent];
                }

                this.Partitions[indInPartitions] = part;
            }

            // updating all references inside maps
            for (int indInMaps = 0, len = this.NumMaps; indInMaps < len; indInMaps++)
            {
                PartitionMap map = this.Maps[indInMaps];
                map.Partition -= partitionHoleRolling[map.Partition];
                this.Maps[indInMaps] = map;
            }

            // placing new map
            partitionInd -= partitionHoleRolling[partitionInd];

            newMapInd = this.NumMaps++;
            this.Maps[newMapInd] = new PartitionMap()
            {
                Flags = left ? PartitionMapFlags.Left : 0,
                Partition = partitionInd,
                Map = new SimpleAAMap<T>(-1, -1),
                Rect = new Rect2(0, 0, newWidth, newHeight),
                Position = new Vector2(newX, newY)
            };

            this.Maps[newMapInd].Map.Collidables.AddRange(children);

            if (left)
            {
                Partition part = this.Partitions[partitionInd];
                part.Left = newMapInd;
                part.Flags |= PartitionFlags.LeftLeaf;
                this.Partitions[partitionInd] = part;
            }
            else
            {
                Partition part = this.Partitions[partitionInd];
                part.Right = newMapInd;
                part.Flags |= PartitionFlags.RightLeaf;
                this.Partitions[partitionInd] = part;
            }
        }


        /// <summary>
        /// Considers splitting the specified map into two parts. If it meets the criteria for
        /// splitting than a split will occur, otherwise nothing will happen
        /// </summary>
        /// <param name="mapInd">the index in Maps for the map to consider splitting</param>
        /// <returns>True if we split, False if we did not</returns>
        public bool ConsiderSplit(int mapInd)
        {
            int numEnts = this.Maps[mapInd].Map.Collidables.Count;

            if (numEnts > this.TriggerCreatePartitionEntities)
            {
                Split(mapInd);
                return true;
            }

            return false;
        }

        /// <summary>
        /// Counts the number of entities that are on a particular side of the partition with the given index.
        /// </summary>
        /// <param name="partInd">the index in Partitions</param>
        /// <param name="left">true if you want to count the number of entities on the left, false for the right</param>
        public int CountNumEntities(int partInd, bool left)
        {
            Stack<int> partitions = new Stack<int>();
            if (left)
            {
                if ((this.Partitions[partInd].Flags & PartitionFlags.LeftLeaf) != 0)
                {
                    return this.Maps[this.Partitions[partInd].Left].Map.Collidables.Count;
                }

                partitions.Push(this.Partitions[partInd].Left);
            }
            else
            {
                if ((this.Partitions[partInd].Flags & PartitionFlags.RightLeaf) != 0)
                {
                    return this.Maps[this.Partitions[partInd].Right].Map.Collidables.Count;
                }

                partitions.Push(this.Partitions[partInd].Right);
            }

            int sum = 0;
            while (partitions.Count > 0)
            {
                int part = partitions.Pop();

                if ((this.Partitions[part].Flags & PartitionFlags.LeftLeaf) != 0)
                {
                    sum += this.Maps[this.Partitions[part].Left].Map.Collidables.Count;
                }
                else
                {
                    partitions.Push(this.Partitions[part].Left);
                }

                if ((this.Partitions[part].Flags & PartitionFlags.RightLeaf) != 0)
                {
                    sum += this.Maps[this.Partitions[part].Right].Map.Collidables.Count;
                }
                else
                {
                    partitions.Push(this.Partitions[part].Right);
                }
            }
            return sum;
        }

        /// <summary>
        /// Calculates where on the map the side of the given partition belongs. This does not 
        /// require that we have a leaf node in the specified spot
        /// </summary>
        /// <param name="partInd">The partition</param>
        /// <param name="left">The side of the partition you are interested in</param>
        /// <param name="x">The x-coordinate of the map that the partition describes</param>
        /// <param name="y">The y-coordinate of the map that the partition describes</param>
        /// <param name="width">The width of the map that the partition describes</param>
        /// <param name="height">The height of the map that the partition describes</param>
        public void FindMapLocation(int partInd, bool left, out float x, out float y, out float width, out float height)
        {
            width = this.Width;
            height = this.Height;
            x = 0;
            y = 0;

            List<bool> pathToPartition = new List<bool>() { left };

            int cur = partInd;
            while ((this.Partitions[cur].Flags & PartitionFlags.Root) == 0)
            {
                bool curIsLeft = (this.Partitions[cur].Flags & PartitionFlags.IsLeftOfParent) != 0;
                cur = this.Partitions[cur].Parent;
                pathToPartition.Add(curIsLeft);
            }

            pathToPartition.Reverse();
            for (int i = 0, len = pathToPartition.Count; i < len; i++)
            {
                bool nextLeft = pathToPartition[i];

                if ((this.Partitions[cur].Flags & PartitionFlags.Horizontal) != 0)
                {
                    if (nextLeft)
                    {
                        height = (this.Partitions[cur].Split - y);
                    }
                    else
                    {
                        height = (y + height) - this.Partitions[cur].Split;
                        y = this.Partitions[cur].Split;
                    }
                }
                else
                {
                    if (nextLeft)
                    {
                        width = (this.Partitions[cur].Split - x);
                    }
                    else
                    {
                        width = (x + width) - this.Partitions[cur].Split;
                        x = this.Partitions[cur].Split;
                    }
                }

                cur = nextLeft ? this.Partitions[cur].Left : this.Partitions[cur].Right;
            }
        }

        /// <summary>
        /// Looks through all the partitions effected by a polygon at the given location and considers
        /// pruning them. The Maps and Partitions arrays may change drastically after this operation.
        /// </summary>
        /// <param name="maps">The maps that you are considering pruning</param>
        public void ConsiderPruneMany(IEnumerable<int> maps)
        {
            Stack<Tuple<int, bool, int>> dirty = new Stack<Tuple<int, bool, int>>();
            HashSet<int> alreadyChecked = new HashSet<int>();

            foreach (int mapInd in maps)
            {
                if ((this.Partitions[this.Maps[mapInd].Partition].Flags & PartitionFlags.Root) != 0)
                {
                    continue;
                }

                if (this.Maps[mapInd].Map.Collidables.Count <= this.TriggerDestroyPartitionEntities)
                {
                    int parentPart = this.Partitions[this.Maps[mapInd].Partition].Parent;
                    if (alreadyChecked.Contains(parentPart))
                    {
                        continue;
                    }

                    alreadyChecked.Add(this.Maps[mapInd].Partition);
                    alreadyChecked.Add(parentPart);

                    bool amLeftOfParent = (this.Partitions[this.Maps[mapInd].Partition].Flags & PartitionFlags.IsLeftOfParent) != 0;

                    dirty.Push(Tuple.Create(parentPart, amLeftOfParent, CountNumEntities(parentPart, amLeftOfParent)));
                }
            }

            if (dirty.Count == 0)
            {
                return;
            }

            Queue<Tuple<int, bool>> finished = new Queue<Tuple<int, bool>>();

            while (dirty.Count > 0)
            {
                Tuple<int, bool, int> next = dirty.Pop();
                int partInd = next.Item1;
                bool left = next.Item2;
                int numEnts = next.Item3;

                if ((this.Partitions[partInd].Flags & PartitionFlags.Root) != 0)
                {
                    finished.Enqueue(Tuple.Create(partInd, left));
                    continue;
                }

                int parentInd = this.Partitions[partInd].Parent;
                if (alreadyChecked.Contains(parentInd))
                {
                    continue;
                }

                alreadyChecked.Add(parentInd);

                int oSideEnts = CountNumEntities(parentInd, !left);
                if (oSideEnts + numEnts <= this.TriggerDestroyPartitionEntities)
                {
                    dirty.Push(Tuple.Create(parentInd, (this.Partitions[partInd].Flags & PartitionFlags.IsLeftOfParent) != 0, oSideEnts + numEnts));
                }
                else
                {
                    finished.Enqueue(Tuple.Create(partInd, left));
                }
            }

            List<int> toConsiderSplitting = new List<int>();
            while (finished.Count > 0)
            {
                Tuple<int, bool> next = finished.Dequeue();

                FindMapLocation(next.Item1, next.Item2, out float x, out float y, out float width, out float height);
                MergeAllChildren(next.Item1, next.Item2, x, y, width, height, out int newMapInd, out int[] mapHoleRolling, out int[] partitionHoleRolling);

                for (int i = 0, len = toConsiderSplitting.Count; i < len; i++)
                {
                    toConsiderSplitting[i] -= mapHoleRolling[toConsiderSplitting[i]];
                }
                toConsiderSplitting.Add(newMapInd);

                int num = finished.Count;
                for (int i = 0; i < num; i++)
                {
                    Tuple<int, bool> item = finished.Dequeue();
                    finished.Enqueue(Tuple.Create(item.Item1 - partitionHoleRolling[item.Item1], item.Item2));
                }
            }

            for (int i = 0, len = toConsiderSplitting.Count; i < len; i++)
            {
                ConsiderSplit(toConsiderSplitting[i]);
            }
        }

        /// <summary>
        /// Adds a new collidable to the map. Assigns it a unique id if forceId is false, otherwise
        /// keeps the id the same. Once you use forceId you must always use it
        /// </summary>
        /// <param name="collidable">The collidable to add</param>
        /// <param name="forceId">If set to true then the id is not replaced</param>
        /// <returns>the id of the collidable, collidable.ID, after assigning it one</returns>
        public int Register(T collidable, bool forceId=false)
        {
            if(!forceId)
                collidable.ID = this.CollidableCounter++;

            List<int> maps = new List<int>();
            FindMaps(collidable.Bounds, collidable.Position, maps);


            for (int i = 0, mapLen = maps.Count; i < mapLen; i++)
            {
                this.Maps[maps[i]].Map.Collidables.Add(collidable);
            }

            this.CollidablesLookup[collidable.ID] = collidable;
            this.Collidables.Add(collidable);

            foreach (int mapInd in maps)
            {
                ConsiderSplit(mapInd);
            }

            return collidable.ID;
        }

        /// <summary>
        /// Removes the collidable with the given id from this map
        /// </summary>
        /// <param name="id">The id of the collidable to remove</param>
        public void Unregister(int id)
        {
            T collidable = this.CollidablesLookup[id];

            List<int> maps = new List<int>();
            FindMaps(collidable.Bounds, collidable.Position, maps);

            for (int i = 0, mapLen = maps.Count; i < mapLen; i++)
            {
                this.Maps[i].Map.Collidables.Remove(collidable);
            }

            this.CollidablesLookup.Remove(id);
            this.Collidables.Remove(collidable);

            ConsiderPruneMany(maps);
        }

        /// <summary>
        /// Must be invoked to change the position of a collidable
        /// </summary>
        /// <param name="id">the id of the collidable to move</param>
        /// <param name="position">the new position for the collidable</param>
        public void Move(int id, Vector2 position)
        {
            T collidable = this.CollidablesLookup[id];

            List<int> oldMaps = new List<int>();

            FindMaps(collidable.Bounds, collidable.Position, oldMaps);

            if (oldMaps.Count == 1) // We can shortcut the remaining checks if we know we didn't change maps
            {
                PartitionMap map = this.Maps[oldMaps[0]];
                Rect2 aabb = collidable.Bounds.AABB;

                bool contained = Rect2.Contains(map.Rect, map.Position, position + aabb.Min, false);
                contained = contained && Rect2.Contains(map.Rect, map.Position, position + aabb.Max, false);

                if (contained)
                {
                    collidable.Position = position;
                    return;
                }
            }

            List<int> newMaps = new List<int>();
            FindMaps(collidable.Bounds, position, newMaps);

            HashSet<int> removedMaps = new HashSet<int>();
            removedMaps.UnionWith(oldMaps);
            removedMaps.ExceptWith(newMaps);

            foreach (int mapInd in removedMaps)
            {
                this.Maps[mapInd].Map.Collidables.Remove(collidable);
            }

            HashSet<int> addedMaps = new HashSet<int>();
            addedMaps.UnionWith(newMaps);
            addedMaps.ExceptWith(oldMaps);

            foreach (int mapInd in addedMaps)
            {
                this.Maps[mapInd].Map.Collidables.Add(collidable);
            }

            collidable.Position = position;

            foreach (int mapInd in addedMaps)
            {
                ConsiderSplit(mapInd);
            }

            ConsiderPruneMany(removedMaps);
        }

        /// <summary>
        /// Determines if a polygon located at the given spot would fit in the map
        /// </summary>
        /// <param name="poly">The polygon</param>
        /// <param name="pos">The position of the polygon</param>
        /// <returns>True if the specified polygon fits in the map, False otherwise</returns>
        public bool Contains(Polygon2 poly, Vector2 pos) => pos.X >= 0 && pos.Y >= 0 && pos.X + poly.AABB.Width < this.Width && pos.Y + poly.AABB.Height < this.Height;

        /// <summary>
        /// Finds the id of the first entity which intersects the given position if any entity fits that
        /// criteria, otherwise returns null
        /// </summary>
        /// <param name="pos">The position to find entities which intersect</param>
        /// <returns>The id of the first entity at the given position if there is one, null otherwise</returns>
        public int? GetIntersecting(Vector2 pos)
        {
            FindMap(pos, out int mapIndex);
            return this.Maps[mapIndex].Map.GetIntersecting(pos);
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
            List<int> maps = new List<int>();
            FindMaps(traces, from, maps);

            for (int i = 0, len = maps.Count; i < len; i++)
            {
                if (this.Maps[maps[i]].Map.Trace(traces, from, excludeIds, excludeFlags))
                {
                    return true;
                }
            }

            return false;
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
        public bool Trace(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags) => Trace(Polygon2.CreateRaytraceAbles(poly, to - from), from, excludeIds, excludeFlags);


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
            HashSet<int> myExclude = excludeIds; // initialized if we find anyone to avoid duplicates because a single entity can be in multiple maps

            List<int> maps = new List<int>();
            FindMaps(traces, from, maps);

            List<T> result = new List<T>();
            for (int i = 0, len = maps.Count; i < len; i++)
            {
                List<T> found = this.Maps[maps[i]].Map.TraceExhaust(traces, from, myExclude, excludeFlags);
                if (found.Count > 0)
                {
                    if (ReferenceEquals(myExclude, excludeIds))
                    {
                        myExclude = new HashSet<int>();
                        myExclude.UnionWith(excludeIds);
                    }
                    for (int j = 0, foundLen = found.Count; j < foundLen; j++)
                    {
                        myExclude.Add(found[j].ID);
                    }

                    result.AddRange(found);
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
        public List<T> TraceExhaust(Polygon2 poly, Vector2 from, Vector2 to, HashSet<int> excludeIds, long excludeFlags) => TraceExhaust(Polygon2.CreateRaytraceAbles(poly, to - from), from, excludeIds, excludeFlags);
    }
}
