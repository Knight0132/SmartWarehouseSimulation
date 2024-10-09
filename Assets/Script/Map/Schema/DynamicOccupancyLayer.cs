using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace Map
{
    public class DynamicOccupancyLayer
    {
        private bool[,,] timeSpaceMatrix;
        private IndoorSpace indoorSpace;
        private Graph graph;
        private Dictionary<string, Vector3Int> cellSpaceToCoordinate;
        private float startTime;
        private float timeStep;
        private float windowDuration;
        private int width, length, timeSteps;

        public int Width => width;
        public int Length => length;
        public int TimeSteps => timeSteps;

        public DynamicOccupancyLayer(
            IndoorSpace indoorSpace, 
            Graph graph, 
            MapGrid mapGrid,
            Dictionary<string, Vector3Int> cellSpaceCentroids, 
            float initialStartTime, 
            float windowDuration, 
            float timeStep)
        {
            this.cellSpaceToCoordinate = cellSpaceCentroids;
            this.indoorSpace = indoorSpace;
            this.graph = graph;
            this.startTime = initialStartTime;
            this.windowDuration = windowDuration;
            this.timeStep = timeStep;

            width = mapGrid.gridWidth;
            length = mapGrid.gridLength;
            timeSteps = Mathf.CeilToInt(windowDuration / timeStep);

            timeSpaceMatrix = new bool[width, length, timeSteps];
        }

        private int TimeToIndex(float time)
        {
            return Mathf.Clamp(Mathf.FloorToInt((time - startTime) / timeStep), 0, timeSteps - 1);
        }

        public void UpdateTimeForIndex(ConnectionPoint connectionPoint, float newTime)
        {
            SetOccupancy(connectionPoint, newTime, true);
        }

        // Rolling-Time-Window Mechanism
        public void UpdateTime(float newTime)
        {
            float deltaTime = newTime - (startTime + windowDuration);
            if (deltaTime > 0)
            {
                int stepsToShift = Mathf.FloorToInt(deltaTime / timeStep);
                if (stepsToShift > 0)
                {
                    ShiftTimeWindow(stepsToShift);
                    startTime += stepsToShift * timeStep;
                }
            }
        }

        private void ShiftTimeWindow(int steps)
        {
            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < length; z++)
                {
                    for (int t = 0; t < timeSteps - steps; t++)
                    {
                        timeSpaceMatrix[x, z, t] = timeSpaceMatrix[x, z, t + steps];
                    }
                    for (int t = timeSteps - steps; t < timeSteps; t++)
                    {
                        timeSpaceMatrix[x, z, t] = false;
                    }
                }
            }
        }

        public void SetOccupancy(CellSpace cellSpace, float time, bool isOccupied)
        {
            if (!cellSpaceToCoordinate.TryGetValue(cellSpace.Id, out Vector3Int coord))
            {
                Debug.LogWarning($"CellSpace {cellSpace.Id} not found in DynamicOccupancyLayer");
                return;
            }

            int timeIndex = TimeToIndex(time);
            if (timeIndex >= 0 && timeIndex < timeSteps)
            {
                timeSpaceMatrix[coord.x, coord.z, timeIndex] = isOccupied;
            }
        }

        public void SetOccupancy(ConnectionPoint connectionPoint, float time, bool isOccupied)
        {
            int timeIndex = TimeToIndex(time);
            CellSpace targetCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, true);
            if (timeIndex >= 0 && timeIndex < timeSteps)
            {
                if (targetCellSpace != null)
                {
                    SetOccupancy(targetCellSpace, time, isOccupied);
                }
            }
        }

        public bool IsOccupied(CellSpace cellSpace, float time)
        {
            if (!cellSpaceToCoordinate.TryGetValue(cellSpace.Id, out Vector3Int coord))
            {
                Debug.LogWarning($"CellSpace {cellSpace.Id} not found in DynamicOccupancyLayer");
                return false;
            }
            int timeIndex = TimeToIndex(time);
            return (timeIndex >= 0 && timeIndex < timeSteps) && timeSpaceMatrix[coord.x, coord.z, timeIndex];
        }

        public bool IsOccupied(ConnectionPoint connectionPoint, float time)
        {
            int timeIndex = TimeToIndex(time);
            CellSpace sourceCellSpace = indoorSpace.GetCellSpaceFromConnectionPoint(connectionPoint, false);
            return IsOccupied(sourceCellSpace, time) && timeIndex >= 0 && timeIndex < timeSteps;
        }

        public List<Tuple<float, float>> GetOccupiedTimeRanges(string cellSpaceId, float queryStartTime, float queryEndTime)
        {
            if (!cellSpaceToCoordinate.TryGetValue(cellSpaceId, out Vector3Int coord))
            {
                Debug.LogWarning($"CellSpace {cellSpaceId} not found in DynamicOccupancyLayer");
                return new List<Tuple<float, float>>();
            }

            List<Tuple<float, float>> occupiedRanges = new List<Tuple<float, float>>();
            int startIndex = TimeToIndex(queryStartTime);
            int endIndex = TimeToIndex(queryEndTime);
            bool rangeStarted = false;
            float rangeStart = 0;

            for (int t = startIndex; t <= endIndex; t++)
            {
                // record the first index of the range
                if (timeSpaceMatrix[coord.x, coord.z, t] && !rangeStarted)
                {
                    rangeStarted = true;
                    rangeStart = startTime + t * timeStep;
                }
                // record the end index of the range
                else if (!timeSpaceMatrix[coord.x, coord.z, t] && rangeStarted)
                {
                    rangeStarted = false;
                    occupiedRanges.Add(new Tuple<float, float>(rangeStart, startTime + t * timeStep));
                }
            }

            // if the range is still ongoing at the end of the query window
            if (rangeStarted)
            {
                occupiedRanges.Add(new Tuple<float, float>(rangeStart, startTime + endIndex * timeStep));
            }

            return occupiedRanges;
        }

        public void MergeTimeSpaceMatrix(bool[,,] otherMatrix)
        {
            if (otherMatrix.GetLength(0) != width || 
                otherMatrix.GetLength(1) != length || 
                otherMatrix.GetLength(2) != timeSteps)
            {
                throw new ArgumentException("Input matrix dimensions must match the DynamicOccupancyLayer dimensions");
            }

            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < length; z++)
                {
                    for (int t = 0; t < timeSteps; t++)
                    {
                        timeSpaceMatrix[x, z, t] |= otherMatrix[x, z, t];
                    }
                }
            }
        }

        public void ClearOccupancy(string cellSpaceId, float startClearTime, float endClearTime)
        {
            if (!cellSpaceToCoordinate.TryGetValue(cellSpaceId, out Vector3Int coord))
            {
                Debug.LogWarning($"CellSpace {cellSpaceId} not found in DynamicOccupancyLayer");
                return;
            }

            int startIndex = TimeToIndex(startClearTime);
            int endIndex = TimeToIndex(endClearTime);

            for (int t = startIndex; t <= endIndex && t < timeSteps; t++)
            {
                timeSpaceMatrix[coord.x, coord.z, t] = false;
            }
        }

        public void DeleteOldData(float trimBeforeTime)
        {
            int delIndex = TimeToIndex(trimBeforeTime);
            // if it is before the start time of the hole system
            if (delIndex <= 0)
            {
                return;
            }

            // if it is after the current time window, clear the matrix
            if (delIndex >= timeSteps)
            {
                Array.Clear(timeSpaceMatrix, 0, timeSpaceMatrix.Length);
                startTime = trimBeforeTime;
                return;
            }

            // move the data forward to the beginning of the matrix
            for (int x = 0; x < width; x++)
            {
                for (int z = 0; z < length; z++)
                {
                    for (int t = 0; t < timeSteps - delIndex; t++)
                    {
                        timeSpaceMatrix[x, z, t] = timeSpaceMatrix[x, z, t + delIndex];
                    }
                    for (int t = timeSteps - delIndex; t < timeSteps; t++)
                    {
                        timeSpaceMatrix[x, z, t] = false;
                    }
                }
            }

            // update the start time
            startTime += delIndex * timeStep;
        }

        public float GetStartTime()
        {
            return startTime;
        }

        public float GetEndTime()
        {
            return startTime + windowDuration;
        }

        public bool[,,] GetTimeSpaceMatrix()
        {
            return (bool[,,])timeSpaceMatrix.Clone();
        }

        public Dictionary<string, Vector3Int> GetcellSpaceCentroids()
        {
            return new Dictionary<string, Vector3Int>(cellSpaceToCoordinate);
        }

        public float GetWindowDuration()
        {
            return windowDuration;
        }

        public float GetTimeStep()
        {
            return timeStep;
        }
    }
}