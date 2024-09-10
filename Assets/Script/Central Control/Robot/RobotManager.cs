using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using NetTopologySuite.Geometries;
using Map;

namespace CentralControl
{
    public class RobotManager : MonoBehaviour
    {
        public GameObject robotPrefab;
        public int numberOfRobots = 10;
        public MapLoader mapLoader;
        public Transform targetIndicatorPrefab;
        public IndoorSpace indoorSpace;
        public float width, length;
        public Graph graph;
        private List<RobotController> robots = new List<RobotController>();

        public int TotalRobots => robots.Count;
        public int BusyRobots => robots.Count(r => !r.IsFree);

        void Update()
        {
            CheckRobotStatus();
        }

        public async Task InitializeRobotsAsync()
        {
            indoorSpace = await mapLoader.LoadJsonAsync();
            if (indoorSpace == null)
            {
                Debug.LogError("Failed to load indoor space data.");
                return;
            }

            width = mapLoader.width;
            length = mapLoader.length;
            graph = mapLoader.GenerateRouteGraph(indoorSpace);
            if (graph == null)
            {
                Debug.LogError("Failed to generate route graph.");
                return;
            }
            
            for (int i = 0; i < numberOfRobots; i++)
            {
                await Task.Delay(1);
                Vector3 initialPosition = GetValidInitialPosition();
                if (initialPosition != Vector3.zero)
                {
                    CreateRobot(i + 1, initialPosition);
                }
                else
                {
                    Debug.LogWarning($"Failed to find valid position for robot {i + 1}");
                }
            }
        }

        private Vector3 GetValidInitialPosition()
        {
            for (int attempts = 0; attempts < 10; attempts++)
            {
                Vector3 initialPosition = new Vector3(Random.Range(0, width), mapLoader.height, Random.Range(0, length));
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(initialPosition);
                if (cellSpace != null && cellSpace.IsNavigable())
                {
                    Point point = (Point)cellSpace.Node;
                    return new Vector3((float)point.X, mapLoader.height, (float)point.Y);
                }
            }
            return Vector3.zero;
        }

        private void CreateRobot(int id, Vector3 position)
        {
            GameObject robotObj = Instantiate(robotPrefab, position, Quaternion.identity);
            RobotController robot = robotObj.GetComponent<RobotController>();
            if (robot != null)
            {
                robot.InitializeRobot(id, indoorSpace, graph);

                Transform targetIndicatorInstance = Instantiate(targetIndicatorPrefab);
                robot.mapLoader = this.mapLoader;
                robot.targetIndicator = targetIndicatorInstance;

                robots.Add(robot);
                Debug.Log($"Robot {id} created at position {position}");
            }
            else
            {
                Debug.LogError($"Failed to get RobotController component for robot {id}");
            }
        }

        public RobotController GetClosestRobot(Vector3 position)
        {
            var closestAvailableRobot = GetClosestRobotByCondition(position, robot => robot.IsAvailable);
            if (closestAvailableRobot != null)
            {
                return closestAvailableRobot;
            }

            var closestFreeRobot = GetClosestRobotByCondition(position, robot => robot.IsFree);
            if (closestFreeRobot != null)
            {
                return closestFreeRobot;
            }

            Debug.Log("No suitable robot found.");
            return null;
        }

        private RobotController GetClosestRobotByCondition(Vector3 position, System.Func<RobotController, bool> condition)
        {
            var closestRobot = robots.Where(condition)
                                     .OrderBy(robot => Vector3.Distance(robot.transform.position, position))
                                     .FirstOrDefault();

            if (closestRobot != null)
            {
                Debug.Log($"Closest robot meeting condition is {closestRobot.Id} at position {closestRobot.transform.position}");
            }
            else
            {
                Debug.Log("No robot meeting condition found.");
            }
            return closestRobot;
        }

        public List<RobotController> GetAllRobots()
        {
            return new List<RobotController>(robots);
        }

        public void CheckRobotStatus()
        {
            foreach (var robot in robots)
            {
                if (!robot.IsAvailable)
                {
                    Debug.Log($"Robot {robot.Id} is busy with {robot.GetRobotOrdersQueueCount} orders: {string.Join(", ", robot.listOrdersQueue)}");
                }
                if (robot.IsFree)
                {
                    Debug.Log($"Robot {robot.Id} is free at position {robot.transform.position}");
                }

                if (robot.IsFree && !robot.IsProcessingOrders)
                {
                    Debug.LogWarning($"Robot {robot.Id} is free but not processing orders. This might indicate an issue.");
                }
            }
        }

        public void ResetStuckRobots()
        {
            foreach (var robot in robots)
            {
                if (robot.IsFree && !robot.IsProcessingOrders && robot.GetRobotOrdersQueueCount > 0)
                {
                    Debug.LogWarning($"Resetting stuck robot {robot.Id}");
                    robot.ResetRobot();
                }
            }
        }

        public void DiagnoseAllRobots()
        {
            Debug.Log($"Diagnosing all robots. Total: {TotalRobots}, Busy: {BusyRobots}");
            foreach (var robot in robots)
            {
                Debug.Log($"Robot {robot.Id}: Free: {robot.IsFree}, Available: {robot.IsAvailable}, " +
                          $"Processing Orders: {robot.IsProcessingOrders}, Queue Count: {robot.GetRobotOrdersQueueCount}, " +
                          $"Position: {robot.transform.position}");
            }
        }

        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(r => r.IsFree).ToList();
        }
    }
}