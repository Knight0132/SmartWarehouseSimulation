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

        void Update()
        {
            CheckRobotStatus();
        }

        public async Task InitializeRobotsAsync()
        {
            indoorSpace = await mapLoader.LoadJsonAsync();
            width = mapLoader.width;
            length = mapLoader.length;
            graph = mapLoader.GenerateRouteGraph(indoorSpace); 
            
            int i = 0;
            while (i < numberOfRobots)
            {
                await Task.Delay(10);
                Vector3 initialPosition = new Vector3(Random.Range(0, width), 0, Random.Range(0, length));
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(initialPosition);
                Point point = (Point)cellSpace.Node;
                Vector3 position = new Vector3((float)point.X, 0, (float)point.Y);
                if (cellSpace != null && cellSpace.IsNavigable())
                {
                    GameObject robotObj = Instantiate(robotPrefab, position, Quaternion.identity);
                    RobotController robot = robotObj.GetComponent<RobotController>();
                    if (robot != null)
                    {
                        robot.InitializeRobot(i + 1, indoorSpace, graph);

                        Transform targetIndicatorInstance = Instantiate(targetIndicatorPrefab);
                        robot.mapLoader = this.mapLoader;
                        robot.targetIndicator = targetIndicatorInstance;

                        robots.Add(robot);
                        i++;
                    }
                }
            }
        }

        public List<RobotController> GetAvailableRobots()
        {
            var freeRobots = robots.Where(robot => robot.IsAvailable).ToList();
            Debug.Log($"Found {freeRobots.Count} free robots.");

            foreach (var robot in freeRobots)
            {
                Debug.Log($"Free robot {robot.Id} at position {robot.transform.position}");
            }
            return freeRobots;
        }

        public RobotController GetClosestAvailableRobot(Vector3 position)
        {
            var closestRobot = robots.Where(robot => robot.IsAvailable)
                                     .OrderBy(robot => Vector3.Distance(robot.transform.position, position))
                                     .FirstOrDefault();

            if (closestRobot != null)
            {
                Debug.Log($"Closest available robot is {closestRobot.Id} at position {closestRobot.transform.position}");
            }
            else
            {
                Debug.Log("No available robot found.");
            }
            return closestRobot;
        }

        public void CheckRobotStatus()
        {
            foreach (var robot in robots)
            {
                if (!robot.IsAvailable)
                {
                    Debug.Log($"Robot {robot.Id} is busy with {robot.GetRobotOrdersQueueCount()} orders");
                }
            }
        }
    }
}