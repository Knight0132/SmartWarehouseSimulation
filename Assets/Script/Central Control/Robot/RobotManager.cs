using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class RobotManager : MonoBehaviour
    {
        public GameObject robotPrefab;
        public int numberOfRobots = 10;
        public MapLoader mapLoader;
        public IndoorSpace indoorSpace;
        public float width, length;
        public Graph graph;
        private List<RobotController> robots = new List<RobotController>();

        async void Start()
        {
            indoorSpace = await mapLoader.LoadJsonAsync();
            width = mapLoader.width;
            length = mapLoader.length;
            graph = mapLoader.GenerateRouteGraph(indoorSpace); 
            await InitializeRobotsAsync();
        } 

        void Update()
        {
            CheckRobotStatus();
        }

        private async Task InitializeRobotsAsync()
        {
            for (int i = 0; i < numberOfRobots; i++)
            {
                await Task.Delay(10);
                Vector3 position = new Vector3(Random.Range(0, width), 0, Random.Range(0, length));
                CellSpace cellSpace = indoorSpace.GetCellSpaceFromCoordinates(position);
                if (cellSpace != null && cellSpace.IsNavigable())
                {
                    GameObject robotObj = Instantiate(robotPrefab, position, Quaternion.identity);
                    RobotController robot = robotObj.GetComponent<RobotController>();
                    if (robot != null)
                    {
                        robot.InitializeRobot(i + 1, indoorSpace, graph);
                        robots.Add(robot);
                    }
                }
            }
        }

        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(robot => robot.IsFree).ToList();
        }

        public RobotController GetClosestFreeRobot(Vector3 position)
        {
            return robots.Where(robot => robot.IsFree)
                        .OrderBy(robot => Vector3.Distance(robot.transform.position, position))
                        .FirstOrDefault();
        }

        private void CheckRobotStatus()
        {
            foreach (var robot in robots)
            {
                if (!robot.IsFree)
                {
                    Debug.Log("Robot " + robot.Id + " is active.");
                }
            }
        }
    }
}