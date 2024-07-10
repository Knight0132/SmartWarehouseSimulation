using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class RobotManager : MonoBehaviour
    {
        public GameObject robotPrefab;
        public Transform robotContainer;
        public List<RobotController> robots = new List<RobotController>();
        public int numberOfRobots = 10;
        public IndoorSpace indoorSpace;

        public void start()
        {
            InitializeRobots();
        } 

        void Update()
        {
            UpdateRobotPositions();
        }

        private void InitializeRobots()
        {
            for (int i = 0; i < numberOfRobots; i++)
            {
                GameObject robotObj = Instantiate(robotPrefab, new Vector3(i * 2.0f, 0, 0), Quaternion.identity);
                RobotController robot = robotObj.GetComponent<RobotController>();
                if (robot != null)
                {
                    robot.InitializeRobot();
                }
            }
        }

        // 获取所有处于空闲状态的机器人
        public List<RobotController> GetFreeRobots()
        {
            return robots.Where(robot => robot.IsFree).ToList();
        }

        // 找到最接近指定位置的空闲机器人
        public RobotController GetClosestFreeRobot(Vector3 position)
        {
            return robots.Where(robot => robot.IsFree)
                        .OrderBy(robot => Vector3.Distance(robot.transform.position, position))
                        .FirstOrDefault();
        }

        // 更新所有机器人的位置
        public void UpdateRobotPositions()
        {
            foreach (var robot in robots)
            {
                if (robot.IsMoving)  // 假设 IsMoving 是 RobotController 中的一个属性
                {
                    robot.MoveTowardsTarget();
                }
            }
        }
    }
}