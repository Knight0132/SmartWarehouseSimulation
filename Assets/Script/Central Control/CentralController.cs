using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace CentralControl
{
    public class CentralControl
    {
        private OrderManager orderManager;
        private RobotManager robotManager;
        private float interval = 5f;
        private int orderCount = 10;

        public CentralControl(OrderManager orderManager, RobotManager robotManager)
        {
            this.orderManager = orderManager;
            this.robotManager = robotManager;
        }

        // 设置生成订单的参数
        public void SetOrderGenerationParameters(float interval, int orderCount)
        {
            this.interval = interval;
            this.orderCount = orderCount;
        }

        // 开始生成订单
        public IEnumerator StartGeneratingOrders()
        {
            while (true)
            {
                GenerateRandomOrders();
                yield return new WaitForSeconds(interval);
            }
        }

        // 生成随机订单
        private void GenerateRandomOrders()
        {
            for (int i = 0; i < orderCount; i++)
            {
                string id = System.Guid.NewGuid().ToString();
                Vector3 coordinates = new Vector3(Random.Range(0, 100), 0, Random.Range(0, 100));
                orderManager.AddOrder(id, coordinates);
                Debug.Log($"Generated order: {id} at {coordinates}");
            }
        }

        // 分发订单
        public void DispatchOrders()
        {
            foreach (var order in orderManager.GetAllOrders())
            {
                Robot closestRobot = robotManager.GetClosestFreeRobot(order.Coordinates);
                if (closestRobot != null)
                {
                    closestRobot.AssignOrder(order);
                    Debug.Log($"Order {order.Id} assigned to robot {closestRobot.Id}");
                }
            }
        }

        // 更新机器人的位置信息并进行避障检查
        public void UpdateRobotPositions()
        {
            robotManager.UpdateRobotPositions();
            robotManager.AvoidCollisions();
        }
    }
}

