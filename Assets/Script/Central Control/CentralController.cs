using System.Collections.Generic;
using System.Collections;
using System.Linq;
using NetTopologySuite.Geometries;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class CentralController: MonoBehaviour
    {
        public OrderManager orderManager;
        public RobotManager robotManager;
        public IndoorSpace indoorSpaceProvider;
        public float interval = 5f;
        public int orderCount = 10;

        public void Start()
        {
            StartCoroutine(StartGeneratingOrders());
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
        private void GenerateRandomOrders(IndoorSpace indoorSpaceProvider)
        {
            int attempts = 0;
            int maxAttempts = orderCount * 10;
            int i = 0;
            while (i < orderCount && attempts < maxAttempts)
            {
                string id = System.Guid.NewGuid().ToString();
                Vector3 coordinates = new Vector3(Random.Range(0, 100), 0, Random.Range(0, 100));
                CellSpace cellSpace = indoorSpaceProvider.GetCellSpaceFromCoordinates(coordinates);
                float executionTime = Random.Range(0f, 30f);
                if (cellSpace.IsBusinesspoint())
                {
                    orderManager.AddOrder(id, coordinates, executionTime);
                    Debug.Log($"Generated order: {id} at {coordinates} lasting {executionTime} seconds");
                    i++;
                }
                attempts++;
            }
            if (attempts >= maxAttempts)
            {
                Debug.Log("Reached maximum attempts without fulfilling order count.");
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

