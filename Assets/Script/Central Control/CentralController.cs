using System;
using System.Collections.Generic;
using System.Collections;
using System.Linq;
using System.Threading.Tasks;
using NetTopologySuite.Geometries;
using UnityEngine;
using Map;

namespace CentralControl
{
    public class CentralController : MonoBehaviour
    {
        public OrderManager orderManager;
        public RobotManager robotManager;
        public IndoorSpace indoorSpaceProvider;
        public MapLoader mapLoader;
        public float width, length;
        public Graph graph;
        public float interval = 5f;
        public int orderCount = 10;
        public float correctionFactor = 0.1f;

        private bool generatingOrders;
        private List<Order> pendingOrders = new List<Order>();
        private List<Order> unassignedOrders = new List<Order>();
        private readonly object dispatchLock = new object();

        public static event EventHandler<RobotStatusEventArgs> OnRobotBecameFree;

        async void Start()
        {
            await InitializeCentralControllerAsync();
            await InitializeRobotsAsync();
            StartCoroutine(StartGeneratingOrders());

            OnRobotBecameFree += HandleRobotBecameFree;
        }

        private void OnDisable()
        {
            OnRobotBecameFree -= HandleRobotBecameFree;
        }

        private async Task InitializeCentralControllerAsync()
        {
            indoorSpaceProvider = await mapLoader.LoadJsonAsync();
            if (indoorSpaceProvider == null)
            {
                Debug.LogError("Failed to load indoor space data.");
                return;
            }

            width = mapLoader.width;
            length = mapLoader.length;
            graph = await Task.Run(() => mapLoader.GenerateRouteGraph(indoorSpaceProvider));
            if (graph == null)
            {
                throw new System.ArgumentNullException(nameof(graph), "Graph must not be null.");
            }
            Debug.Log("Central controller initialized.");
        }

        private async Task InitializeRobotsAsync()
        {
            await robotManager.InitializeRobotsAsync();
        }

        IEnumerator StartGeneratingOrders()
        {
            generatingOrders = true;
            while (generatingOrders)
            {
                Debug.Log("Starting to generate orders...");
                GenerateRandomOrders();
                DispatchOrders();
                yield return new WaitForSeconds(interval);
            }
        }

        private void GenerateRandomOrders()
        {
            pendingOrders.Clear();

            for (int i = 0; i < orderCount; i++)
            {
                Order newOrder = CreateRandomOrder();
                if (newOrder != null)
                {
                    orderManager.AddOrder(newOrder);
                    pendingOrders.Add(newOrder);
                }
            }
        }

        private Order CreateRandomOrder()
        {
            string id = System.Guid.NewGuid().ToString();
            int randomIndex = UnityEngine.Random.Range(0, indoorSpaceProvider.BusinessPoints.Count);
            CellSpace cellSpace = indoorSpaceProvider.BusinessPoints[randomIndex];
            Vector3 position = new Vector3((float)cellSpace.Space.Centroid.X, 0, (float)cellSpace.Space.Centroid.Y);

            float executionTime = UnityEngine.Random.Range(0f, 30f) * correctionFactor;
            PickingPoint pickingPointCellSpace = indoorSpaceProvider.GetPickingPointFromBusinessPoint(cellSpace);

            if (pickingPointCellSpace == null)
            {
                Debug.LogError($"Failed to get picking point for cell space {cellSpace.Id}");
                return null;
            }

            Order newOrder = new Order(id, position, pickingPointCellSpace, executionTime);
            Debug.Log($"Generated order {id} at {cellSpace.Id} with execution time {executionTime}");
            return newOrder;
        }

        private void DispatchOrders()
        {
            Debug.Log($"Dispatching {pendingOrders.Count} pending orders and {unassignedOrders.Count} unassigned orders.");
            lock (dispatchLock)
            {
                DispatchUnassignedOrders();
                DispatchPendingOrders();
            }
        }

        private void DispatchUnassignedOrders()
        {
            for (int i = unassignedOrders.Count - 1; i >= 0; i--)
            {
                if (TryAssignOrderToRobot(unassignedOrders[i]))
                {
                    unassignedOrders.RemoveAt(i);
                }
            }
        }

        private void DispatchPendingOrders()
        {
            foreach (var order in pendingOrders.ToList())
            {
                if (order.AssignedRobotId != null)
                {
                    Debug.Log($"Order {order.Id} is already assigned to robot {order.AssignedRobotId}. Skipping.");
                    continue;
                }

                if (!TryAssignOrderToRobot(order))
                {
                    unassignedOrders.Add(order);
                    Debug.Log($"Order {order.Id} could not be assigned and added to unassigned orders");
                }
            }
            pendingOrders.Clear();
        }

        private bool TryAssignOrderToRobot(Order order)
        {
            if (order == null || string.IsNullOrEmpty(order.Id))
            {
                Debug.LogError("Invalid order encountered.");
                return false;
            }

            try
            {
                Debug.Log($"Trying to find the closest robot for order {order.Id} with destination {order.Destination}");
                RobotController closestRobot = robotManager.GetClosestRobot(order.Destination);

                if (closestRobot == null)
                {
                    Debug.LogError($"No available or free robot for order {order.Id}.");
                    return false;
                }

                Debug.Log($"Closest robot for order {order.Id} found: {closestRobot.Id}");

                closestRobot.ReceiveOrder(order);
                order.AssignedRobotId = closestRobot.Id;
                Debug.Log($"Order {order.Id} assigned to robot {closestRobot.Id}");
                return true;
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"Error dispatching order {order.Id}: {ex.Message}");
                Debug.LogError($"Stack Trace: {ex.StackTrace}");
                return false;
            }
        }

        public static void NotifyRobotFree(RobotController robot)
        {
            OnRobotBecameFree?.Invoke(null, new RobotStatusEventArgs(robot));
        }

        private void HandleRobotBecameFree(object sender, RobotStatusEventArgs e)
        {
            Debug.Log($"Robot {e.Robot.Id} became free. Attempting to assign unassigned order.");
            AssignOrderToFreeRobot(e.Robot);
        }

        private void AssignOrderToFreeRobot(RobotController robot)
        {
            lock (dispatchLock)
            {
                if (unassignedOrders.Count > 0)
                {
                    var order = unassignedOrders[0];
                    unassignedOrders.RemoveAt(0);
                    if (TryAssignOrderToRobot(order))
                    {
                        Debug.Log($"Unassigned order {order.Id} assigned to newly freed robot {robot.Id}");
                    }
                    else
                    {
                        unassignedOrders.Add(order);
                        Debug.LogError($"Failed to assign order {order.Id} to newly freed robot {robot.Id}");
                    }
                }
                else
                {
                    Debug.Log($"No unassigned orders for newly freed robot {robot.Id}");
                }
            }
        }
    }
}