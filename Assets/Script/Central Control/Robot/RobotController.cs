using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using NetTopologySuite.Geometries;
using PathPlanning;
using Map;

namespace CentralControl
{
    public class RobotController : MonoBehaviour
    {
        public MapLoader mapLoader;
        public Transform targetIndicator;
        public float defaultSpeed = 5.0f;
        public int Id { get; private set; }
        public Graph graph { get; private set; }
        public IndoorSpace indoorSpace {get; private set; }

        private bool isMoving = false;
        private Queue<Order> ordersQueue = new Queue<Order>();

        void Update()
        {
            if (!isMoving && ordersQueue.Count > 0)
            {
                StartNextOrder();
            }
        }

        public void InitializeRobot(int id, IndoorSpace indoorSpace, Graph graph)
        {
            this.Id = id;
            this.indoorSpace = indoorSpace;
            this.graph = graph; 
        }

        public void ReceiveOrder(Order order)
        {
            ordersQueue.Enqueue(order);
            Debug.Log($"Robot {Id} received order {order.Id}");
            if (!isMoving)
            {
                StartNextOrder();
            }
        }

        private void StartNextOrder()
        {
            if (ordersQueue.Count > 0)
            {
                Order order = ordersQueue.Dequeue();
                Debug.Log($"Robot {Id} starts order {order.Id}");
                ExecuteOrder(order);
            }
        }

        private void ExecuteOrder(Order order)
        {
            Vector3 targetPosition = order.Destination;
            SetTargetIndicator(targetPosition);
            isMoving = true;
            Debug.Log($"Robot {Id} executing order {order.Id} to {targetPosition}");
            StartCoroutine(MoveToPosition(targetPosition, order.ExecutionTime));
        }

        IEnumerator MoveToPosition(Vector3 target, float executionTime)
        {
            while (Vector3.Distance(transform.position, target) > 0.1f)
            {
                transform.position = Vector3.MoveTowards(transform.position, target, defaultSpeed * Time.deltaTime);
                yield return null;
            }
            
            yield return new WaitForSeconds(executionTime);
            
            isMoving = false;
            Debug.Log($"Order completed and executed for {executionTime} seconds at {target}");
        }

        public void SetTargetIndicator(Vector3 position)
        {
            if (targetIndicator != null)
            {
                targetIndicator.position = position;
            }
        }

        public bool IsFree
        {
            get { return !isMoving && ordersQueue.Count == 0; }
        }

        public bool IsMoving
        {
            get { return isMoving; }
        }
    }
}
