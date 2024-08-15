using System;
using UnityEngine;

namespace CentralControl
{
    public class RobotStatusEventArgs : EventArgs
    {
        public RobotController Robot { get; private set; }
        public RobotStatusEventArgs(RobotController robot)
        {
            Robot = robot;
        }
    }

    public class RobotOrderCompletedEventArgs : EventArgs
    {
        public RobotController Robot { get; private set; }
        public Order CompletedOrder { get; private set; }
        public RobotOrderCompletedEventArgs(RobotController robot, Order completedOrder)
        {
            Robot = robot;
            CompletedOrder = completedOrder;
        }
    }
}