#!/bin/bash

echo "Waiting for CAN nodes to start..."
sleep 2

echo "Activating CAN receiver..."
ros2 lifecycle set /socket_can_receiver_node configure
sleep 0.5
ros2 lifecycle set /socket_can_receiver_node activate

echo "Activating CAN sender..."
ros2 lifecycle set /socket_can_sender_node configure
sleep 0.5
ros2 lifecycle set /socket_can_sender_node activate

echo "CAN nodes activated successfully!"
