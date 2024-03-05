import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const RosbagSize = () => {
  const [rosbagSize, setRosbagSize] = useState('0 GB');
  const sizeListener = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://0.0.0.0:9090' });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });
    ros.on('error', () => {
      console.log('Error');
    });
    ros.on('close', () => {
      console.log('Closed to websocket server.');
    });

    sizeListener.current = new ROSLIB.Topic({
      ros: ros,
      name: '/rosbag_size',
      messageType: 'std_msgs/String'
    });
  }, []);

  useEffect(() => {
    sizeListener.current.subscribe((message) => {
      setRosbagSize(message.data);
    });
  })

  return (
    <p>{rosbagSize}</p>
  );
};

export default RosbagSize;
