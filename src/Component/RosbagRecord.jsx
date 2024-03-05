import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const RosbagRecord = () => {
  const [isOn, setIsOn] = useState(false);
  const [timer, setTimer] = useState(0);
  const [rosbagStatus, setRosbagStatus] = useState(false);
  const recordPublisher = useRef(null);
  const statusListener = useRef(null);

  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: 'ws://0.0.0.0:9090' });

    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });
    ros.on('error', () => {
      console.log('Error connecting to websocket server.');
    });
    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    recordPublisher.current = new ROSLIB.Topic({
      ros: ros,
      name: '/rosbag_record',
      messageType: 'std_msgs/String'
    });

    statusListener.current = new ROSLIB.Topic({
      ros: ros,
      name: '/rosbag_status',
      messageType: 'std_msgs/Bool'
    });
  }, []);

  useEffect(() => {
    let interval = null;
    if (isOn) {
      interval = setInterval(() => {
        setTimer((prevTimer) => prevTimer + 1);
      }, 1000);

      const message = new ROSLIB.Message({
        data: 'ON'
      });
      recordPublisher.current.publish(message);
    } else {
      clearInterval(interval);
      setTimer(0); // OFF 상태일 때 타이머 리셋

      const message = new ROSLIB.Message({
        data: 'OFF'
      });
      recordPublisher.current.publish(message);
    }
    statusListener.current.subscribe((message) => {
      setRosbagStatus(message.data);
    });
    return () => clearInterval(interval);
  }, [isOn]);

  const formatTime = (totalSeconds) => {
    const hours = Math.floor(totalSeconds / 3600).toString().padStart(2, '0');
    const minutes = Math.floor((totalSeconds % 3600) / 60).toString().padStart(2, '0');
    const seconds = (totalSeconds % 60).toString().padStart(2, '0');
    return `${hours}:${minutes}:${seconds}`;
  };

  const toggleButton = () => {
    setIsOn(!isOn);
  };

  const getButtonStyle = (isActive) => ({
    padding: '10px 20px',
    margin: '15px 0px',
    border: 'none',
    borderRadius: '30px',
    backgroundColor: isActive ? 'blue' : 'red',
    fontSize: '30px',
    color: 'white',
    cursor: 'pointer',
  });

  return (
    <div>
      <button onClick={toggleButton} style={getButtonStyle(rosbagStatus)}>
        <h6>{isOn ? `${formatTime(timer)}` : 'RECORD'}</h6>
      </button>
    </div>
  );
};

export default RosbagRecord;
