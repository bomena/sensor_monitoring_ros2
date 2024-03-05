import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import sensorConfig from '../sensorConfig.json';

const DataStreamButtons = () => {
  const [sensorData, setSensorData] = useState(
    Object.entries(sensorConfig).reduce((acc, [key]) => {
      acc[key] = { active: false, lastUpdate: 0 };
      return acc;
    }, {})
  );

  useEffect(() => {
    const ros = new ROSLIB.Ros({url: 'ws://0.0.0.0:9090'});
    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });
    ros.on('error', () => {
      console.log('Error');
    });
    ros.on('close', () => {
      console.log('Closed to websocket server.');
    });
    Object.entries(sensorConfig).forEach(([key, { topic, messageType }]) => {
      const sensorTopic = new ROSLIB.Topic({
        ros: ros,
        name: topic,
        messageType: messageType
      });

      sensorTopic.subscribe(() => {
        setSensorData(prevData => ({
          ...prevData,
          [key]: { active: true, lastUpdate: Date.now() }
        }));
      });
    });

    const interval = setInterval(() => {
      const now = Date.now();
      setSensorData(prevData => {
        const newData = { ...prevData };
        Object.keys(newData).forEach(key => {
          if (newData[key].active && now - newData[key].lastUpdate > 2000) {
            newData[key].active = false;
          }
        });
        return newData;
      });
    }, 1000);

    // Clean up the subscriptions and interval when the component unmounts
    return () => {
      clearInterval(interval);
      Object.entries(sensorConfig).forEach(({ topic }) => {
        const sensorTopic = new ROSLIB.Topic({
          ros: ros,
          name: topic,
          messageType: 'sensor_msgs/CompressedImage'
        });
        sensorTopic.unsubscribe();
      });
      ros.close();
    };
  }, []);

  const getButtonStyle = (isActive) => ({
    padding: '15px 25px',
    margin: '5px',
    border: 'none',
    borderRadius: '5px',
    backgroundColor: isActive ? 'blue' : 'black',
    fontSize: '30px',
    color: 'white',
    cursor: 'pointer',
  });

  return (
    <div>
      {Object.entries(sensorData).map(([key, data]) => (
        <button key={key} style={getButtonStyle(data.active)}>
          {key}
        </button>
      ))}
    </div>
  );
};

export default DataStreamButtons;
