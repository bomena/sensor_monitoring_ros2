import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import '../index.css';
import sensorConfig from '../sensorConfig.json';
import noSignalImage from '../svg/black.png';

let active = false;


const Sensor = () => {
  const [imageSrc1, setImageSrc1] = useState(noSignalImage);
  const [imageSrc2, setImageSrc2] = useState(noSignalImage);
  const [lidarSrc, setLidarSrc] = useState(noSignalImage);
  
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

    const handleImageMessage = (sensorId, message) => {
      const imageUrl = `data:image/jpeg;base64,${message.data}`;
      if (sensorId === 'Color1') {
        setImageSrc1(imageUrl);
      } else if (sensorId === 'Color2') {
        setImageSrc2(imageUrl);
      } else if (sensorId === 'LiDAR_img') {
        setLidarSrc(imageUrl);
        active = true;
      }
    };

    ['Color1', 'Color2', 'LiDAR_img'].forEach(sensorId => {
      const sensorConfigData = sensorConfig[sensorId];
      if (sensorConfigData) {
        const sensorTopic = new ROSLIB.Topic({
          ros: ros,
          name: sensorConfigData.topic,
          messageType: sensorConfigData.messageType
        });

        sensorTopic.subscribe((message) => {
          handleImageMessage(sensorId, message);
        });
      }
    });

    return () => {
      ['Color1', 'Color2', 'LiDAR_img'].forEach(sensorId => {
        const sensorConfigData = sensorConfig[sensorId];
        if (sensorConfigData) {
          const sensorTopic = new ROSLIB.Topic({
            ros: ros,
            name: sensorConfigData.topic,
            messageType: sensorConfigData.messageType
          });
          sensorTopic.unsubscribe();
          if (sensorId === 'Color1') {
            setImageSrc1(noSignalImage);
          } else if (sensorId === 'Color2') {
            setImageSrc2(noSignalImage);
          } else if (sensorId === 'LiDAR_img') {
            setLidarSrc(noSignalImage);
            active = false;
          }
        }
      });
      ros.close();
    };
  }, []);

  const imageStyle = {
    width: '25vw', // Set your desired width
    height: 'auto', // Set your desired height
    margin: '5px'
  };

  const LidarStyle = (isActive) => ({
    width: isActive ? '45vw' : '25vw',
    margin: '5px'
  });

  return (
    <div className='display_grid'>
      <img src={imageSrc1} alt="Color Camera 1" style={imageStyle} />
      <img src={imageSrc2} alt="Color Camera 2" style={imageStyle} />
      <img src={lidarSrc} alt="Lidar" style={LidarStyle(active)} />
    </div>
  );
};

export default Sensor;
