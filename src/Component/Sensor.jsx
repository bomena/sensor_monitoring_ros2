import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import '../index.css';
import sensorConfig from '../sensorConfig.json';
import noSignalImage from '../svg/black.png';

const Sensor = () => {
  const [imageSrc1, setImageSrc1] = useState(noSignalImage);
  const [imageSrc2, setImageSrc2] = useState(noSignalImage);
  const [lidarSrc1, setLidarSrc1] = useState(noSignalImage);
  const [lidarSrc2, setLidarSrc2] = useState(noSignalImage);
  const [lidarSrc3, setLidarSrc3] = useState(noSignalImage);
  
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
      } else if (sensorId === 'LiDAR_img1') {
        setLidarSrc1(imageUrl);
        // active = true;
      } else if (sensorId === 'LiDAR_img2') {
        setLidarSrc2(imageUrl);
        // active = true;
      } else if (sensorId === 'LiDAR_img3') {
        setLidarSrc3(imageUrl);
        // active = true;
      }
    };

    ['Color1', 'Color2', 'LiDAR_img1', 'LiDAR_img2', 'LiDAR_img3'].forEach(sensorId => {
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
      ['Color1', 'Color2', 'LiDAR_img1', 'LiDAR_img2', 'LiDAR_img3'].forEach(sensorId => {
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
          } else if (sensorId === 'LiDAR_img1') {
            setLidarSrc1(noSignalImage);
            // active = false;
          } else if (sensorId === 'LiDAR_img2') {
            setLidarSrc2(noSignalImage);
            // active = false;
          } else if (sensorId === 'LiDAR_img3') {
            setLidarSrc3(noSignalImage);
            // active = false;
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

  const LidarStyle = {
    width: '15vw',
    margin: '5px'
  };

  return (
    <div className='display_grid'>
      <img src={imageSrc1} alt="Color Camera 1" style={imageStyle} />
      <img src={imageSrc2} alt="Color Camera 2" style={imageStyle} />
      <img src={lidarSrc1} alt="Lidar" style={LidarStyle} />
      <img src={lidarSrc2} alt="Lidar" style={LidarStyle} />
      <img src={lidarSrc3} alt="Lidar" style={LidarStyle} />
    </div>
  );
};

export default Sensor;
