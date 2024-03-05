import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import RosbagSize from './FileSize';

const RosbagControl = () => {
  const [isRecording, setIsRecording] = useState(false);
  const [startTime, setStartTime] = useState(null);
  const [elapsedTime, setElapsedTime] = useState('00:00:00');
  const [recordPath, setRecordPath] = useState(''); // 사용자 입력을 위한 상태

  const handleRosbagControl = () => {
    if (!isRecording) {
      setStartTime(Date.now());
      setIsRecording(true);
      // 사용자가 입력한 경로 또는 기본 경로 사용
      let path = recordPath || 'Documents';
      let fullPath = '/home/user/' + path;
      handlePublish(fullPath).then(() => {
        handleServiceCall('start');
      });
    } else {
      setIsRecording(false);
      handleServiceCall('stop', '');
      setTimeout(() => {
        setElapsedTime('00:00:00');
      }, 1000); // Reset timer after 1 second
    }
  };
  

  const handleServiceCall = (action) => {
    const ros = new ROSLIB.Ros({ url: 'ws://0.0.0.0:9090' });
    const service = new ROSLIB.Service({
      ros: ros,
      name: '/rosbag_control',
      serviceType: 'std_srvs/SetBool'
    });

    const request = new ROSLIB.ServiceRequest({
      data: action === 'start', // true for start, false for stop
    });

    service.callService(request, (response) => {
      console.log('Response from service: ', response);
    });

  };

  const handlePublish = (path) => {
    return new Promise((resolve, reject) => {
      const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });
      if (!ros) {
        console.log('ROS is not connected');
        reject('ROS is not connected');
      }
  
      const topic = new ROSLIB.Topic({
        ros: ros,
        name: '/path',
        messageType: 'std_msgs/String'
      });
  
      const message = new ROSLIB.Message({
        data: path
      });
  
      topic.publish(message);
      resolve();
    });
  };
  

  useEffect(() => {
    let interval;
    if (isRecording) {
      interval = setInterval(() => {
        const now = Date.now();
        const diff = new Date(now - startTime);
        const formattedTime = diff.toISOString().substr(11, 8);
        setElapsedTime(formattedTime);
      }, 1000);
    }

    return () => clearInterval(interval);
  }, [isRecording, startTime]);

  const getButtonStyle = () => ({
    padding: '10px 20px',
    margin: '5px',
    border: 'none',
    borderRadius: '20px',
    backgroundColor: isRecording ? 'blue' : 'green',
    fontSize: '20px',
    color: 'white',
    cursor: 'pointer',
  });

  return (
    <div>
      <button onClick={handleRosbagControl} style={getButtonStyle()}>
        {isRecording ? elapsedTime : 'Start Record'}
      </button>
      <input
        type="text"
        value={recordPath}
        onChange={(e) => setRecordPath(e.target.value)}
        disabled={isRecording}
        placeholder="Enter Record Path, Default = '~/Documents/'"
        style={{ margin: '15px', padding: '10px', width: '50%', fontSize: '18px' }}
      />
      <RosbagSize />
      
    </div>
  );
};

export default RosbagControl;
