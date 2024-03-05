import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';


const SensorSyncStatus = () => {
  const [syncStatus, setSyncStatus] = useState({ is_synced: false, max_time_diff: 0 });

  useEffect(() => {
    const ros = new ROSLIB.Ros({url: 'ws://0.0.0.0:9090'});
    ros.on('error', (error) => {
      console.log('Error connecting to ROS:', error);
    });

    ros.on('connection', () => {
      console.log('Connected to ROS.');
    });

    ros.on('close', () => {
      console.log('Connection to ROS closed.');
    });

    const syncStatusListener = new ROSLIB.Topic({
      ros: ros,
      name: '/sensor_sync_status',
      messageType: 'std_msgs/String'
    });

    const handleUpdate = (message) => {
      try {
        const data = JSON.parse(message.data);
        setSyncStatus(data);
      } catch (error) {
        console.error('Error parsing JSON:', error);
      }
    };

    syncStatusListener.subscribe(handleUpdate);

    return () => {
      if (ros && ros.isConnected) {
        syncStatusListener.unsubscribe();
        ros.close();
      }
    };
  }, []);

  const getButtonStyle = (isActive) => ({
    marginRight: '15px',
    backgroundColor: isActive ? 'blue' : 'red',
    border: 'none',
    borderRadius: '5px',
  });

  return (
    <div style={getButtonStyle(syncStatus.is_synced)}>
      <div style={{ padding: '10px' }}>
        <h1>Sensor Synchronization ( per 5s )</h1>
        <p style={{ fontSize: '25px' }}>Is Synced: {syncStatus.is_synced ? 'Yes' : 'No'}</p>
        <p style={{ fontSize: '25px' }}>Max Time Difference: {syncStatus.max_time_diff.toFixed(3)} s</p>
      </div>
    </div>
  );
};

export default SensorSyncStatus;
// import React, { useState, useEffect } from 'react';
// import ROSLIB from 'roslib';

// const SensorSyncStatus = () => {
//   const [rosConnected, setRosConnected] = useState(false);
//   const [sensorDataTimestamps, setSensorDataTimestamps] = useState({
//     camera1: 0,
//     camera2: 0,
//     lidar: 0,
//   });
//   const [syncStatus, setSyncStatus] = useState({
//     isSynced: false,
//     maxTimeDiff: 0,
//   });

//   useEffect(() => {
//     // Initialize ROS connection
//     const ros = new ROSLIB.Ros({
//       url: 'ws://0.0.0.0:9090', // Change this to your WebSocket server URL
//     });

//     ros.on('connection', () => {
//       console.log('Connected to websocket server.');
//       setRosConnected(true);
//     });

//     ros.on('error', (error) => {
//       console.log('Error connecting to websocket server: ', error);
//     });

//     ros.on('close', () => {
//       console.log('Connection to websocket server closed.');
//       setRosConnected(false);
//     });

//     // Subscribe to ROS topics
//     const lidarTopic = new ROSLIB.Topic({
//       ros,
//       name: '/ouster/points',
//       messageType: 'sensor_msgs/PointCloud2',
//     });

//     const camera1Topic = new ROSLIB.Topic({
//       ros,
//       name: '/cam1/image_color/compressed',
//       messageType: 'sensor_msgs/CompressedImage',
//     });

//     const camera2Topic = new ROSLIB.Topic({
//       ros,
//       name: '/cam2/image_color/compressed',
//       messageType: 'sensor_msgs/CompressedImage',
//     });

//     const updateTimestamp = (sensor) => {
//       setSensorDataTimestamps((prevState) => ({
//         ...prevState,
//         [sensor]: Date.now(),
//       }));
//     };

//     lidarTopic.subscribe(() => {
//       updateTimestamp('lidar');
//     });

//     camera1Topic.subscribe(() => {
//       updateTimestamp('camera1');
//     });

//     camera2Topic.subscribe(() => {
//       updateTimestamp('camera2');
//     });

//     // Cleanup function
//     return () => {
//       lidarTopic.unsubscribe();
//       camera1Topic.unsubscribe();
//       camera2Topic.unsubscribe();
//       ros.close();
//     };
//   }, []);

//   useEffect(() => {
//     const checkSync = () => {
//       const timestamps = Object.values(sensorDataTimestamps);
//       const maxTimeDiff = Math.max(...timestamps) - Math.min(...timestamps);

//       setSyncStatus({
//         isSynced: maxTimeDiff < 50, // Tune this threshold as needed
//         maxTimeDiff,
//       });
//     };

//     const intervalId = setInterval(checkSync, 5000); // Check sync every 5 seconds

//     return () => clearInterval(intervalId);
//   }, [sensorDataTimestamps]);

//   return (
//     <div>
//       <h2>Sensor Synchronization Status</h2>
//       <p>ROS Connection: {rosConnected ? 'Connected' : 'Disconnected'}</p>
//       <p>Camera 1 Timestamp: {sensorDataTimestamps.camera1}</p>
//       <p>Camera 2 Timestamp: {sensorDataTimestamps.camera2}</p>
//       <p>Lidar Timestamp: {sensorDataTimestamps.lidar}</p>
//       <p>Is Synced: {syncStatus.isSynced ? 'Yes' : 'No'}</p>
//       <p>Max Time Difference: {syncStatus.maxTimeDiff}ms</p>
//     </div>
//   );
// };

// export default SensorSyncStatus;
