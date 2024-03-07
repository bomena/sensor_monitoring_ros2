import React, { useState, useEffect } from 'react';
import ROSLIB from 'roslib';


const SensorSyncStatus = () => {
  const [syncStatus, setSyncStatus] = useState(false);

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
      messageType: 'std_msgs/Bool'
    });

    const handleUpdate = (message) => {
      try {
        setSyncStatus(message.data);
      } catch (error) {
        console.error('Error', error);
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
    paddingBottom: '30px',
    backgroundColor: isActive ? 'blue' : 'red',
    border: 'none',
    borderRadius: '5px',
  });

  return (
    <div style={getButtonStyle(syncStatus)}>
      <div style={{ padding: '10px' }}>
        <h1>Sensor Synchronization</h1>
        <h2>(per 5s)</h2>
        <h1 style={{ marginTop: '13px' }}>{syncStatus ? 'Is_Synced : Yes' : 'Is_Synced : NO'}</h1>
        {/* <p style={{ fontSize: '25px' }}>Is Synced: {syncStatus.is_synced ? 'Yes' : 'No'}</p> */}
        {/* <p style={{ fontSize: '25px' }}>Max Time Difference: {syncStatus.max_time_diff.toFixed(3)} s</p> */}
      </div>
    </div>
  );
};

export default SensorSyncStatus;
