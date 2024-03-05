import React from 'react';
import MyMap from './MyMap';
import RosbagRecord from './RosbagRecord';
import DataStreamButtons from './SensorState';
import SensorSyncStatus from './Sync';
import RosbagSize from './FileSize';
import Sensor from './Sensor';
import '../index.css'


const SetViz = (props) => {

  return (
    <>
      <div className="grid">
        <div>
          <RosbagSize />
          <h1>Recording Status</h1>
          <RosbagRecord />
        </div>
        <div className='sensor_sync'>
          <SensorSyncStatus />
        </div>
        <div className='mapOnly'>
          <MyMap />
        </div>
        <div className='sensor_title'>
          <h1>Sensor Status</h1>
        </div>
        <div className='sensor_state'>
          <DataStreamButtons />
        </div>
        <div className='sensor_display'>
          <Sensor />
        </div>
        <div>
          
        </div>
      </div>
    </>   

  )
}

export default SetViz;