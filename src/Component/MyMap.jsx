import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, useMap } from 'react-leaflet';
import sensorConfig from '../sensorConfig.json';
import 'leaflet/dist/leaflet.css';
import ROSLIB from 'roslib';

const UpdateMapView = ({ position }) => {
    const map = useMap();
    useEffect(() => {
      if (position) {
        map.setView(position, map.getZoom());
      }
    }, [position, map]);

    return null;
};

const MyMap = () => {
  const [position, setPosition] = useState(null);

  useEffect(() => {
    // Connect to ROSBridge
    const ros = new ROSLIB.Ros({url: 'ws://0.0.0.0:9090'});
    
    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });

    ros.on('error', (error) => {
      console.log('Error connecting to websocket server: ', error);
    });

    ros.on('close', () => {
      console.log('Connection to websocket server closed.');
    });

    // Subscribe to the GPS topic
    const gpsTopic = new ROSLIB.Topic({
      ros: ros,
      name: sensorConfig.GPS.topic,
      messageType: sensorConfig.GPS.messageType
    });

    let dataTimeout;

    gpsTopic.subscribe((message) => {
      const { latitude, longitude } = message;
      setPosition([latitude, longitude]);

      // Reset the timer on new data
      clearTimeout(dataTimeout);
      dataTimeout = setTimeout(() => {
        setPosition(null);
      }, 3000); // Set to null after 3 seconds of no new data
    });

    return () => {
      gpsTopic.unsubscribe();
      clearTimeout(dataTimeout);
      ros.close();
    };
  }, []);

  return (
    <MapContainer center={[37.38402987655305, 126.65707983143997]} zoom={16} style={{ height: "100%", width: "100%" }}>
      <TileLayer
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
      />
      {position && <Marker position={position}></Marker>}
      {position && <UpdateMapView position={position} />}
    </MapContainer>
  );
};

export default MyMap;
