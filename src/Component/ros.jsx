import { useEffect } from "react";
import ROSLIB from 'roslib';
import L from 'leaflet';

delete L.Icon.Default.prototype._getIconUrl;

L.Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png')
});

function RosConnection() {
  useEffect(() => {
    const ros = new ROSLIB.Ros({ url: "ws://0.0.0.0:9090" });
    ros.on('connection', () => {
      console.log('Connected to websocket server.');
    });
    ros.on('error', () => {
      console.log('Error');
    });
    ros.on('close', () => {
      console.log('Closed to websocket server.');
    });
    return () => {
      ros.close();
    };
  }, []);
}

export default RosConnection;