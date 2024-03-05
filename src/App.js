import RosConnection from './Component/ros';
import SetViz from './Component/struct';
import './App.css';

function App() {
  return (
    <div className="App">
      <RosConnection />
      <SetViz />
    </div>
  );
}

export default App;