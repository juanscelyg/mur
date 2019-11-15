import { Component } from '@angular/core';
import { map } from 'rxjs/operators';
import { Breakpoints, BreakpointObserver } from '@angular/cdk/layout';

declare var ROSLIB: any;

export let ros;
let isConnected = false;

@Component({
  selector: 'app-dashboard',
  templateUrl: './dashboard.component.html',
  styleUrls: ['./dashboard.component.css'],
  providers: []
})
export class DashboardComponent {
  /** Based on the screen size, switch from standard to one column per row */
  cards = this.breakpointObserver.observe(Breakpoints.Handset).pipe(
    map(({ matches }) => {
      if (matches) {
        return [
          { title: 'Pose', cols: 2, rows: 1 },
          { title: 'Cameras', cols: 2, rows: 1 },
          { title: 'Navigation', cols: 2, rows: 1 },
          { title: 'Controls', cols: 2, rows: 1 }
        ];
      }

      return [
        { title: 'Pose', cols: 1, rows: 1 },
        { title: 'Cameras', cols: 1, rows: 1 },
        { title: 'Navigation', cols: 1, rows: 1 },
        { title: 'Controls', cols: 1, rows: 1 }
      ];
    })
  );

  constructor(private breakpointObserver: BreakpointObserver) {}
//}

//export class DashboardBehaviour{
  data:{
    rosout: any[],
    //nodes: Node[],
    //globalParameters: Parameter[],
  };
  //activeNode: Node;
  isConnected: boolean;
  //setting: Setting;
  maxConsoleEntries = 200;
  ready = false;
  hasNodes = true;
  onConnectedText: string;
  onConnectedLabel: string;

  ngOnInit(){
    this.isConnected = isConnected;
    this.onConnectedText = 'indeterminate';
    this.onConnectedLabel = 'Disconnected';
    //this.setting = Setting.getCurrent();
    // Load a new Ros connection with a new try every time if it fails
    this.newRosConnection();
    setInterval(()=>{this.newRosConnection();},5000);
    this.data={
      rosout: [],
      //nodes: [],
      //globalParameters: [],
    };
    if (isConnected){
      this.onConnected();
    }
  }
  newRosConnection(): void {
    if (isConnected) {
      return;
    }

    if (ros) {
      ros.close(); // Close old connection
      ros = false;
      return;
    }

    //ros = new ROSLIB.Ros({ url: `ws://${this.setting.address}:${this.setting.port}` });
    ros = new ROSLIB.Ros({ url: `ws://127.0.0.1:9090` });

    ros.on('connection', () => {
      this.onConnected();
      isConnected = true;
      this.isConnected = isConnected;
      this.onConnectedText = 'determinate';
      this.onConnectedLabel = 'Connected';
    });

    ros.on('error', () => {
      isConnected = false;
      this.isConnected = isConnected;
      this.onConnectedText = 'indeterminate';
      this.onConnectedLabel = 'Disconnected';
    });

    ros.on('close', () => {
      isConnected = false;
      this.isConnected = isConnected;
      this.onConnectedText = 'indeterminate';
      this.onConnectedLabel = 'Disconnected';
    });
  }

  onConnected(): void {
  this.data = {
    rosout: [],
    //nodes: [],
    //globalParameters: [],
  };
/*
  this.loadData();

  this.setConsole();
  if (this.setting.battery) {
    this.setBattery();
  }*/
  }
}
