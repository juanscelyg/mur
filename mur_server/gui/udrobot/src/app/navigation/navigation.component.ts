import { Component, OnInit } from '@angular/core';

declare var ROS3D: any;
declare var ROSLIB: any;

import { ros, DashboardComponent } from '../dashboard/dashboard.component';

@Component({
  selector: 'app-navigation',
  templateUrl: './navigation.component.html',
  styleUrls: ['./navigation.component.css']
})
export class NavigationComponent implements OnInit {

  constructor() { }

  ngOnInit() {
    // Create the main viewer.
    var viewer = new ROS3D.Viewer({
      divID: 'rviz',
      width: 566,
      height: 320,
      antialias: true
    });

    // Add a grid.
    viewer.addObject(new ROS3D.Grid());

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
      ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10.0,
      fixedFrame: 'world'
    });

    // Setup the URDF client.
    var urdfClient = new ROS3D.UrdfClient({
      ros,
      param : '/mur/robot_description',
      tfClient : tfClient,
      //path : 'http://localhost:9090/',
      rootObject : viewer.scene,
    });

    var laserClient = new ROS3D.LaserScan({
      ros,
      tfClient: tfClient,
      rootObject: viewer.scene,
      topic: '/mur/sonar',
      material: { size: 0.2, color: 0xff0000 }
  });

  }

}
