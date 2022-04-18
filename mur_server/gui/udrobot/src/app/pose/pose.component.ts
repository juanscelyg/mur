import { Component, OnInit } from '@angular/core';

declare var ROSLIB: any;
import { ros, DashboardComponent } from '../dashboard/dashboard.component';

@Component({
  selector: 'app-pose',
  templateUrl: './pose.component.html',
  styleUrls: ['./pose.component.css']
})
export class PoseComponent implements OnInit {
  constructor() { }
  public pose_msg: any;
  public omega_msg: any;
  public depth_msg: any;
  public vel_msg: any;
  public x: any;
  public y: any;
  public z: any;
  public pitch: any;
  public roll: any;
  public yaw: any;
  public vx: any;
  public vy: any;
  public vz: any;
  public ax: any;
  public ay: any;
  public az: any;

  ngOnInit() {
    this.pose_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/angular/orientation',
      messageType: 'geometry_msgs/PointStamped'
    })

    this.omega_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/angular/velocity',
      messageType: 'geometry_msgs/PointStamped'
    })

    this.depth_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/depth/pose',
      messageType: 'geometry_msgs/PoseWithCovarianceStamped'
    })

    this.vel_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/depth/velocity',
      messageType: 'geometry_msgs/TwistStamped'
    })

    this.pose_msg.subscribe(function(msg){
      this.pitch = msg.point.x;
      this.roll = msg.point.y;
      this.yaw = msg.point.z;
      document.getElementById("pitch").innerHTML = this.pitch.toFixed(4);
      document.getElementById("roll").innerHTML = this.roll.toFixed(4);
      document.getElementById("yaw").innerHTML = this.yaw.toFixed(4);
    })

    this.omega_msg.subscribe(function(msg){
      this.ax = msg.point.x;
      this.ay = msg.point.y;
      this.az = msg.point.z;
      document.getElementById("aX").innerHTML = this.ax.toFixed(4);
      document.getElementById("aY").innerHTML = this.ay.toFixed(4);
      document.getElementById("aZ").innerHTML = this.az.toFixed(4);
    })

    this.depth_msg.subscribe(function(msg){
      this.x = msg.pose.pose.position.x;
      this.y = msg.pose.pose.position.y;
      this.z = msg.pose.pose.position.z;
      document.getElementById("X").innerHTML = this.x.toFixed(4);
      document.getElementById("Y").innerHTML = this.y.toFixed(4);
      document.getElementById("Z").innerHTML = this.z.toFixed(4);
    })

    this.vel_msg.subscribe(function(msg){
      this.vx = msg.twist.linear.x;
      this.vy = msg.twist.linear.y;
      this.vz = msg.twist.linear.z;
      document.getElementById("vX").innerHTML = this.vx.toFixed(4);
      document.getElementById("vY").innerHTML = this.vy.toFixed(4);
      document.getElementById("vZ").innerHTML = this.vz.toFixed(4);
    })
  }

}
