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
  private   q: any;
  private q2:any;
  private   a:any;

  ngOnInit() {
    this.pose_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/odom_filtered',
      messageType: 'nav_msgs/Odometry'
    })
    this.pose_msg.subscribe(function(message){
      this.x = message.pose.pose.position.x;
      this.y = message.pose.pose.position.y;
      this.z = message.pose.pose.position.z;
      this.q = new ROSLIB.Quaternion({
          x : message.pose.pose.orientation.x,
          y : message.pose.pose.orientation.y,
          z : message.pose.pose.orientation.z,
          w : message.pose.pose.orientation.w
        });
      this.vx = message.twist.twist.linear.x;
      this.vy = message.twist.twist.linear.y;
      this.vz = message.twist.twist.linear.z;
      this.ax = message.twist.twist.angular.x;
      this.ay = message.twist.twist.angular.y;
      this.az = message.twist.twist.angular.z;
      // Quaternion to euler
      var sinr_cosp = 2 * (this.q.w * this.q.x + this.q.y * this.q.z);
      var cosr_cosp = 1 - 2 * (Math.pow(this.q.x,2) + Math.pow(this.q.y,2));
      this.pitch = Math.atan2(sinr_cosp, cosr_cosp);
      // roll (y-axis rotation)
      var sinp = 2 * (this.q.w * this.q.y - this.q.z * this.q.x);
      if (Math.abs(sinp) >= 1){
        if ((sinp/Math.abs(sinp))>0){
          this.roll = Math.PI/2;} // use 90 degrees if out of range
        else{
            this.roll = -Math.PI/2;
          }}
      else{
          this.roll = Math.asin(sinp);
      }
      //yaw (z-axis rotation)
      var siny_cosp = 2 * (this.q.w * this.q.z + this.q.x * this.q.y);
      var cosy_cosp = 1 - 2 * (Math.pow(this.q.y,2) + Math.pow(this.q.z,2));
      this.yaw = Math.atan2(siny_cosp, cosy_cosp);
      document.getElementById("X").innerHTML = this.x.toFixed(4);
      document.getElementById("Y").innerHTML = this.y.toFixed(4);
      document.getElementById("Z").innerHTML = this.z.toFixed(4);
      document.getElementById("pitch").innerHTML = this.pitch.toFixed(4);
      document.getElementById("roll").innerHTML = this.roll.toFixed(4);
      document.getElementById("yaw").innerHTML = this.yaw.toFixed(4);
      document.getElementById("vX").innerHTML = this.vx.toFixed(4);
      document.getElementById("vY").innerHTML = this.vy.toFixed(4);
      document.getElementById("vZ").innerHTML = this.vz.toFixed(4);
      document.getElementById("aX").innerHTML = this.ax.toFixed(4);
      document.getElementById("aY").innerHTML = this.ay.toFixed(4);
      document.getElementById("aZ").innerHTML = this.az.toFixed(4);
    });
  }

}
