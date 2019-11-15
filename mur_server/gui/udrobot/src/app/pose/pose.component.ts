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
  private pose_msg: any;
  public x: any;
  public y: any;
  public z: any;
  public pitch: any;
  public roll: any;
  public yaw: any;
  private   q: any;
  private q2:any;
  private   a:any;



  ngOnInit() {
    this.pose_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/pose_gt',
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
        var sinr_cosp = 2 * (this.q.w * this.q.x + this.q.y * this.q.z);
        var cosr_cosp = 1 - 2 * (Math.pow(this.q.x,2) + Math.pow(this.q.y,2));
        this.pitch = Math.atan2(sinr_cosp, cosr_cosp);
        // roll (y-axis rotation)
        var sinp = 2 * (this.q.w * this.q.y - this.q.z * this.q.x);
        if (Math.abs(sinp) >= 1){
          if ((sinp/Math.abs(sinp))>0){
            this.roll = Math.PI/2;
          } // use 90 degrees if out of range
          else{
              this.roll = -Math.PI/2;
            }
          }
          else{
        this.roll = Math.asin(sinp);
        }
        //yaw (z-axis rotation)
        var siny_cosp = 2 * (this.q.w * this.q.z + this.q.x * this.q.y);
        var cosy_cosp = 1 - 2 * (Math.pow(this.q.y,2) + Math.pow(this.q.z,2));
        this.yaw = Math.atan2(siny_cosp, cosy_cosp);
      document.getElementById("X").innerHTML = this.x;
      document.getElementById("Y").innerHTML = this.y;
      document.getElementById("Z").innerHTML = this.z;
      document.getElementById("pitch").innerHTML = this.pitch;
      document.getElementById("roll").innerHTML = this.roll;
      document.getElementById("yaw").innerHTML = this.yaw;
    });
  }
}
