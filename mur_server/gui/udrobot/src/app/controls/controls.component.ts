import { Component, OnInit, Input } from '@angular/core';
import { FormControl, FormBuilder, FormGroup, Validators} from '@angular/forms';
import {MatFormFieldControl} from '@angular/material/form-field';
import { MatRadioChange, MatRadioButton } from '@angular/material';
import {MatButtonModule} from '@angular/material/button';
import {MatInputModule} from '@angular/material/input';
import {MatSlideToggleModule} from '@angular/material'

declare var ROSLIB: any;
import { ros, DashboardComponent } from '../dashboard/dashboard.component';

@Component({
  selector: 'app-controls',
  templateUrl: './controls.component.html',
  styleUrls: ['./controls.component.css']
})

export class ControlsComponent implements OnInit {

  constructor() {}

  public x_d: any;
  public y_d: any;
  public z_d: number = -1;
  public z_n: number;
  public x_value: number = 0;
  public y_value: number = 0;
  public yaw_d: any;
  public z_landing: number = -1.4;
  public status:string = "unknown";
  public state_label:string;
  public input_method_disabled: boolean = true;
  public leak_enabled: boolean = false;
  public armed_enabled: boolean = false;
  public stabilized_flag: boolean = false;
  public altitude_flag: boolean = false;
  public position_flag: boolean = false;
  public armingClient: any;
  public setstream_srv: any;
  public state_msg: any;
  public desired_pose_msg: any;
  public pose_msg: any;

  ngOnInit() {
    this.state_msg = new ROSLIB.Topic({
      ros: ros,
      name: '/mur/state',
      messageType: 'mavros_msgs/State',
      throttle_rate: '20'
    });
    this.desired_pose_msg = new ROSLIB.Topic({
      ros : ros,
      name : '/mur/cmd_pose',
      messageType: 'geometry_msgs/Pose',
      throttle_rate: '20'
    });
    this.pose_msg = new ROSLIB.Topic({
      ros,
      name: '/mur/pose_gt',
      messageType: 'nav_msgs/Odometry'
    });
    this.pose_msg.subscribe(function(message){
      this.z_n = message.pose.pose.position.z;
    });
    window.onkeydown = (event: KeyboardEvent): any =>{
      if (this.stabilized_flag){
        if (event.key=="w"){this.y_value+=0.01;}
        if (event.key=="s"){this.y_value-=0.01;}
        if (event.key=="a"){this.x_value-=0.01;}
        if (event.key=="d"){this.x_value+=0.01;}
        var q_w: any; var q_x: any; var q_y: any; var q_z: any;
        [q_w,q_x,q_y,q_z] = this.get_angle(this.y_value,this.x_value,0);
        let desired_pose = new ROSLIB.Message({
          position:{
            z : this.z_d
          },
          orientation:{
            x : q_x,
            y : q_y,
            z : q_z,
            w : q_w
          }
        });
        let state = new ROSLIB.Message({
          mode: "STABILIZED"
        });
        this.state_msg.publish(state);
        this.desired_pose_msg.publish(desired_pose);
      }
    };
  }

  input_control_radioChange(event: MatRadioChange){
     console.log(event.value);
  }

  mode_radioChange(event: MatRadioChange){
    if (event.value == "STABILIZED"){
      this.stabilized_flag = true;
      this.altitude_flag = false;
      this.position_flag = false;
    }else if(event.value == "ALTCTL")
    {
      this.stabilized_flag = false;
      this.altitude_flag = true;
      this.position_flag = false;
    }else if(event.value == "POSCTL"){
      this.stabilized_flag = false;
      this.altitude_flag = false;
      this.position_flag = true;
    }
    this.input_method_disabled = !(this.armed_enabled && this.stabilized_flag);
    this.state_label = event.value;
    console.log(event.value);
  }

  set_pose_handle(event: MatButtonModule, x_desired: any, y_desired: any, z_desired: any, yaw_desired: any){
    this.x_d = parseFloat(x_desired.value);
    this.y_d = parseFloat(y_desired.value);
    this.z_d = parseFloat(z_desired.value);
    this.yaw_d = parseFloat(yaw_desired.value);
    var q_w: any; var q_x: any; var q_y: any; var q_z: any;
    [q_w,q_x,q_y,q_z] = this.get_angle(0,0,this.yaw_d);
    let desired_pose = new ROSLIB.Message({
      position:{
        x : this.x_d,
        y : this.y_d,
        z : this.z_d
      },
      orientation:{
        x : q_x,
        y : q_y,
        z : q_z,
        w : q_w
      }
    });
    let state = new ROSLIB.Message({
      mode: this.state_label
    });
    this.state_msg.publish(state);
    this.desired_pose_msg.publish(desired_pose);
  }

  set_z_handle(event: MatButtonModule, z_landing: any){
    this.z_landing = parseFloat(z_landing.value);
    console.log("The Z landing has been set. Value:= "+this.z_landing);
  }

  land_handle(event: MatButtonModule, z_landing: any){
    let desired_pose = new ROSLIB.Message({
      position:{
        z : this.z_landing
      }
    });
    let state = new ROSLIB.Message({
      mode: "AUTO.LAND"
    });
    this.state_msg.publish(state);
    this.desired_pose_msg.publish(desired_pose);
  }

  takeoff_handle(event: MatButtonModule){
    let desired_pose = new ROSLIB.Message({
      position:{
        z : this.z_d
      }
    });
    let state = new ROSLIB.Message({
      mode: "AUTO.TAKEOFF"
    });
    this.state_msg.publish(state);
    this.desired_pose_msg.publish(desired_pose);
    console.log("Take Off to:= " + this.z_d);
  }

  arm_handle(event: any){
    this.setstream_srv = new ROSLIB.Service({
      ros: ros,
      name: 'mavros/set_stream_rate',
      serviceType: 'mavros_msgs/StreamRate'
    });
    var request2 = new ROSLIB.ServiceRequest({
      stream_id: 0,
      message_rate: 10,
      on_off: true
    });
    this.setstream_srv.callService(request2, responde =>{});
    this.armingClient = new ROSLIB.Service({
      ros: ros,
      name : '/mavros/cmd/arming',
      serviceType: 'mavros_msgs/CommandBool'
    });
    var request = new ROSLIB.ServiceRequest({
      value: event.checked,
    });
    this.armingClient.callService(request, response =>{
      var  string_armed = "DISARMED";
      if(event.checked){
        string_armed = "ARMED";
      }
      else if(!event.checked){
        string_armed = "DISARMED";
      }
      this.armed_enabled = response.success && event.checked;
      this.input_method_disabled = !(this.armed_enabled && this.stabilized_flag);
      console.log("CONTROL "+string_armed+":="+response.success);
    });

    /*Falta set stream rate y roslaunch roscontrol*/
  }

  get_angle(pitch: number, roll: number, yaw: number){
    var cy = Math.cos((Math.PI*yaw/180) * 0.5);
    var sy = Math.sin((Math.PI*yaw/180) * 0.5);
    var cp = Math.cos(roll * 0.5);
    var sp = Math.sin(roll * 0.5);
    var cr = Math.cos(pitch * 0.5);
    var sr = Math.sin(pitch * 0.5);
    var q_w = cy * cp * cr + sy * sp * sr;
    var q_x = cy * cp * sr - sy * sp * cr;
    var q_y = sy * cp * sr + cy * sp * cr;
    var q_z = sy * cp * cr - cy * sp * sr;
    return [q_w,q_x,q_y,q_z];
  }

}
