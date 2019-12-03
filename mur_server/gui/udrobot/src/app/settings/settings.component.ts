import { Component, OnInit, Input } from '@angular/core';
import {coerceNumberProperty} from '@angular/cdk/coercion';
import {MatInputModule} from '@angular/material/input';
import { FormControl, FormBuilder, FormGroup, Validators} from '@angular/forms';
import {MatFormFieldControl} from '@angular/material/form-field';
import { map } from 'rxjs/operators';
import { Breakpoints, BreakpointObserver } from '@angular/cdk/layout';
import {MatSliderModule} from '@angular/material/slider';

declare var ROSLIB: any;
import { ros, DashboardComponent } from '../dashboard/dashboard.component';
import { Setting } from './settings'


@Component({
  selector: 'app-settings',
  templateUrl: './settings.component.html',
  styleUrls: ['./settings.component.css']
})
export class SettingsComponent implements OnInit {
  /** Based on the screen size, switch from standard to one column per row */
  cards = this.breakpointObserver.observe(Breakpoints.Handset).pipe(
    map(({ matches }) => {
      if (matches) {
        return [
          { title: 'Settings', cols: 2, rows: 1 },
          { title: 'Testings', cols: 2, rows: 1 },
        ];
      }

      return [
        { title: 'Settings', cols: 1, rows: 1 },
        { title: 'Testings', cols: 1, rows: 1 },
      ];
    })
  );

  public address: string;
  public port: number;
  private motor_max: number = 1900;
  private motor_min: number = 1100;
  private motor_init: number = 1500;
  public armed_enabled: boolean = false;
  setting: Setting;
  private storageSettingsName = 'roscc2-settings';
  private storageIndexName = 'roscc2-index';
  public armingClient: any;
  public setstream_srv: any;
  public motor_msg:any;
  public motor_1_value: any;
  public motor_2_value: any;
  public motor_3_value: any;
  public motor_4_value: any;

  constructor(private breakpointObserver: BreakpointObserver) { }

  ngOnInit() {
    this.setting = JSON.parse(localStorage.getItem(this.storageSettingsName)) || [ Setting.getDefault() ];
    this.getDefault();
    this.motor_msg = new ROSLIB.Topic({
      ros: ros,
      name: '/mavros/rc/override',
      messageType: 'mavros_msgs/OverrideRCIn',
      throttle_rate: '2'
    });
  }

  getDefault(){
    this.address = '127.0.0.1';
    this.setting[0].address = this.address;
    this.port = 9090;
    this.setting[0].port = this.port;
    localStorage.setItem(this.storageSettingsName, JSON.stringify(this.setting));
    return 0;
  }

  set_server_handle(event: any, robot_address: any, robot_port:any){
    this.address = robot_address.value;
    this.port = robot_port.value;
    this.setting[0].address = this.address;
    this.setting[0].port = this.port;
    localStorage.setItem(this.storageSettingsName, JSON.stringify(this.setting));
    console.log(this.address, this.port);
  }

  set_default_handle(event: any, robot_address: string, robot_port:number){
    this.getDefault()
    console.log("Default config has been set up");
  }

  set_ekf_handle(event: any, x_ekf: any, y_ekf:any, z_ekf:any, pitch_ekf:any, roll_ekf:any, yaw_ekf:any){
    /*this.address = this.getDefault.address;
    this.port = this.getDefault.port;
    console.log(this.address, this.port);*/
    console.log(x_ekf.value,y_ekf.value,z_ekf.value,pitch_ekf.value,roll_ekf.value,yaw_ekf.value);
  }

  get_value_motor(event: any, motor_1: any, motor_2: any, motor_3: any, motor_4: any){
    this.motor_1_value = parseFloat(motor_1.value);
    this.motor_2_value = parseFloat(motor_2.value);
    this.motor_3_value = parseFloat(motor_3.value);
    this.motor_4_value = parseFloat(motor_4.value);
    let motor_values = new ROSLIB.Message({
      channels: [this.motor_1_value,this.motor_2_value,this.motor_3_value,this.motor_4_value,0,0,0,0]
    });
    this.motor_msg.publish(motor_values);
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
      console.log("TESTING "+string_armed+":="+response.success);
    });
    /*Falta el stream rate*/
  }

}
