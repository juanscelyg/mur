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
  public z_d: any;
  public yaw_d: any;
  public status:string = "unknown";
  public input_method_disabled: boolean = true;
  public leak_enabled: boolean = false;
  public armed_enabled: boolean = false;
  public stabilized_flag: boolean = false;
  public altitude_flag: boolean = false;
  public position_flag: boolean = false;


  ngOnInit() {
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
    console.log(event.value);
  }

  set_pose_handle(event: MatButtonModule, x_desired: any, y_desired: any, z_desired: any, yaw_desired: any){
    console.log(x_desired.value);
    console.log(y_desired.value);
    console.log(z_desired.value);
    console.log(yaw_desired.value);
    this.leak_enabled = false;
    this.armed_enabled = true;
  }

  set_z_handle(event: MatButtonModule, z_landing: any){
    console.log(z_landing.value);
    this.leak_enabled = false;
    this.armed_enabled = false;
  }

  land_handle(event: MatButtonModule, z_landing: any){
    console.log(z_landing.value);
  }

  takeoff_handle(event: MatButtonModule){
    console.log(event);
  }

  arm_handle(event: any){
    this.armed_enabled=event.checked;
    this.input_method_disabled = !(this.armed_enabled && this.stabilized_flag);
  }

}
