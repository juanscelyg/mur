import { Component, OnInit } from '@angular/core';

import { Setting } from '../settings/settings';

export interface Tile {
  cols: number;
  rows: number;
  text: string;
}

declare var ROSLIB: any;
import { ros, DashboardComponent } from '../dashboard/dashboard.component';

@Component({
  selector: 'app-cameras',
  templateUrl: './cameras.component.html',
  styleUrls: ['./cameras.component.css']
})
export class CamerasComponent implements OnInit {
  constructor() { }

  setting: Setting;
  public link_1: any;
  public topic_name: any = "/mur/mur/camera1/camera_image";
  tiles: Tile[] = [
    {text: 'Imagen', cols: 2, rows: 3},
    {text: 'Controls', cols: 1, rows: 3},
  ];

  ngOnInit() {
    setInterval(()=>{this.newServerConnection();},5000);

  }

  newServerConnection(): void{
    this.set_current_connection();

  }

  set_current_connection(): void{
    this.setting = Setting.getCurrent();
    this.link_1 = "http://"+this.setting[0].address+":9091/stream?topic="+this.topic_name;
  }

  set_topic_handle(event: any, camera_topic: any){
    this.topic_name = camera_topic.value;
    this.set_current_connection();
    console.log(this.topic_name);
    console.log("Camera Topic has been changed");
  }
}
