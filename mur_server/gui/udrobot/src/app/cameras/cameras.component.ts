import { Component, OnInit } from '@angular/core';

import { Setting } from '../settings/settings';

@Component({
  selector: 'app-cameras',
  templateUrl: './cameras.component.html',
  styleUrls: ['./cameras.component.css']
})
export class CamerasComponent implements OnInit {

  constructor() { }

  setting: Setting;
  public link_1: any;
  public link_2: any;

  ngOnInit() {
    setInterval(()=>{this.newServerConnection();},5000);
  }

  newServerConnection(): void{
    this.setting = Setting.getCurrent();
    this.link_1 = "http://"+this.setting[0].address+":9091/stream?topic=/mur/camera1/image_raw";
    this.link_2 = "http://"+this.setting[0].address+":9091/stream?topic=/mur/camera2/image_raw";
  }

}
