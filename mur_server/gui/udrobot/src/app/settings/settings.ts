export class Setting {
  address: string;
  port:number;

  static getDefault(): Setting {
    return {
      address: '0.0.0.0',
      port:9090,
    };
  }

  static getCurrent(): Setting {
    const settings = JSON.parse(localStorage.getItem('roscc2-settings')) || [ Setting.getDefault() ];

    return settings;
  }
}
