import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { PoseComponent } from './pose.component';

describe('PoseComponent', () => {
  let component: PoseComponent;
  let fixture: ComponentFixture<PoseComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ PoseComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(PoseComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
