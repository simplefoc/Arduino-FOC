# Arduino based Field Oriented Control (FOC) for gimbal motors

This project is based on widely used in Hobby world brushless gimbal controller HMBGC V2.0. 
<p><img src="./Images/ebay.jpg" height="200px"></p>

Proper low cost FOC supporting board is very hard to find these days even may not exist. The reason may be that the hobby community has not yet dug into it properly.

The closest you can get to FOC support and low cost (I was able to find) is:
- [Odrive   ![](https://static1.squarespace.com/static/58aff26de4fcb53b5efd2f02/t/5c2c766921c67c143049cbd3/1546417803031/?format=1200w)](https://odriverobotics.com/)
- [Trinamic   ![](http://i3.ytimg.com/vi/g2BHEdvW9bU/maxresdefault.jpg)](https://www.youtube.com/watch?v=g2BHEdvW9bU)

There are two main probelms with these kinds of borads:
- Both of them cost more than 100$. 
- Both of them are oriented to high current operations.

This porject aims to close the gab and demistify FOC control in a simple way. 

All you need for this project is (an exaple in brackets):
 - Brushless motor - 3 pahse    (IPower GBM4198H-120T [Ebay](https://www.ebay.com/itm/iPower-Gimbal-Brushless-Motor-GBM4108H-120T-for-5N-7N-GH2-ILDC-Aerial-photo-FPV/252025852824?hash=item3aade95398:g:q94AAOSwPcVVo571:rk:2:pf:1&frcectupt=true))
 - Encoder - ( Incremental 2400cpr [Ebay](https://www.ebay.com/itm/600P-R-Photoelectric-Incremental-Rotary-Encoder-5V-24V-AB-2-Phases-Shaft-6mm-New/173145939999?epid=19011022356&hash=item28504d601f:g:PZsAAOSwdx1aKQU-:rk:1:pf:1))
 - HMBGC V2.2 [Ebay](https://www.ebay.com/itm/HMBGC-V2-0-3-Axle-Gimbal-Controller-Control-Plate-Board-Module-with-Sensor/351497840990?hash=item51d6e7695e:g:BAsAAOSw0QFXBxrZ:rk:1:pf:1)

