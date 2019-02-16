# Arduino-based-soldering-station
This is a soldering station using Weller RT soldering tips
The project contains 3D model for the housing and a cirquit board using Arduino Nano microcontroller.
The source code is written i C, and uses PID library for temperature control.

<p align="center">
  <img align="center" width="500" src="Soldering%20Station.png">
</p>


This project is based on [ConnyCola](https://github.com/ConnyCola/SolderingStation/tree/master/3D)'s soldering station, but uses a modified source code and 3D model.

The display show the following information:

<img align="left" width="100" height="100" src="SET_ST.png">
The display will change color from Cyan, yellow, and Green dependant on the difference between SET and ACTual temperature. The SET point will be set to 20 degrees if the soldering tip it placed in the holder, or the selector button is pressed once - Standby mode.


<img align="left" width="100" height="100" src="Grapth_ST.png">
The display will show the tempetature curve, repeating approx. every 20 second. To enter the Graph mode, the selector scwith has to bressed for more than 1 second.

At the bottom right of the screen, a small bar showing the PWM output is shown. 

At the mottom left a small dot is show is the soldering station is in Standby mode.

The PCB giles are generated in KiCad, but Gerber files are included as well.

## Credits

The Weller RT product is a trademark of [Weller Tools GmbH](https://www.weller-tools.com/index.html)

The original project is written by [ConnyCola](https://github.com/ConnyCola/SolderingStation/tree/master/3D)

## License

### The MIT License (MIT)
