<h1>WB8WFK ARDF Foxoring Transmitter</h1>
<p>This is a simple, inexpensive, transmitter controller for conducting ARDF Foxoring competitions and for demonstrating ARDF principles. The software project is an Arduino script that builds for and runs on most Arduino-like products, including <a href="https://www.adafruit.com/product/2590">AdaFruit's Metro Mini 328</a>, or <a href="https://www.sparkfun.com/products/11113">SparkFun's Arduino Pro Mini 328</a>, and many more.</p>

<p>Controller support for all the common ARDF competition formats is provided: Classic, Sprint, and Foxoring.

<h2>Software</h2>
<p>The software is a proper Arduino script that will open and build without warnings or errors in Arduino IDE version 1.8.12 and later. The Arduino IDE can also be used to program a device that holds a compatible bootloader. Most Arduino products ship with a bootloader pre-installed.</p>

<p>After being programmed, the controller can be configured via a serial port on the board. An Arduino board that includes a USB-to-Serial converter will allow you to configure the controller with a simple USB cable (standard to micro) connected between a USB port on your computer to one on the Arduino board. Less sophisticated Arduino boards lacking a USB-to-Serial converter will require a special FTDI cable like <a href="https://www.adafruit.com/product/70">AdaFruit's FTDI Serial TTL-232 USB Cable</a>, or <a href="https://www.sparkfun.com/products/9718">SparkFun's FTDI Cable 5V</a>.</p>

<p>The controller includes an output pin for controlling a transmitter for sending Morse code characters (high = key down; low = key up) and a separate pin that sends the Morse code as audio tones that can be used to drive a speaker. The controller can also provide a sequence of starting-tones prompting competitors to begin a competition, and can thus serve as an electronic starting "gun" for use in ARDF events.</p>

<h3>Serial Commands</h3>
<p>The controller serial interface operates at 57600 baud that can be accessed using any serial TTY interface program such as PuTTY or Arduino's own Serial Monitor tool. It provides a command prompt > indicating that it is ready to receive recognized the following commands.<p>

<pre><p><b>> CAL [num]</b>  <= <i>* Sets the clock calibration for precise timing</i><br>
<b>> CAL</b>  <= <i>Displays the clock calibration setting</i><br>
<b>> DIP [val]</b>  <= <i>* Sets the competition format, overriding the DIP switch settings<br>
<b>> DIP</b>  <= <i>Displays the competition format setting<br>
<b>> FAC</b>  <= <i>Sets saved EEPROM values to their original defaults</i><br>
<b>> GO</b>  <= <i>Starts operation from zero seconds, equivalent to pressing the sync button</i><br>
<b>> ID [string]</b>  <= <i>* Sets the callsign that gets sent</i><br>
<b>> ID</b>  <= <i>Displays the saved callsign setting</i><br>
<b>> LED [on|off]</b>  <= <i>* Turns on/off LED pin</i><br>
<b>> LED</b>  <= <i>Displays the LED pin setting</i><br>
<b>> RST</b>  <= <i>Resets the processor</i><br>
<b>> SPD ID [num]</b>  <= <i>* Sets the ID code speed in WPM<br>
<b>> SPD</b>  <= <i>Displays the ID code speed setting<br>
<b>> STA [on|off]</b>  <= <i>* Turns on/off the starting tones function</i><br>
<b>> STA</b>  <= <i>Displays the starting tones setting</i><br>
<b>> SYN [on|off]</b>  <= <i>* Turns on/off synchronization using sync button or "GO" command</i><br>
<b>> SYN</b>  <= <i>Displays the synchronization setting</i><br>
<b>> TEM</b>  <= <i>Displays the processor's temperature in C</i><br>
<b>> VER</b>  <= <i>Displays the software version number</i><br></p></pre>
  
  <p>* These values get stored to EEPROM and are retained between power cycles. </p>
<h2>Hardware</h2>
<p>Look in the Hardware folder for all hardware-related documents</p>
<h3>Pinout</h3>
<pre><p><b>PB1 - Board Pin 9  (Output)</b> <= Audio Out (=Gnd when no tone)<br>
<b>PB5 - Board Pin 13 (Output)</b> <= LED On=+V<br>
<b>PD2 - Board Pin 2  (Output)</b> <= Key/PTT On=+V<br>
<b>PD3 - Board Pin 3  (Input)</b>  <= Synchronize (mom. switch to Gnd)<br>
<b>PD4 - Board Pin 4  (Input)</b> <= DIP Switch Bit 0 (switch to Gnd)<br>
<b>PD5 - Board Pin 5  (Input)</b> <= DIP Switch Bit 1 (switch to Gnd)<br>
<b>PD6 - Board Pin 6  (Input)</b> <= DIP Switch Bit 2 (switch to Gnd)</p></pre>
